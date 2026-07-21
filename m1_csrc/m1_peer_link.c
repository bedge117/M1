/* See COPYING.txt for license details. */
/*
 * m1_peer_link.c — M1<->M1 peer link over ESP-NOW (see m1_peer_link.h).
 *
 * Discover nearby M1 units and BEAM SAVED FILES between them. The ESP32-C6 does
 * the ESP-NOW radio work (m1_esp_client_now_*); this drives it and layers a tiny
 * chunked file-transfer protocol on top of the message pipe.
 *
 * Transfer sub-protocol — first byte of each ESP-NOW DATA payload is a tag:
 *   PL_TAG_MSG (0) : [text]                          — chat (display only)
 *   PL_TAG_META(1) : [nameLen:1][name][size:4 LE]    — start of a file
 *   PL_TAG_DATA(2) : [seq:2 LE][bytes]               — a chunk (in order)
 *   PL_TAG_END (3) : [crc32:4 LE]                    — end; receiver checks size + CRC
 * A dropped/reordered chunk is detected by the seq check and aborts the file.
 * The END CRC32 (whole-file, computed as sent/received) catches silent corruption
 * that the size check can't — important for larger files. A legacy END with no
 * CRC (len < 5) falls back to a size-only check.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_system.h"
#include "m1_display.h"
#include "m1_file_browser.h"
#include "m1_esp_client.h"
#include "m1_peer_link.h"
#include "ff.h"

#define PL_CHANNEL     1
#define PL_MAX_PEERS   8
#define PL_POLL_MS     800          /* announce + peer refresh cadence */
#define PL_CHUNK       220          /* file bytes per ESP-NOW packet */
#define PL_RX_DIR      "0:/Received"

enum { PL_TAG_MSG = 0, PL_TAG_META = 1, PL_TAG_DATA = 2, PL_TAG_END = 3,
       PL_TAG_ACK = 4 };

/* ACK payload = [PL_TAG_ACK][phase][status]. Receiver -> sender so the sender
 * knows a peer is actually receiving (not blasting into the void). */
enum { PL_ACK_META = 0, PL_ACK_DONE = 1, PL_ACK_PROG = 2 };  /* phase */
enum { PL_ST_OK = 0, PL_ST_ERR = 1, PL_ST_CRC = 1, PL_ST_SIZE = 2 };  /* status */

#define PL_ACK_EVERY       32     /* receiver progress-ACKs every N chunks */
#define PL_PEER_TIMEOUT_MS 5000   /* sender aborts if no progress ACK for this long */

static const uint8_t PL_BCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* ---- incoming-file reassembly state ---- */
static struct {
    bool     active;
    FIL      f;
    char     name[40];
    uint32_t expected;
    uint32_t received;
    uint16_t next_seq;
    uint32_t crc;           /* running CRC32 (pre-final XOR) over written bytes */
} s_rx;
static char s_rx_status[28];        /* one-line status shown on the screen */

/* Shared ESP-drain scratch, sized to a full ESP drain (M1_RPC_MAX_PAYLOAD=4082);
 * see pl_drain_rx. File-scope + single-caller-per-task, so both pl_drain_rx and
 * pl_wait_ack reuse it (never concurrently — same peer-link task). */
static uint8_t s_drain_buf[4096];

/* Send a small ACK back to `mac`. */
static void pl_send_ack(const uint8_t *mac, uint8_t phase, uint8_t status)
{
    uint8_t ack[3] = { PL_TAG_ACK, phase, status };
    m1_esp_client_now_send(mac, ack, sizeof(ack));
}

/* Software CRC32 (reflected, poly 0xEDB88320 — standard zlib/PKZIP). Self-contained
 * so it never touches the STM32 hardware CRC unit, which the m1_link transport may
 * be using concurrently during a transfer. Incremental: seed 0xFFFFFFFF, feed
 * chunks, final result is (crc ^ 0xFFFFFFFF). Sender and receiver only need to
 * agree on the algorithm, not match the firmware-image CRC. */
static uint32_t pl_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
}

static void rx_abort(void)
{
    if (s_rx.active) f_close(&s_rx.f);
    s_rx.active = false;
}

/* Keep only a safe leaf filename (defend against path separators). */
static void sanitize_name(char *n)
{
    for (char *p = n; *p; ++p) {
        if (*p == '/' || *p == '\\' || *p == ':') *p = '_';
    }
}

/* Process one received DATA payload (tag + body). */
static void rx_feed(const uint8_t *from, const uint8_t *p, int len)
{
    if (len < 1) return;

    switch (p[0]) {
    case PL_TAG_MSG: {
        int n = len - 1;
        if (n > (int)sizeof(s_rx_status) - 8) n = sizeof(s_rx_status) - 8;
        char msg[24];
        if (n < 0) n = 0;
        memcpy(msg, p + 1, n); msg[n] = '\0';
        snprintf(s_rx_status, sizeof(s_rx_status), "<%02X%02X %s", from[4], from[5], msg);
        break;
    }
    case PL_TAG_META: {
        if (len < 2) return;
        int nl = p[1];
        if (nl > 30) nl = 30;
        if (len < 2 + nl + 4) return;
        rx_abort();

        char nm[40];
        memcpy(nm, p + 2, nl); nm[nl] = '\0';
        sanitize_name(nm);

        s_rx.expected = (uint32_t)p[2 + nl] | ((uint32_t)p[3 + nl] << 8) |
                        ((uint32_t)p[4 + nl] << 16) | ((uint32_t)p[5 + nl] << 24);

        f_mkdir(PL_RX_DIR);
        char path[64];
        snprintf(path, sizeof(path), "%s/%s", PL_RX_DIR, nm);
        if (f_open(&s_rx.f, path, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
            s_rx.active   = true;
            s_rx.next_seq = 0;
            s_rx.received = 0;
            s_rx.crc      = 0xFFFFFFFFu;
            strncpy(s_rx.name, nm, sizeof(s_rx.name) - 1);
            s_rx.name[sizeof(s_rx.name) - 1] = '\0';
            snprintf(s_rx_status, sizeof(s_rx_status), "Recv %s 0%%", nm);
            pl_send_ack(from, PL_ACK_META, PL_ST_OK);      /* "I'm receiving" */
        } else {
            pl_send_ack(from, PL_ACK_META, PL_ST_ERR);     /* can't open target */
        }
        break;
    }
    case PL_TAG_DATA: {
        if (!s_rx.active || len < 3) return;
        uint16_t seq = (uint16_t)(p[1] | (p[2] << 8));
        int16_t  d   = (int16_t)(seq - s_rx.next_seq);   /* signed: handles wrap */
        if (d < 0) {
            /* Duplicate of an already-written chunk (ESP-NOW MAC-layer retry after
             * a lost ACK re-delivers a frame we already have). Ignore — not an error. */
            break;
        }
        if (d > 0) {
            /* Forward gap = a chunk was truly lost. Writing this later chunk at the
             * current file offset would corrupt the file, so abort. Shows the seq
             * pair for diagnosis (should not happen now the drain buffer is sized to
             * the full ESP payload). */
            snprintf(s_rx_status, sizeof(s_rx_status), "drop s%u!=%u", seq, s_rx.next_seq);
            rx_abort();
            return;
        }
        UINT bw = 0;
        f_write(&s_rx.f, p + 3, len - 3, &bw);
        s_rx.crc = pl_crc32_update(s_rx.crc, p + 3, (uint32_t)(len - 3));
        s_rx.received += (uint32_t)(len - 3);
        s_rx.next_seq++;
        /* Periodic liveness ACK so the sender knows we're still here. */
        if ((s_rx.next_seq % PL_ACK_EVERY) == 0)
            pl_send_ack(from, PL_ACK_PROG, PL_ST_OK);
        int pct = s_rx.expected ? (int)((uint64_t)100 * s_rx.received / s_rx.expected) : 0;
        snprintf(s_rx_status, sizeof(s_rx_status), "Recv %s %d%%", s_rx.name, pct);
        break;
    }
    case PL_TAG_END: {
        if (!s_rx.active) return;
        f_close(&s_rx.f);
        s_rx.active = false;
        uint32_t local = s_rx.crc ^ 0xFFFFFFFFu;
        uint8_t st;
        if (s_rx.received != s_rx.expected) {
            snprintf(s_rx_status, sizeof(s_rx_status), "RX err (size)");
            st = PL_ST_SIZE;
        } else if (len >= 5) {
            /* END carries the sender's whole-file CRC32 (LE). */
            uint32_t rcrc = (uint32_t)p[1] | ((uint32_t)p[2] << 8) |
                            ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24);
            bool good = (rcrc == local);
            snprintf(s_rx_status, sizeof(s_rx_status),
                     good ? "Got %s" : "RX err (crc)", s_rx.name);
            st = good ? PL_ST_OK : PL_ST_CRC;
        } else {
            /* Legacy sender: no CRC in END — size-only check. */
            snprintf(s_rx_status, sizeof(s_rx_status), "Got %s", s_rx.name);
            st = PL_ST_OK;
        }
        pl_send_ack(from, PL_ACK_DONE, st);      /* tell sender the outcome */
        break;
    }
    default: break;
    }
}

/* Drain everything queued on the ESP into rx_feed(). */
static void pl_drain_rx(void)
{
    for (int guard = 0; guard < 40; guard++) {
        int r = m1_esp_client_now_recv(s_drain_buf, sizeof(s_drain_buf));
        if (r < 1 || s_drain_buf[0] == 0) break;
        int count = s_drain_buf[0], off = 1;
        for (int i = 0; i < count; i++) {
            if (off + 8 > r) break;
            uint8_t fmac[6];
            memcpy(fmac, &s_drain_buf[off], 6); off += 6;
            int len = s_drain_buf[off] | (s_drain_buf[off + 1] << 8); off += 2;
            if (off + len > r) break;
            rx_feed(fmac, &s_drain_buf[off], len);
            off += len;
        }
        if (count < 4) break;   /* ring likely drained */
    }
}

/* Sender-side: one non-blocking pass — drain the ESP RX once and return the
 * status byte of a PL_TAG_ACK of `phase` from `mac` if present, else -1. Reads
 * the full drain buffer so nothing is lost off the ESP ring. */
static int pl_poll_ack(const uint8_t *mac, uint8_t phase)
{
    int r = m1_esp_client_now_recv(s_drain_buf, sizeof(s_drain_buf));
    if (r < 1 || s_drain_buf[0] == 0) return -1;
    int count = s_drain_buf[0], off = 1, found = -1;
    for (int i = 0; i < count; i++) {
        if (off + 8 > r) break;
        uint8_t fmac[6];
        memcpy(fmac, &s_drain_buf[off], 6); off += 6;
        int len = s_drain_buf[off] | (s_drain_buf[off + 1] << 8); off += 2;
        if (off + len > r) break;
        const uint8_t *p = &s_drain_buf[off];
        if (len >= 3 && p[0] == PL_TAG_ACK && p[1] == phase && memcmp(fmac, mac, 6) == 0)
            found = p[2];
        off += len;
    }
    return found;
}

/* Sender-side: poll for a PL_TAG_ACK of `phase` from `mac`, up to timeout_ms.
 * Returns the status byte (>=0), or -1 on timeout. */
static int pl_wait_ack(const uint8_t *mac, uint8_t phase, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    do {
        int s = pl_poll_ack(mac, phase);
        if (s >= 0) return s;
        HAL_Delay(15);
    } while ((HAL_GetTick() - start) < timeout_ms);
    return -1;
}

/* Sender-side: non-blocking Back check during a send (Back = cancel). */
static bool pl_send_cancelled(void)
{
    S_M1_Main_Q_t       q;
    S_M1_Buttons_Status b;
    if (xQueueReceive(main_q_hdl, &q, 0) != pdTRUE) return false;
    if (q.q_evt_type != Q_EVENT_KEYPAD) return false;
    if (xQueueReceive(button_events_q_hdl, &b, 0) != pdTRUE) return false;
    return b.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK;
}

/* ---- sender ---- */

static void pl_draw_send(const char *name, int pct)
{
    m1_u8g2_firstpage();
    do {
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 0, 12, "Beaming file...");
        char l[28];
        snprintf(l, sizeof(l), "%.20s", name);
        u8g2_DrawStr(&m1_u8g2, 0, 26, l);
        snprintf(l, sizeof(l), "%d%%", pct);
        u8g2_DrawStr(&m1_u8g2, 0, 42, l);
        u8g2_DrawFrame(&m1_u8g2, 0, 46, 128, 8);
        u8g2_DrawBox(&m1_u8g2, 1, 47, (pct * 126) / 100, 6);
    } while (m1_u8g2_nextpage());
}

static void peer_send_file(const uint8_t *mac, const char *path)
{
    FIL  f;
    UINT br;

    if (f_open(&f, path, FA_READ) != FR_OK) {
        m1_message_box(&m1_u8g2, "Peer Link", "Open", "failed", " OK ");
        return;
    }
    uint32_t size = f_size(&f);

    const char *nm = strrchr(path, '/');
    nm = nm ? nm + 1 : path;
    int nl = (int)strlen(nm);
    if (nl > 30) nl = 30;

    /* META */
    uint8_t meta[2 + 30 + 4];
    meta[0] = PL_TAG_META;
    meta[1] = (uint8_t)nl;
    memcpy(&meta[2], nm, nl);
    meta[2 + nl] = (uint8_t)size;
    meta[3 + nl] = (uint8_t)(size >> 8);
    meta[4 + nl] = (uint8_t)(size >> 16);
    meta[5 + nl] = (uint8_t)(size >> 24);

    /* Flush any stale RX so a leftover ACK can't false-positive the handshake. */
    for (int g = 0; g < 40; g++) {
        int r = m1_esp_client_now_recv(s_drain_buf, sizeof(s_drain_buf));
        if (r < 1 || s_drain_buf[0] == 0) break;
    }

    /* Handshake: only proceed if a peer is actually receiving. ESP-NOW send is
     * fire-and-forget, so without this the sender would "beam" into the void. */
    m1_esp_client_now_send(mac, meta, (uint16_t)(6 + nl));
    int a = pl_wait_ack(mac, PL_ACK_META, 1500);
    if (a < 0) {
        f_close(&f);
        m1_message_box(&m1_u8g2, "Peer Link", "No peer", "receiving", " OK ");
        return;
    }
    if (a != PL_ST_OK) {
        f_close(&f);
        m1_message_box(&m1_u8g2, "Peer Link", "Peer can't", "receive", " OK ");
        return;
    }

    /* DATA */
    uint16_t seq = 0;
    uint32_t sent = 0;
    uint32_t crc = 0xFFFFFFFFu;   /* whole-file CRC32, sent in END */
    uint32_t last_alive = HAL_GetTick();   /* last progress ACK from receiver */
    bool ok = true, cancelled = false, peer_lost = false;
    uint8_t pkt[3 + PL_CHUNK];
    while (1) {
        if (f_read(&f, &pkt[3], PL_CHUNK, &br) != FR_OK) { ok = false; break; }
        if (br == 0) break;
        pkt[0] = PL_TAG_DATA;
        pkt[1] = (uint8_t)seq;
        pkt[2] = (uint8_t)(seq >> 8);
        if (!m1_esp_client_now_send(mac, pkt, (uint16_t)(3 + br))) { ok = false; break; }
        crc = pl_crc32_update(crc, &pkt[3], br);
        seq++;
        sent += br;
        pl_draw_send(nm, size ? (int)((uint64_t)100 * sent / size) : 100);
        HAL_Delay(20);            /* pace so the receiver keeps up */

        if (pl_send_cancelled()) { cancelled = true; break; }   /* Back = cancel */

        /* Liveness: the receiver progress-ACKs every PL_ACK_EVERY chunks. If we
         * hear nothing for PL_PEER_TIMEOUT_MS, it left — stop beaming into the void. */
        if ((seq % PL_ACK_EVERY) == 0) {
            if (pl_poll_ack(mac, PL_ACK_PROG) >= 0) last_alive = HAL_GetTick();
            else if ((HAL_GetTick() - last_alive) > PL_PEER_TIMEOUT_MS) {
                peer_lost = true; break;
            }
        }
    }
    f_close(&f);

    if (cancelled) {
        m1_message_box(&m1_u8g2, "Peer Link", "Send", "cancelled", " OK ");
        return;
    }
    if (peer_lost) {
        m1_message_box(&m1_u8g2, "Peer Link", "Peer left,", "send stopped", " OK ");
        return;
    }
    if (!ok) {
        m1_message_box(&m1_u8g2, "Peer Link", "Send failed", nm, " OK ");
        return;
    }

    uint32_t fcrc = crc ^ 0xFFFFFFFFu;
    uint8_t end[5] = { PL_TAG_END,
                       (uint8_t)fcrc, (uint8_t)(fcrc >> 8),
                       (uint8_t)(fcrc >> 16), (uint8_t)(fcrc >> 24) };
    m1_esp_client_now_send(mac, end, sizeof(end));

    /* Wait for the receiver to confirm the completed file (size + CRC). */
    int e = pl_wait_ack(mac, PL_ACK_DONE, 4000);
    const char *l1, *l2;
    if      (e < 0)          { l1 = "Sent, no";   l2 = "confirm"; }   /* peer left mid-send */
    else if (e == PL_ST_OK)  { l1 = "Sent OK";    l2 = nm;        }
    else if (e == PL_ST_CRC) { l1 = "Peer CRC";   l2 = "mismatch"; }
    else                     { l1 = "Peer size";  l2 = "mismatch"; }
    m1_message_box(&m1_u8g2, "Peer Link", l1, l2, " OK ");
}

/* File-picker modal -> send the chosen file to `mac`. */
static void pl_send_file_flow(const uint8_t *mac)
{
    S_M1_Buttons_Status btn;
    S_M1_Main_Q_t       q;
    S_M1_file_info     *fi;
    char path[80] = "";

    m1_fb_init(&m1_u8g2);
    m1_fb_set_dir("0:");
    m1_fb_display(NULL);

    while (1) {
        if (xQueueReceive(main_q_hdl, &q, portMAX_DELAY) != pdTRUE) continue;
        if (q.q_evt_type != Q_EVENT_KEYPAD) continue;
        if (xQueueReceive(button_events_q_hdl, &btn, 0) != pdTRUE) continue;

        if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK) {
            m1_fb_deinit();
            return;              /* cancelled */
        }
        fi = m1_fb_display(&btn);
        if (fi && fi->status == FB_OK && fi->file_is_selected) {
            snprintf(path, sizeof(path), "%s/%s", fi->dir_name, fi->file_name);
            m1_fb_deinit();
            break;
        }
    }

    if (path[0]) peer_send_file(mac, path);
}

/* ---- main screen ---- */

void m1_peer_link_run(void)
{
    S_M1_Buttons_Status btn;
    S_M1_Main_Q_t       q_item;

    uint8_t       my_mac[6] = {0};
    char          myname[24];
    m1_now_peer_t peers[PL_MAX_PEERS];
    int           npeers = 0, sel = 0;
    uint32_t      last_poll = 0;

    memset(&s_rx, 0, sizeof(s_rx));
    s_rx_status[0] = '\0';

    bool started = m1_esp_client_now_start(PL_CHANNEL, "M1", my_mac);
    if (started) {
        snprintf(myname, sizeof(myname), "M1-%02X%02X", my_mac[4], my_mac[5]);
        m1_esp_client_now_start(PL_CHANNEL, myname, my_mac);
    } else {
        snprintf(myname, sizeof(myname), "M1");
    }

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (started) {
            pl_drain_rx();                             /* every loop — catch bursts */
            if ((now - last_poll) >= PL_POLL_MS) {     /* slower: presence + peers */
                last_poll = now;
                m1_esp_client_now_announce();
                int n = m1_esp_client_now_get_peers(peers, PL_MAX_PEERS);
                npeers = (n > 0) ? n : 0;
                if (sel >= npeers) sel = npeers ? npeers - 1 : 0;
            }
        }

        /* Draw */
        m1_u8g2_firstpage();
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            if (started) {
                u8g2_DrawStr(&m1_u8g2, 0, 8, myname);
            } else {
                char noesp[28];
                if (m1_esp_client_now_last_err)
                    snprintf(noesp, sizeof(noesp), "noESP e=0x%lX",
                             (unsigned long)m1_esp_client_now_last_err);
                else
                    snprintf(noesp, sizeof(noesp), "noESP n=%d b0=%d",
                             m1_esp_client_now_last_n, m1_esp_client_now_last_b0);
                u8g2_DrawStr(&m1_u8g2, 0, 8, noesp);
            }

            char hdr[24];
            snprintf(hdr, sizeof(hdr), "Peers: %d", npeers);
            u8g2_DrawStr(&m1_u8g2, 0, 18, hdr);

            int y = 28;
            for (int i = 0; i < npeers && i < 3; i++) {
                char line[28];
                snprintf(line, sizeof(line), "%c%s %ddB",
                         (i == sel) ? '>' : ' ', peers[i].name, peers[i].rssi);
                u8g2_DrawStr(&m1_u8g2, 0, y, line);
                y += 10;
            }
            if (s_rx_status[0])
                u8g2_DrawStr(&m1_u8g2, 0, 50, s_rx_status);

            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            u8g2_DrawStr(&m1_u8g2, 4, 61, "OK:Send File");
            u8g2_DrawStr(&m1_u8g2, 96, 61, "Back");
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        } while (m1_u8g2_nextpage());

        if (xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(120)) == pdTRUE
            && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &btn, 0);

            if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK) {
                if (started) m1_esp_client_now_stop();
                rx_abort();
                xQueueReset(main_q_hdl);
                break;
            } else if (btn.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK) {
                if (sel > 0) sel--;
            } else if (btn.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK) {
                if (sel < npeers - 1) sel++;
            } else if (btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK) {
                if (started && npeers > 0) {
                    pl_send_file_flow(peers[sel].mac);   /* modal: pick + beam */
                    last_poll = 0;                        /* refresh on return */
                }
            }
        }
    }
}
