/* See COPYING.txt for license details. */
/*
 * m1_esp_client.c — M1 -> ESP32-C6 operations over m1_link + m1_esp_rpc.
 */
#include "m1_esp_client.h"
#include "m1_link.h"
#include "m1_qmon_relay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

#define ESP_CALL_TIMEOUT_MS   500
/* Screen push uses the full default timeout so healthy pushes complete; control
 * starvation is prevented by pacing the WiFi frame interval (m1_rpc.c), not by a
 * short timeout (60/200ms were too short and dropped every frame). */
#define SCREEN_PUSH_TIMEOUT_MS 500

/* Serialized scratch buffers (ESP ops are one-at-a-time). Responses can be
 * reassembled from fragments, so s_resp holds more than one frame.
 * Must be >= the ESP's M1_RPC_MAX_PAYLOAD (4082): the ESP's NOW_RECV_GET drain
 * packs up to that many bytes of queued messages and DEQUEUES them, so if we
 * reassemble/read fewer, the tail messages are lost off the ESP ring (dropped
 * peer-link chunks -> intermittent "RX err"). Sized to hold a full ESP drain. */
#define M1_ESP_RESP_MAX   4096
static uint8_t s_resp[M1_ESP_RESP_MAX];

/* Tracks whether the ESP is currently associated to a WiFi AP, updated by the
 * connect/disconnect/status calls below. The qMonstatek-over-WiFi relay uses it
 * to skip polling entirely when there's no WiFi (no network -> no TCP client). */
static volatile bool s_wifi_connected = false;
/* SSID we believe we're associated to (updated on connect success / cleared on
 * disconnect). Used to make connect idempotent across every caller. */
static char s_wifi_ssid[33] = { 0 };
bool m1_esp_client_wifi_is_connected(void) { return s_wifi_connected; }

/* Serializes the WHOLE esp_call across all callers. The static s_resp buffer is
 * shared, and the parse/copy below happens AFTER m1_link releases its own mutex —
 * so without this, the higher-priority screen task can preempt the relay task
 * between m1_link's return and the parse, overwrite s_resp with its own response,
 * and the relay then reads a clobbered button/poll frame (buttons silently lost).
 * Must cover m1_link_request_msg AND the s_resp parse. */
static SemaphoreHandle_t s_esp_mtx = NULL;

/*
 * Do one request/response. Builds a REQ(msg_id, payload), sends via m1_link,
 * parses the reply. On success copies the response payload into resp_out and
 * returns its length (>=0). Returns 0 on timeout, negative on error/NAK.
 */
static int esp_call_to(uint16_t msg_id, const uint8_t *payload, uint16_t plen,
                       uint8_t *resp_out, uint16_t resp_cap, uint32_t timeout_ms)
{
    if (s_esp_mtx) xSemaphoreTake(s_esp_mtx, portMAX_DELAY);

    int ret;
    /* Fragments a large request and reassembles a large response internally. */
    int r = m1_link_request_msg(M1ESP_REQ, msg_id, payload, plen,
                                s_resp, sizeof(s_resp), timeout_ms);
    if (r <= 0) {
        ret = r;                             /* 0 = timeout, <0 = link error */
    } else {
        m1esp_header_t h;
        const uint8_t *p;
        uint16_t pl;
        if (m1esp_parse(s_resp, r, &h, &p, &pl) != 0)      ret = -2;
        else if (h.msg_type == M1ESP_NAK)                  ret = -3;
        else {
            uint16_t n = pl > resp_cap ? resp_cap : pl;
            if (resp_out && n) memcpy(resp_out, p, n);
            ret = n;
        }
    }

    if (s_esp_mtx) xSemaphoreGive(s_esp_mtx);
    return ret;
}

/* Default-timeout call. */
static int esp_call(uint16_t msg_id, const uint8_t *payload, uint16_t plen,
                    uint8_t *resp_out, uint16_t resp_cap)
{
    return esp_call_to(msg_id, payload, plen, resp_out, resp_cap, ESP_CALL_TIMEOUT_MS);
}

void m1_esp_client_init(void)
{
    if (!s_esp_mtx) s_esp_mtx = xSemaphoreCreateMutex();
    m1_link_init();
}

bool m1_esp_client_ping(void)
{
    const uint8_t cookie[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    uint8_t back[4] = { 0 };
    int n = esp_call(M1ESP_SYS_PING, cookie, sizeof(cookie), back, sizeof(back));
    return (n == 4) && (memcmp(back, cookie, 4) == 0);
}

bool m1_esp_client_get_status(m1esp_devstatus_t *out)
{
    if (!out) return false;
    int n = esp_call(M1ESP_SYS_GET_STATUS, NULL, 0, (uint8_t *)out, sizeof(*out));
    return n == (int)sizeof(*out);
}

/* Native ESP firmware version -> formatted "maj.min.patch <ver>" into out. */
bool m1_esp_client_fw_version(char *out, uint16_t cap)
{
    if (!out || cap == 0) return false;
    m1esp_fw_version_t v;
    int n = esp_call(M1ESP_SYS_GET_FW_VERSION, NULL, 0, (uint8_t *)&v, sizeof(v));
    if (n < (int)sizeof(v)) return false;
    v.git_hash[sizeof(v.git_hash) - 1] = '\0';
    snprintf(out, cap, "%u.%u.%u %s", v.major, v.minor, v.patch, v.git_hash);
    return true;
}

/* ---- ESP-NOW peer-to-peer M1<->M1 link ---- */

/* Diagnostic: raw esp_call result + first response byte from the last
 * now_start attempt. n>=0 is a payload length (0=timeout), n<0 is a link/NAK
 * error (-3 = ESP replied NAK, i.e. esp_now_link_start() failed on the ESP). */
volatile int     m1_esp_client_now_last_n   = 0x7FFF;
volatile int     m1_esp_client_now_last_b0  = -1;
volatile uint32_t m1_esp_client_now_last_err = 0;   /* esp_err_t reported by ESP */

bool m1_esp_client_now_start(uint8_t channel, const char *name, uint8_t mac_out[6])
{
    uint8_t req[1 + 23];
    int nlen = name ? (int)strlen(name) : 0;
    if (nlen > 23) nlen = 23;
    req[0] = channel;
    if (nlen) memcpy(&req[1], name, nlen);

    uint8_t resp[7];
    int n = esp_call(M1ESP_NOW_START, req, (uint16_t)(1 + nlen), resp, sizeof(resp));
    m1_esp_client_now_last_n  = n;
    m1_esp_client_now_last_b0 = (n >= 1) ? resp[0] : -1;
    /* On failure the ESP returns the raw esp_err_t (4 bytes LE) as the payload. */
    m1_esp_client_now_last_err = (n >= 4)
        ? ((uint32_t)resp[0] | ((uint32_t)resp[1] << 8) |
           ((uint32_t)resp[2] << 16) | ((uint32_t)resp[3] << 24))
        : 0;
    if (n < 7 || resp[0] != M1ESP_OK) return false;
    if (mac_out) memcpy(mac_out, &resp[1], 6);
    return true;
}

bool m1_esp_client_now_stop(void)
{
    uint8_t r[1];
    int n = esp_call(M1ESP_NOW_STOP, NULL, 0, r, sizeof(r));
    return n >= 1 && r[0] == M1ESP_OK;
}

bool m1_esp_client_now_announce(void)
{
    uint8_t r[1];
    int n = esp_call(M1ESP_NOW_ANNOUNCE, NULL, 0, r, sizeof(r));
    return n >= 1 && r[0] == M1ESP_OK;
}

int m1_esp_client_now_get_peers(m1_now_peer_t *out, int max)
{
    uint8_t buf[512];
    int n = esp_call(M1ESP_NOW_PEERS_GET, NULL, 0, buf, sizeof(buf));
    if (n < 1) return -1;

    int count = buf[0];
    int off = 1, got = 0;
    for (int i = 0; i < count && got < max; i++) {
        if (off + 8 > n) break;                 /* mac(6)+rssi(1)+namelen(1) */
        memcpy(out[got].mac, &buf[off], 6); off += 6;
        out[got].rssi = (int8_t)buf[off++];
        int nlen = buf[off++];
        if (off + nlen > n) break;
        int cn = nlen > 23 ? 23 : nlen;
        memcpy(out[got].name, &buf[off], cn);
        out[got].name[cn] = '\0';
        off += nlen;
        got++;
    }
    return got;
}

bool m1_esp_client_now_send(const uint8_t mac[6], const uint8_t *data, uint16_t len)
{
    uint8_t req[6 + 240];
    if (len > 240) len = 240;
    memcpy(req, mac, 6);
    if (len) memcpy(&req[6], data, len);

    uint8_t r[1];
    int n = esp_call(M1ESP_NOW_SEND, req, (uint16_t)(6 + len), r, sizeof(r));
    return n >= 1 && r[0] == M1ESP_OK;
}

int m1_esp_client_now_recv(uint8_t *buf, uint16_t cap)
{
    /* Returns the raw [count:1] then per-msg [mac:6][len:2 LE][data] blob;
     * caller parses. <0 on transport error. */
    return esp_call(M1ESP_NOW_RECV_GET, NULL, 0, buf, cap);
}

/* NTP time sync (WiFi must be connected). ESP blocks up to ~5s for the sync. */
bool m1_esp_client_sntp_sync(m1esp_time_t *out)
{
    if (!out) return false;
    int n = esp_call_to(M1ESP_SYS_SNTP_SYNC, NULL, 0, (uint8_t *)out, sizeof(*out), 8000u);
    return n == (int)sizeof(*out);
}

/* ---- 802.15.4 (Zigbee/Thread) device discovery ---- */
/* Enabling the 802.15.4 radio does a synchronous PHY calibration + coexistence
 * hand-off on the ESP; once WiFi already owns the radio this can take well over
 * the 500ms default, so give ZB_SNIFF_START a wider window (the ESP replies as
 * soon as the radio is armed — usually far sooner). */
#define ESP_ZB_START_TIMEOUT_MS   3000u

/* channel 0 = hop all 11-26, else dwell that channel. */
bool m1_esp_client_zb_sniff_start(uint8_t channel)
{
    uint8_t p[1] = { channel };
    uint8_t resp[1] = { 0 };
    int n = esp_call_to(M1ESP_ZB_SNIFF_START, p, 1, resp, sizeof(resp),
                        ESP_ZB_START_TIMEOUT_MS);
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_zb_sniff_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call_to(M1ESP_ZB_SNIFF_STOP, NULL, 0, resp, sizeof(resp), 2000u);
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* 802.15.4 beacon-request flood. channel 0 = sweep 11-26, else dwell it. */
bool m1_esp_client_zb_flood_start(uint8_t channel)
{
    uint8_t p[1] = { channel };
    uint8_t resp[1] = { 0 };
    int n = esp_call_to(M1ESP_ZB_FLOOD_START, p, 1, resp, sizeof(resp),
                        ESP_ZB_START_TIMEOUT_MS);
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_zb_flood_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call_to(M1ESP_ZB_FLOOD_STOP, NULL, 0, resp, sizeof(resp), 2000u);
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* Fills buf with [u8 count][m1esp_zb_device_t x count]. Returns bytes, <0 err. */
int m1_esp_client_zb_sniff_get(uint8_t *buf, uint16_t cap)
{
    return esp_call(M1ESP_ZB_SNIFF_GET, NULL, 0, buf, cap);
}

/* A blocking WiFi scan on the ESP (esp_wifi_scan_start(...,true)) sweeps every
 * channel and takes ~3-4s; the 500ms default would time out mid-scan (rc=0 with
 * hs=1/send_r=0). Give it an 8s window. */
#define ESP_SCAN_TIMEOUT_MS      8000u
/* Association + DHCP lease can also take several seconds. */
#define ESP_CONNECT_TIMEOUT_MS   12000u

int m1_esp_client_wifi_scan(uint8_t *buf, uint16_t buf_size)
{
    return esp_call_to(M1ESP_WIFI_SCAN, NULL, 0, buf, buf_size, ESP_SCAN_TIMEOUT_MS);
}

bool m1_esp_client_wifi_connect(const char *ssid, const char *pwd, uint8_t ip_out[4])
{
    /* Refuse an M1-initiated connect while a qMonstatek desktop session is live
     * over WiFi: re-associating tears down the very STA link the desktop is riding
     * on (instant "IP N/A"), and it can't be brought back without the desktop. */
    if (m1_qmon_relay_session_active())
        return false;

    /* Idempotency guard covering every connect path (saved menu + scan). Never
     * re-associate to the AP we're already on — that tears the live link down on
     * the ESP. If we think we're on this SSID, verify with a status poll and
     * no-op (returning the current IP) when it's still up. If we're associated to
     * a DIFFERENT SSID, cleanly disconnect and let the radio settle first. */
    if (s_wifi_connected && strncmp(s_wifi_ssid, ssid, 32) == 0) {
        uint8_t connected = 0;
        uint8_t ip[4] = { 0 };
        if (m1_esp_client_wifi_status(&connected, ip) && connected) {
            if (ip_out) memcpy(ip_out, ip, 4);
            return true;                 /* already on this SSID — no-op */
        }
        /* status says the link actually dropped — fall through and reconnect */
    } else if (s_wifi_connected) {
        m1_esp_client_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    uint8_t payload[2 + 32 + 64];
    uint16_t off = 0;
    uint8_t sl = (uint8_t)strnlen(ssid, 32);
    uint8_t pl = (uint8_t)strnlen(pwd, 64);
    payload[off++] = sl;
    memcpy(&payload[off], ssid, sl); off += sl;
    payload[off++] = pl;
    memcpy(&payload[off], pwd, pl); off += pl;

    /* Hand off, then poll. The ESP kicks the association asynchronously and
     * answers in milliseconds (resp[0]=OK means "started"), so this call releases
     * the shared link mutex almost immediately instead of parking it for the whole
     * ~12s association — which used to starve the qMonstatek relay and screen. We
     * then poll wifi_status; each poll is a short, mutex-releasing transaction, so
     * other link users keep running while the ESP associates + gets its lease. */
    uint8_t resp[5] = { 0 };
    int n = esp_call_to(M1ESP_WIFI_CONNECT, payload, off, resp, sizeof(resp), 1500u);
    if (n < 1 || resp[0] != M1ESP_OK) { s_wifi_connected = false; return false; }

    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(ESP_CONNECT_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(300));
        uint8_t connected = 0;
        uint8_t ip[4] = { 0 };
        if (m1_esp_client_wifi_status(&connected, ip) && connected) {
            s_wifi_connected = true;                 /* also refreshed inside status */
            strncpy(s_wifi_ssid, ssid, 32);
            s_wifi_ssid[32] = '\0';
            if (ip_out) memcpy(ip_out, ip, 4);
            return true;
        }
    }
    s_wifi_connected = false;
    return false;
}

bool m1_esp_client_wifi_disconnect(void)
{
    s_wifi_connected = false;
    s_wifi_ssid[0] = '\0';
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_WIFI_DISCONNECT, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_wifi_status(uint8_t *connected, uint8_t ip_out[4])
{
    uint8_t resp[5] = { 0 };
    int n = esp_call(M1ESP_WIFI_GET_STATUS, NULL, 0, resp, sizeof(resp));
    if (n < 1) return false;
    s_wifi_connected = (resp[0] != 0);   /* authoritative refresh */
    if (connected) *connected = resp[0];
    if (ip_out && n >= 5) memcpy(ip_out, &resp[1], 4);
    return true;
}

bool m1_esp_client_deauth_start(const uint8_t bssid[6], uint8_t channel,
                                const uint8_t station[6], uint16_t count,
                                uint16_t interval_ms)
{
    m1esp_deauth_req_t req;
    memcpy(req.bssid, bssid, 6);
    req.channel = channel;
    memcpy(req.station, station, 6);
    req.count = count;
    req.interval_ms = interval_ms;

    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_DEAUTH_START, (const uint8_t *)&req, sizeof(req),
                     resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_deauth_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_DEAUTH_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

int m1_esp_client_ble_scan(uint8_t *buf, uint16_t buf_size, uint8_t duration_sec)
{
    uint8_t dur[1] = { duration_sec };
    if (esp_call(M1ESP_BLE_SCAN_START, dur, 1, NULL, 0) < 0) return -1;
    vTaskDelay(pdMS_TO_TICKS((uint32_t)duration_sec * 1000u + 250u));  /* let it scan */
    return esp_call(M1ESP_BLE_SCAN_RESULTS, NULL, 0, buf, buf_size);
}

bool m1_esp_client_beacon_start(const char ssids[][33], uint8_t count)
{
    uint8_t payload[M1L_PAYLOAD_MAX];
    uint16_t off = 0;
    payload[off++] = count;
    for (int i = 0; i < count; i++) {
        uint8_t l = (uint8_t)strnlen(ssids[i], 32);
        if ((uint16_t)(off + 1 + l) > sizeof(payload)) { count = i; payload[0] = count; break; }
        payload[off++] = l;
        memcpy(&payload[off], ssids[i], l);
        off += l;
    }
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_BEACON_START, payload, off, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_beacon_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_BEACON_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* Probe-request flood. Payload: [channel:1][count:1] then [len:1][ssid] x count.
 * count 0 = wildcard broadcast probes. */
bool m1_esp_client_probe_start(uint8_t channel, const char ssids[][33], uint8_t count)
{
    uint8_t payload[M1L_PAYLOAD_MAX];
    uint16_t off = 0;
    payload[off++] = channel;
    uint8_t n = count > 16 ? 16 : count;
    payload[off++] = n;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t l = (uint8_t)strnlen(ssids[i], 32);
        if ((uint16_t)(off + 1 + l) > sizeof(payload)) { n = i; payload[1] = n; break; }
        payload[off++] = l;
        memcpy(&payload[off], ssids[i], l);
        off += l;
    }
    uint8_t resp[1] = { 0 };
    int r = esp_call(M1ESP_OFF_PROBE_START, payload, off, resp, sizeof(resp));
    return (r >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_probe_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_PROBE_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* Karma probe-response auto-responder on `channel`. */
bool m1_esp_client_karma_start(uint8_t channel)
{
    uint8_t p[1] = { channel };
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_KARMA_START, p, 1, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_karma_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_KARMA_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* ---- Packet Monitor (promiscuous sniffer, runs on the ESP) ---- */

/* Returns the ESP's esp_err (0 = OK); negative = SPI/link failure (e.g. timeout
 * = -1000), so a crash/no-response is distinguishable from an app-level error. */
int m1_esp_client_monitor_start(uint8_t channel)   /* channel 0 = hop 1-13 */
{
    uint8_t req[2] = { channel, 0 };   /* band 0 = 2.4GHz */
    uint8_t resp[4] = { 0 };
    int n = esp_call_to(M1ESP_OFF_MONITOR_START, req, sizeof(req),
                        resp, sizeof(resp), 2000u);
    if (n < 4) return -1000;           /* no/short response = link timeout */
    int32_t err;
    memcpy(&err, resp, 4);
    return (int)err;
}

bool m1_esp_client_monitor_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_MONITOR_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_monitor_stats(uint32_t *total, uint32_t *dropped, uint8_t *buffered)
{
    uint8_t resp[9] = { 0 };
    int n = esp_call(M1ESP_OFF_MONITOR_STATS, NULL, 0, resp, sizeof(resp));
    if (n < 9) return false;
    if (total)    memcpy(total, &resp[0], 4);
    if (dropped)  memcpy(dropped, &resp[4], 4);
    if (buffered) *buffered = resp[8];
    return true;
}

/* Pull one captured frame. Returns frame length (0 = none buffered). */
uint16_t m1_esp_client_monitor_read(uint8_t *out, uint16_t out_cap,
                                    int8_t *rssi, uint8_t *channel)
{
    uint8_t resp[4 + 256];
    int n = esp_call(M1ESP_OFF_MONITOR_READ, NULL, 0, resp, sizeof(resp));
    if (n < 4) return 0;   /* empty response = nothing buffered */
    uint16_t flen = (uint16_t)(resp[2] | (resp[3] << 8));
    if (flen > (uint16_t)(n - 4)) flen = (uint16_t)(n - 4);
    uint16_t cp = flen < out_cap ? flen : out_cap;
    memcpy(out, &resp[4], cp);
    if (channel) *channel = resp[0];
    if (rssi)    *rssi = (int8_t)resp[1];
    return cp;
}

/* ---- Captive Portal (rogue AP credential capture, runs on the ESP) ---- */

/* Returns the ESP's esp_err (0 = OK); negative = SPI/link failure. */
int m1_esp_client_captive_start(const char *ssid, const char *title, uint8_t channel)
{
    if (!ssid || !ssid[0]) return -1001;
    uint8_t payload[100];
    uint16_t off = 0;
    uint8_t sl = (uint8_t)strnlen(ssid, 32);
    payload[off++] = channel;
    payload[off++] = sl;
    memcpy(&payload[off], ssid, sl); off += sl;
    if (title && title[0]) {
        uint8_t tl = (uint8_t)strnlen(title, 48);
        memcpy(&payload[off], title, tl); off += tl;
    }
    uint8_t resp[4] = { 0 };
    /* AP-mode switch + DNS/HTTP bring-up is slow — allow 3s. */
    int n = esp_call_to(M1ESP_OFF_CAPTIVE_START, payload, off, resp, sizeof(resp), 3000u);
    if (n < 4) return -1000;
    int32_t err;
    memcpy(&err, resp, 4);
    return (int)err;
}

bool m1_esp_client_captive_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_CAPTIVE_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* Fetch captured credentials as raw bytes:
 * [count:1] then per cred [ts:4][ulen:1][user][plen:1][pass]. Returns bytes. */
int m1_esp_client_captive_creds(uint8_t *buf, uint16_t buf_size)
{
    return esp_call(M1ESP_OFF_CAPTIVE_CREDS, NULL, 0, buf, buf_size);
}

/* Portal chain diagnostics: DNS queries answered, HTTP page hits, associated
 * clients, and the raw last-POST snapshot (so we can see why parsing fails). */
bool m1_esp_client_captive_diag(uint32_t *dns_q, uint32_t *http_hits, uint8_t *clients,
                                char *last_post, uint16_t last_post_cap)
{
    uint8_t resp[9 + 96] = { 0 };
    int n = esp_call(M1ESP_OFF_CAPTIVE_DIAG, NULL, 0, resp, sizeof(resp));
    if (n < 9) return false;
    if (dns_q)     memcpy(dns_q, &resp[0], 4);
    if (http_hits) memcpy(http_hits, &resp[4], 4);
    if (clients)   *clients = resp[8];
    if (last_post && last_post_cap > 0) {
        int pl = n - 9;
        if (pl < 0) pl = 0;
        if (pl > last_post_cap - 1) pl = last_post_cap - 1;
        memcpy(last_post, &resp[9], pl);
        last_post[pl] = 0;
    }
    return true;
}

bool m1_esp_client_hs_start(const uint8_t bssid[6], uint8_t channel, uint16_t deauth_count)
{
    uint8_t req[9];
    memcpy(req, bssid, 6);
    req[6] = channel;
    req[7] = (uint8_t)(deauth_count & 0xFF);
    req[8] = (uint8_t)(deauth_count >> 8);
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_HS_START, req, sizeof(req), resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_hs_status(uint8_t *state, uint32_t *total_len)
{
    uint8_t resp[5] = { 0 };
    int n = esp_call(M1ESP_OFF_HS_STATUS, NULL, 0, resp, sizeof(resp));
    if (n < 5) return false;
    if (state) *state = resp[0];
    if (total_len) memcpy(total_len, &resp[1], 4);
    return true;
}

int m1_esp_client_hs_read(uint32_t offset, uint8_t *buf, uint16_t want)
{
    uint8_t req[6];
    req[0] = (uint8_t)offset; req[1] = (uint8_t)(offset >> 8);
    req[2] = (uint8_t)(offset >> 16); req[3] = (uint8_t)(offset >> 24);
    req[4] = (uint8_t)(want & 0xFF); req[5] = (uint8_t)(want >> 8);
    return esp_call(M1ESP_OFF_HS_GET, req, sizeof(req), buf, want);
}

bool m1_esp_client_hs_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_OFF_HS_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

int m1_esp_client_sta_scan(const uint8_t bssid[6], uint8_t channel, uint8_t duration_sec,
                           uint8_t *buf, uint16_t buf_size)
{
    uint8_t req[8];
    memcpy(req, bssid, 6);
    req[6] = channel;
    req[7] = duration_sec;
    if (esp_call(M1ESP_OFF_STA_SCAN_START, req, sizeof(req), NULL, 0) < 0) return -1;
    vTaskDelay(pdMS_TO_TICKS((uint32_t)duration_sec * 1000u + 250u));
    return esp_call(M1ESP_OFF_STA_SCAN_RESULTS, NULL, 0, buf, buf_size);
}

bool m1_esp_client_ble_connect(const char *addr_str, uint8_t addr_type)
{
    unsigned a[6];
    if (!addr_str ||
        sscanf(addr_str, "%x:%x:%x:%x:%x:%x", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) != 6)
        return false;
    uint8_t payload[7];
    for (int i = 0; i < 6; i++) payload[i] = (uint8_t)a[i];
    payload[6] = addr_type;
    uint8_t resp[1] = { 0 };
    /* ESP blocks up to ~8 s establishing the link — allow more than that. */
    int n = esp_call_to(M1ESP_BLE_CONNECT, payload, sizeof(payload), resp, sizeof(resp), 10000);
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_ble_advertise(const char *name)
{
    uint8_t nl = (uint8_t)strnlen(name ? name : "", 31);
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_BLE_ADV_START, (const uint8_t *)name, nl, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* BLE spam: start the proximity-pair popup flooder in `mode`
 * (0=Apple 1=FastPair 2=Samsung 3=SwiftPair 4=All), or stop it. */
bool m1_esp_client_ble_spam_start(uint8_t mode)
{
    uint8_t p[1] = { mode };
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_BLE_SPAM_START, p, 1, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_ble_spam_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_BLE_SPAM_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* WiFi Hotspot (SoftAP): bring up a real AP the phone can join (no router).
 * Payload: [channel:1][ssid\0][pass\0]. */
bool m1_esp_client_softap_start(const char *ssid, const char *pass, uint8_t channel)
{
    uint8_t p[1 + 33 + 65];
    uint16_t o = 0;
    size_t sl = strnlen(ssid ? ssid : "", 32);
    size_t pl = strnlen(pass ? pass : "", 64);

    p[o++] = channel;
    memcpy(&p[o], ssid, sl); o += (uint16_t)sl; p[o++] = '\0';
    memcpy(&p[o], pass, pl); o += (uint16_t)pl; p[o++] = '\0';

    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_SOFTAP_START, p, o, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_softap_stop(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_SOFTAP_STOP, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

/* Number of stations currently joined to the hotspot. */
uint8_t m1_esp_client_softap_sta_count(void)
{
    uint8_t resp[2] = { 0, 0 };
    int n = esp_call(M1ESP_SOFTAP_STA_LIST, NULL, 0, resp, sizeof(resp));
    return (n >= 1) ? resp[0] : 0;
}

/* Hotspot status in one call: connected client count + whether internet is being
 * shared to clients (NAT via an upstream STA link). Returns false on RPC error. */
bool m1_esp_client_softap_status(uint8_t *count, uint8_t *internet_shared)
{
    uint8_t resp[2] = { 0, 0 };
    int n = esp_call(M1ESP_SOFTAP_STA_LIST, NULL, 0, resp, sizeof(resp));
    if (n < 1) return false;
    if (count)           *count           = resp[0];
    if (internet_shared) *internet_shared = (n >= 2) ? resp[1] : 0;
    return true;
}

/* Bluetooth Direct: start/stop the ESP advertising its NUS RPC service so a
 * phone can connect over BLE and speak the same RPC as WiFi/USB. */
bool m1_esp_client_ble_direct(bool enable)
{
    uint8_t p[1] = { enable ? 1u : 0u };
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_BLE_RPC_ADV, p, 1, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_ble_hid_init(const char *name)
{
    uint8_t nl = (uint8_t)strnlen(name ? name : "", 31);
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_BLE_HID_INIT, (const uint8_t *)name, nl, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_ble_hid_deinit(void)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_BLE_HID_DEINIT, NULL, 0, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_ble_hid_status(uint8_t *connected)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_BLE_HID_STATUS, NULL, 0, resp, sizeof(resp));
    if (n < 1) return false;
    if (connected) *connected = resp[0];
    return true;
}

bool m1_esp_client_ble_hid_key(uint8_t modifier, uint8_t keycode)
{
    uint8_t payload[3];
    payload[0] = modifier;
    payload[1] = 1;            /* key_count */
    payload[2] = keycode;
    int n = esp_call(M1ESP_BLE_HID_KEY, payload, sizeof(payload), NULL, 0);
    return n >= 0;
}

bool m1_esp_client_rpc_connected(uint8_t *connected)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_RPC_STATUS, NULL, 0, resp, sizeof(resp));
    if (n < 1) return false;
    if (connected) *connected = resp[0];
    return true;
}

int m1_esp_client_qmon_poll(uint8_t *buf, uint16_t buf_size)
{
    /* Response payload is the queued qMonstatek frame (empty => nothing). */
    return esp_call(M1ESP_QMON_POLL, NULL, 0, buf, buf_size);
}

bool m1_esp_client_qmon_resp(const uint8_t *frame, uint16_t len)
{
    uint8_t resp[1] = { 0 };
    int n = esp_call(M1ESP_QMON_RESP, frame, len, resp, sizeof(resp));
    return (n >= 1) && (resp[0] == M1ESP_OK);
}

bool m1_esp_client_screen_push(const uint8_t *fb, uint16_t len)
{
    /* Send the WHOLE framebuffer — esp_call fragments payloads larger than one
     * SPI MTU (the ESP reassembles). Do NOT cap to M1L_PAYLOAD_MAX or the ESP
     * only ever gets the top half of the screen. */
    uint8_t resp[1] = { 0 };
    int n = esp_call_to(M1ESP_SCREEN_PUSH, fb, len, resp, sizeof(resp),
                        SCREEN_PUSH_TIMEOUT_MS);
    return (n >= 1) && (resp[0] == M1ESP_OK);
}
