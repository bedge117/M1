/*
 * m1_802154.c
 *
 * IEEE 802.15.4 (Zigbee/Thread) scanning for M1
 *
 * Sends AT+ZIGSNIFF commands to ESP32-C6 and parses +ZIGFRAME responses.
 * Builds a deduplicated device list and displays it on the LCD.
 * Two entry points: zigbee_scan() and thread_scan() — same logic,
 * different protocol filter ('Z' vs 'T').
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_802154.h"
#include "m1_esp32_hal.h"
#include "esp_app_main.h"
#include "m1_esp_rpc_cfg.h"   /* M1_USE_ESP_RPC */
#include "m1_esp_client.h"    /* native 802.15.4 device discovery over m1_link */
#include "m1_compile_cfg.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_system.h"

/*************************** D E F I N E S ************************************/

#define SCAN_DWELL_TIME_MS      2000   /* Time per channel in ms (legacy AT path) */
#define SCAN_POLL_INTERVAL_MS   50     /* How often to poll SPI for frames */
#define AT_RESP_BUF_SIZE        512    /* Buffer for AT responses */

/* Option C live-scan window. The ESP dwells ~700ms/channel, so 16 channels is
 * ~11s per full sweep; 22s gives ~2 sweeps so sporadic devices are caught and the
 * count converges run-to-run. */
#define ESP_SCAN_TOTAL_MS       22000

#define LIST_ITEM_HEIGHT        9
#define LIST_START_Y            13
#define LIST_VISIBLE            4


/************************** S T A T I C S ************************************/

static ieee802154_device_t s_devices[IEEE802154_MAX_DEVICES];
static int s_device_count = 0;

/* Filtered view into s_devices for the drill-down list (indices, not copies). */
static int s_view[IEEE802154_MAX_DEVICES];
static int s_view_count = 0;

/********************* H E L P E R S ****************************************/

static void draw_title_bar(const char *title)
{
    u8g2_DrawXBMP(&m1_u8g2, 0, 0, 128, 14, m1_frame_128_14);
    u8g2_DrawStr(&m1_u8g2, 2, 1 + 10, title);
}

static void draw_list_item(uint8_t vis_idx, const char *text, bool selected)
{
    uint8_t y = LIST_START_Y + vis_idx * LIST_ITEM_HEIGHT;

    if (selected)
    {
        u8g2_SetDrawColor(&m1_u8g2, 1);
        u8g2_DrawBox(&m1_u8g2, 0, y, 128, LIST_ITEM_HEIGHT);
        u8g2_SetDrawColor(&m1_u8g2, 0);
    }

    char buf[22];
    strncpy(buf, text, 21);
    buf[21] = '\0';
    u8g2_DrawStr(&m1_u8g2, 2, y + 8, buf);

    if (selected)
        u8g2_SetDrawColor(&m1_u8g2, 1);
}

static void show_message(const char *title, const char *line1, const char *line2, uint16_t delay_ms)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
    draw_title_bar(title);
    if (line1)
        u8g2_DrawStr(&m1_u8g2, 2, 28, line1);
    if (line2)
        u8g2_DrawStr(&m1_u8g2, 2, 40, line2);
    m1_u8g2_nextpage();
    if (delay_ms)
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
}


/********************* D E T A I L   S C R E E N *****************************/

static void device_detail_screen(const char *title, ieee802154_device_t *dev)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    char prn_msg[25];
    bool redraw = true;

    while (1)
    {
        if (redraw)
        {
            redraw = false;
            m1_u8g2_firstpage();
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
            draw_title_bar(title);

            uint8_t y = 22;

            /* Source address */
            u8g2_DrawStr(&m1_u8g2, 2, y, dev->src_addr);
            y += 9;

            /* PAN ID + Channel */
            snprintf(prn_msg, sizeof(prn_msg), "PAN:%s Ch:%u", dev->src_pan, dev->channel);
            u8g2_DrawStr(&m1_u8g2, 2, y, prn_msg);
            y += 9;

            /* RSSI + LQI */
            snprintf(prn_msg, sizeof(prn_msg), "RSSI:%ddBm LQI:%u", dev->rssi, dev->lqi);
            u8g2_DrawStr(&m1_u8g2, 2, y, prn_msg);
            y += 9;

            /* Frame types + count */
            snprintf(prn_msg, sizeof(prn_msg), "%s (%u)", dev->frame_types, dev->frame_count);
            u8g2_DrawStr(&m1_u8g2, 2, y, prn_msg);

            m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "OK", arrowright_8x8);
            m1_u8g2_nextpage();
        }

        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                break;
            }
        }
    }
}

/********************* G R O U P E D   R E S U L T S *************************/

/* Normalize a device's stored proto to the display class: Z, T, or U. */
static char dev_class(const ieee802154_device_t *d)
{
    return (d->proto == 'Z' || d->proto == 'T') ? d->proto : 'U';
}

/* Build s_view: indices of s_devices matching a category.
 *   kind 'P' -> proto == key_proto ('Z'/'T'/'U')
 *   kind 'C' -> channel == key_channel */
static void build_view(char kind, char key_proto, uint8_t key_channel)
{
    s_view_count = 0;
    for (int i = 0; i < s_device_count; i++)
    {
        bool match = (kind == 'P') ? (dev_class(&s_devices[i]) == key_proto)
                                   : (s_devices[i].channel == key_channel);
        if (match && s_view_count < IEEE802154_MAX_DEVICES)
            s_view[s_view_count++] = i;
    }
}

/* Scrollable device list over the current s_view filter. OK/RIGHT opens detail,
 * BACK/LEFT returns to the category screen. */
static void zb_device_list_screen(const char *title)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    int16_t selection = 0, scroll_offset = 0;
    bool redraw = true;
    char page_info[20];

    if (s_view_count == 0) return;

    while (1)
    {
        if (redraw)
        {
            redraw = false;
            m1_u8g2_firstpage();
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

            snprintf(page_info, sizeof(page_info), "%s (%d)", title, s_view_count);
            draw_title_bar(page_info);

            if (selection < scroll_offset) scroll_offset = selection;
            if (selection >= scroll_offset + LIST_VISIBLE)
                scroll_offset = selection - LIST_VISIBLE + 1;

            for (int i = 0; i < LIST_VISIBLE && scroll_offset + i < s_view_count; i++)
            {
                int idx = s_view[scroll_offset + i];
                char item_text[22];
                char tag = (s_devices[idx].proto == 'Z' || s_devices[idx].proto == 'T')
                           ? s_devices[idx].proto : '?';
                snprintf(item_text, sizeof(item_text), "%c %.10s %ddB",
                    tag, s_devices[idx].src_addr, s_devices[idx].rssi);
                draw_list_item(i, item_text, (scroll_offset + i == selection));
            }

            snprintf(page_info, sizeof(page_info), "%d/%d", selection + 1, s_view_count);
            m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, page_info, "Info", arrowright_8x8);
            m1_u8g2_nextpage();
        }

        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                break;
            }
            else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
            {
                selection = (selection > 0) ? selection - 1 : s_view_count - 1;
                redraw = true;
            }
            else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
            {
                selection = (selection < s_view_count - 1) ? selection + 1 : 0;
                redraw = true;
            }
            else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK
                  || this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                device_detail_screen(title, &s_devices[s_view[selection]]);
                redraw = true;
            }
        }
    }
}

/* Category entry: a protocol folder, a channel folder, or a non-selectable
 * header separating the two groups. */
typedef struct {
    char    label[20];
    char    kind;       /* 'P' proto, 'C' channel, 'H' header (skip) */
    char    proto;      /* kind 'P' */
    uint8_t channel;    /* kind 'C' */
} zb_cat_t;

/* Step selection by dir (+1/-1) with wraparound, skipping header rows. */
static int cat_step(const zb_cat_t *cats, int ncat, int sel, int dir)
{
    for (int i = 0; i < ncat; i++)
    {
        sel += dir;
        if (sel < 0)      sel = ncat - 1;
        if (sel >= ncat)  sel = 0;
        if (cats[sel].kind != 'H') break;
    }
    return sel;
}

/* Post-scan results grouped as Thread/Zigbee/Unknown folders (with counts) plus
 * one folder per active channel. Selecting a folder opens the filtered list. */
static void zb_category_screen(const char *base_title)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    zb_cat_t cats[3 + 1 + 16];   /* up to 3 proto + 1 header + 16 channels */
    int ncat = 0;
    int16_t selection = 0, scroll_offset = 0;
    bool redraw = true;
    char page_info[20];

    /* Protocol folders (only those with members) — Thread first, matching UI order. */
    int cz = 0, ct = 0, cu = 0;
    for (int i = 0; i < s_device_count; i++)
    {
        char c = dev_class(&s_devices[i]);
        if (c == 'T') ct++; else if (c == 'Z') cz++; else cu++;
    }
    if (ct) { snprintf(cats[ncat].label, sizeof(cats[0].label), "Thread (%d)",  ct); cats[ncat].kind = 'P'; cats[ncat].proto = 'T'; ncat++; }
    if (cz) { snprintf(cats[ncat].label, sizeof(cats[0].label), "Zigbee (%d)",  cz); cats[ncat].kind = 'P'; cats[ncat].proto = 'Z'; ncat++; }
    if (cu) { snprintf(cats[ncat].label, sizeof(cats[0].label), "Unknown (%d)", cu); cats[ncat].kind = 'P'; cats[ncat].proto = 'U'; ncat++; }

    /* Divider, then one folder per channel that saw traffic. */
    if (ncat > 0) { snprintf(cats[ncat].label, sizeof(cats[0].label), "-- channels --"); cats[ncat].kind = 'H'; ncat++; }
    for (uint8_t ch = 11; ch <= 26; ch++)
    {
        int c = 0;
        for (int i = 0; i < s_device_count; i++) if (s_devices[i].channel == ch) c++;
        if (c) { snprintf(cats[ncat].label, sizeof(cats[0].label), "Ch %u (%d)", ch, c); cats[ncat].kind = 'C'; cats[ncat].channel = ch; ncat++; }
    }

    if (ncat == 0) return;

    while (1)
    {
        if (redraw)
        {
            redraw = false;
            m1_u8g2_firstpage();
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

            snprintf(page_info, sizeof(page_info), "%s (%d)", base_title, s_device_count);
            draw_title_bar(page_info);

            if (selection < scroll_offset) scroll_offset = selection;
            if (selection >= scroll_offset + LIST_VISIBLE)
                scroll_offset = selection - LIST_VISIBLE + 1;

            for (int i = 0; i < LIST_VISIBLE && scroll_offset + i < ncat; i++)
            {
                int idx = scroll_offset + i;
                /* header rows render centered-ish and never highlighted */
                draw_list_item(i, cats[idx].label,
                               (cats[idx].kind != 'H' && idx == selection));
            }

            snprintf(page_info, sizeof(page_info), "%d/%d", selection + 1, ncat);
            m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, page_info, "Open", arrowright_8x8);
            m1_u8g2_nextpage();
        }

        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                break;
            }
            else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
            {
                selection = cat_step(cats, ncat, selection, -1);
                redraw = true;
            }
            else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
            {
                selection = cat_step(cats, ncat, selection, +1);
                redraw = true;
            }
            else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK
                  || this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                zb_cat_t *c = &cats[selection];
                if (c->kind == 'H') continue;

                char sub_title[16];
                if (c->kind == 'P')
                    snprintf(sub_title, sizeof(sub_title), "%s",
                        c->proto == 'T' ? "Thread" : c->proto == 'Z' ? "Zigbee" : "Unknown");
                else
                    snprintf(sub_title, sizeof(sub_title), "Ch %u", c->channel);

                build_view(c->kind, c->proto, c->channel);
                zb_device_list_screen(sub_title);
                redraw = true;
            }
        }
    }
}

/********************* M A I N   S C A N   L O G I C *************************/

/* Rebuild s_devices from an ESP ZB_SNIFF_GET response:
 * [u8 count][m1esp_zb_device_t x count]. The ESP already dedupes by source
 * address and tracks peak RSSI / PAN / channel / frame types, so the M1 just
 * formats it for display. */
static void zb_populate_from_esp(const uint8_t *buf, int nbytes, char filter_proto)
{
    if (nbytes < 1) return;
    uint8_t count = buf[0];
    int off = 1;
    s_device_count = 0;

    for (uint8_t i = 0; i < count && s_device_count < IEEE802154_MAX_DEVICES; i++)
    {
        if (off + (int)sizeof(m1esp_zb_device_t) > nbytes) break;
        m1esp_zb_device_t rec;
        memcpy(&rec, &buf[off], sizeof(rec));
        off += sizeof(rec);

        /* ESP heuristic tag: 'Z', 'T', else Unknown. Each menu shows its own
         * protocol PLUS unclassified (encrypted) devices, so nothing is hidden. */
        char p = (rec.proto == 'Z' || rec.proto == 'T') ? (char)rec.proto : 'U';
        if (filter_proto == 'Z' && !(p == 'Z' || p == 'U')) continue;
        if (filter_proto == 'T' && !(p == 'T' || p == 'U')) continue;

        ieee802154_device_t *d = &s_devices[s_device_count++];
        memset(d, 0, sizeof(*d));
        d->proto       = p;
        d->rssi        = rec.rssi;
        d->lqi         = rec.lqi;
        d->channel     = rec.channel;
        d->frame_count = rec.frames;
        snprintf(d->src_pan, sizeof(d->src_pan), "%04X", rec.panid);

        if (rec.addr_mode == 2)             /* short 16-bit (little-endian) */
            snprintf(d->src_addr, sizeof(d->src_addr), "%02X%02X",
                     rec.addr[1], rec.addr[0]);
        else                                /* extended 64-bit */
            snprintf(d->src_addr, sizeof(d->src_addr),
                     "%02X%02X%02X%02X%02X%02X%02X%02X",
                     rec.addr[7], rec.addr[6], rec.addr[5], rec.addr[4],
                     rec.addr[3], rec.addr[2], rec.addr[1], rec.addr[0]);

        d->frame_types[0] = '\0';
        if (rec.flags & 0x01) strncat(d->frame_types, "BCN",
                                      sizeof(d->frame_types) - 1);
        if (rec.flags & 0x02) strncat(d->frame_types, d->frame_types[0] ? ",DAT" : "DAT",
                                      sizeof(d->frame_types) - strlen(d->frame_types) - 1);
        if (rec.flags & 0x04) strncat(d->frame_types, d->frame_types[0] ? ",CMD" : "CMD",
                                      sizeof(d->frame_types) - strlen(d->frame_types) - 1);
    }
}

/*
 * Core scan function. filter_proto = 'Z' for Zigbee, 'T' for Thread.
 */
static void ieee802154_scan(char filter_proto)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    bool scan_done = false;
    char page_info[20];
    const char *title = (filter_proto == 'Z') ? "Zigbee Scan" :
                        (filter_proto == 'T') ? "Thread Scan" : "802.15.4 Scan";

    s_device_count = 0;
    memset(s_devices, 0, sizeof(s_devices));

    /* Init ESP32 if needed */
    u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
    if (!m1_esp32_get_init_status())
    {
        m1_esp32_init();
    }
    if (!get_esp32_main_init_status())
    {
        show_message(title, "Initializing...", NULL, 0);
        esp32_main_init();
    }

    if (!get_esp32_main_init_status())
    {
        show_message(title, "ESP32 not ready!", "Press Back", 0);
        goto wait_exit;
    }


    /* Option C: native 802.15.4 device discovery over m1_link. The ESP hops
     * channels 11-26 in promiscuous mode, parses MAC headers, and aggregates a
     * deduped device table (addr/PAN/channel/peak-RSSI). We poll it live. */
    {
        static uint8_t zbuf[1 + IEEE802154_MAX_DEVICES * sizeof(m1esp_zb_device_t)];

        if ( !m1_esp_client_zb_sniff_start(0) )   /* 0 = hop all channels */
        {
            show_message(title, "802.15.4 radio", "start failed", 0);
            goto wait_exit;
        }

        /* Sweep for ~2 full channel passes, refreshing the list live. BACK aborts. */
        uint32_t scan_start = HAL_GetTick();
        while ( (HAL_GetTick() - scan_start) < ESP_SCAN_TOTAL_MS )
        {
            vTaskDelay(pdMS_TO_TICKS(700));
            int gn = m1_esp_client_zb_sniff_get(zbuf, sizeof(zbuf));
            if ( gn >= 1 ) zb_populate_from_esp(zbuf, gn, filter_proto);

            m1_u8g2_firstpage();
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
            draw_title_bar(title);
            u8g2_DrawStr(&m1_u8g2, 2, 26, "Scanning 11-26...");
            snprintf(page_info, sizeof(page_info), "Found: %d", s_device_count);
            u8g2_DrawStr(&m1_u8g2, 2, 38, page_info);
            {
                uint8_t bar_x = 2, bar_y = 44, bar_w = 124, bar_h = 6;
                uint32_t el = HAL_GetTick() - scan_start;
                if (el > ESP_SCAN_TOTAL_MS) el = ESP_SCAN_TOTAL_MS;
                uint8_t fill = (uint8_t)(el * (bar_w - 2) / ESP_SCAN_TOTAL_MS);
                u8g2_DrawFrame(&m1_u8g2, bar_x, bar_y, bar_w, bar_h);
                u8g2_DrawBox(&m1_u8g2, bar_x + 1, bar_y + 1, fill, bar_h - 2);
            }
            m1_u8g2_nextpage();

            if ( xQueueReceive(main_q_hdl, &q_item, 0) == pdTRUE )
            {
                if ( q_item.q_evt_type == Q_EVENT_KEYPAD )
                {
                    xQueueReceive(button_events_q_hdl, &this_button_status, 0);
                    if ( this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
                      || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK )
                    {
                        m1_esp_client_zb_sniff_stop();
                        xQueueReset(main_q_hdl);
                        return;
                    }
                }
            }
        }
        m1_esp_client_zb_sniff_stop();
        scan_done = true;
        goto have_results;
    }

have_results:
    if (!scan_done || s_device_count == 0)
    {
        show_message(title, "No devices found", "Press Back", 0);
        goto wait_exit;
    }

    /* Grouped results: Thread/Zigbee/Unknown + per-channel folders, each with a
     * count. Drilling in opens the filtered device list. */
    zb_category_screen(title);

    xQueueReset(main_q_hdl);
    return;

wait_exit:
    while (1)
    {
        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
                break;
        }
    }
    /* Stop sniffer just in case. Under Option C the ESP speaks m1_link, NOT AT —
     * calling the legacy spi_AT_send_recv() here blocks for its full timeout
     * waiting for an AT reply that never comes and desyncs the SPI link, which
     * tripped the IWDG and rebooted the M1 when backing out of the error screen.
     * Use the native RPC stop instead. */
    m1_esp_client_zb_sniff_stop();
    xQueueReset(main_q_hdl);
}

/********************* P U B L I C   E N T R Y   P O I N T S *****************/

void zigbee_scan(void)
{
    ieee802154_scan(IEEE802154_PROTO_ZIGBEE);
}

void thread_scan(void)
{
    ieee802154_scan(IEEE802154_PROTO_THREAD);
}

/* Single unified 802.15.4 scan — shows every device the ESP sees, each tagged
 * with its classification (Z Zigbee / T Thread / ? unidentified). Classification
 * is informational; nothing is hidden. Replaces the separate Zigbee/Thread menus. */
void ieee802154_scan_all(void)
{
    ieee802154_scan(0);
}

/********************* B E A C O N   F L O O D (offensive) *******************/

/* Hammer broadcast 802.15.4 Beacon Requests, which every Zigbee coordinator/
 * router must answer — floods the channel and their request handlers. The flood
 * runs autonomously on the ESP; the M1 just picks a channel and starts/stops it. */
void zigbee_beacon_flood(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    static const struct { const char *label; uint8_t ch; } chans[] = {
        { "All (11-26)", 0 }, { "Ch 11", 11 }, { "Ch 15", 15 },
        { "Ch 20", 20 },      { "Ch 25", 25 }, { "Ch 26", 26 },
    };
    const int nchan = (int)(sizeof(chans) / sizeof(chans[0]));
    int sel = 0;
    bool running = false;
    bool redraw = true;

    u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
    if (!m1_esp32_get_init_status())
        m1_esp32_init();
    if (!get_esp32_main_init_status())
    {
        show_message("Beacon Flood", "Initializing...", NULL, 0);
        esp32_main_init();
    }
    if (!get_esp32_main_init_status())
    {
        show_message("Beacon Flood", "ESP32 not ready!", "Press Back", 0);
        goto wait_back;
    }

    for (;;)
    {
        if (redraw && !running)
        {
            redraw = false;
            m1_u8g2_firstpage();
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
            draw_title_bar("Beacon Flood");
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);   /* smaller: 6 rows fit without touching */
            for (int i = 0; i < nchan; i++)
                u8g2_DrawStr(&m1_u8g2, 12, 22 + i * 8, chans[i].label);
            u8g2_DrawStr(&m1_u8g2, 3, 22 + sel * 8, ">");
            m1_u8g2_nextpage();
        }

        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret != pdTRUE || q_item.q_evt_type != Q_EVENT_KEYPAD)
            continue;
        xQueueReceive(button_events_q_hdl, &this_button_status, 0);

        if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
         || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (running) { m1_esp_client_zb_flood_stop(); running = false; redraw = true; continue; }
            m1_esp_client_zb_flood_stop();   /* belt-and-braces */
            xQueueReset(main_q_hdl);
            return;
        }
        if (running)
            continue;

        if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
        {
            sel = (sel > 0) ? sel - 1 : nchan - 1; redraw = true;
        }
        else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
        {
            sel = (sel < nchan - 1) ? sel + 1 : 0; redraw = true;
        }
        else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK
              || this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (m1_esp_client_zb_flood_start(chans[sel].ch))
            {
                running = true;
                m1_u8g2_firstpage();
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
                u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
                draw_title_bar("Beacon Flood");
                u8g2_DrawStr(&m1_u8g2, 2, 30, "Flooding...");
                u8g2_DrawStr(&m1_u8g2, 2, 42, chans[sel].label);
                u8g2_DrawStr(&m1_u8g2, 2, 58, "BACK to stop");
                m1_u8g2_nextpage();
            }
            else
            {
                show_message("Beacon Flood", "start failed", NULL, 1200);
                redraw = true;
            }
        }
    }

wait_back:
    while (1)
    {
        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK
             || this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
                break;
        }
    }
    m1_esp_client_zb_flood_stop();
    xQueueReset(main_q_hdl);
}
