/* See COPYING.txt for license details. */
/*
 * m1_field_detector.c — NFC/RFID reader-field detector (see header).
 *
 * Native port of the SDK example app (m1-sdk/examples/nfc_rfid_detector). The
 * detection engine (m1_field_detect_*) is already firmware-native; this replaces
 * the ELF-app UI shell with a built-in screen so it lives in the menu.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_system.h"
#include "m1_display.h"
#include "m1_buzzer.h"
#include "m1_lp5814.h"
#include "m1_field_detect.h"
#include "m1_field_detector.h"

#define FIELD_PERSIST_TICKS 5      /* hysteresis: hold indicator for N cycles */
#define POLL_INTERVAL_MS    150

#define TITLE_H       12
#define NFC_ICON_X    8
#define NFC_ICON_Y    18
#define RFID_ICON_X   68
#define RFID_ICON_Y   18
#define ICON_W        50

/* 16x16 XBM icons (from the SDK app). */
static const uint8_t icon_nfc[] = {
    0x00, 0x00, 0xFC, 0x3F, 0x04, 0x20, 0xF4, 0x2F,
    0x14, 0x28, 0xD4, 0x2B, 0x54, 0x2A, 0x54, 0x2A,
    0x54, 0x2A, 0x54, 0x2A, 0xD4, 0x2B, 0x14, 0x28,
    0xF4, 0x2F, 0x04, 0x20, 0xFC, 0x3F, 0x00, 0x00,
};
static const uint8_t icon_rfid[] = {
    0xC0, 0x03, 0x30, 0x0C, 0xC8, 0x13, 0x24, 0x24,
    0x92, 0x49, 0x52, 0x4A, 0x4A, 0x52, 0x4A, 0x52,
    0x4A, 0x52, 0x4A, 0x52, 0x52, 0x4A, 0x92, 0x49,
    0x24, 0x24, 0xC8, 0x13, 0x30, 0x0C, 0xC0, 0x03,
};

/* ---- state ---- */
static uint8_t  s_nfc_weight;
static uint8_t  s_rfid_weight;
static uint32_t s_rfid_freq_hz;
static int      s_init_result;
static int      s_nfc_aux;
static int      s_nfc_opctl;
static int      s_rfid_raw;
static int      s_rfid_detect;

static void draw_detected(void)
{
    char buf[32];

    if (s_nfc_weight) {
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, NFC_ICON_X + 8, NFC_ICON_Y + 2, "NFC");
        u8g2_DrawXBM(&m1_u8g2, NFC_ICON_X + 10, NFC_ICON_Y + 6, 16, 16, icon_nfc);

        u8g2_DrawFrame(&m1_u8g2, NFC_ICON_X, NFC_ICON_Y + 26, ICON_W, 6);
        uint8_t fill_w = (uint8_t)((ICON_W - 2) * s_nfc_weight / FIELD_PERSIST_TICKS);
        u8g2_DrawBox(&m1_u8g2, NFC_ICON_X + 1, NFC_ICON_Y + 27, fill_w, 4);

        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, NFC_ICON_X + 2, 62, "13.56 MHz");
    }

    if (s_rfid_weight) {
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, RFID_ICON_X + 2, RFID_ICON_Y + 2, "LF RFID");
        u8g2_DrawXBM(&m1_u8g2, RFID_ICON_X + 10, RFID_ICON_Y + 6, 16, 16, icon_rfid);

        u8g2_DrawFrame(&m1_u8g2, RFID_ICON_X, RFID_ICON_Y + 26, ICON_W, 6);
        uint8_t fill_w = (uint8_t)((ICON_W - 2) * s_rfid_weight / FIELD_PERSIST_TICKS);
        u8g2_DrawBox(&m1_u8g2, RFID_ICON_X + 1, RFID_ICON_Y + 27, fill_w, 4);

        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        if (s_rfid_freq_hz > 0) {
            snprintf(buf, sizeof(buf), "%lu.%02lu kHz",
                     (unsigned long)(s_rfid_freq_hz / 1000),
                     (unsigned long)((s_rfid_freq_hz % 1000) / 10));
            u8g2_DrawStr(&m1_u8g2, RFID_ICON_X, 62, buf);
        } else {
            u8g2_DrawStr(&m1_u8g2, RFID_ICON_X, 62, "~125 kHz");
        }
    }

    if (s_nfc_weight && s_rfid_weight)
        u8g2_DrawVLine(&m1_u8g2, 64, TITLE_H + 2, 64 - TITLE_H - 4);
}

static void draw_screen(void)
{
    char buf[40];

    u8g2_DrawBox(&m1_u8g2, 0, 0, M1_LCD_DISPLAY_WIDTH, TITLE_H);
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 16, 10, "Field Detector");
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    if (s_nfc_weight || s_rfid_weight) {
        draw_detected();
    } else {
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        snprintf(buf, sizeof(buf), "NFC:%s OP:0x%02X",
                 s_init_result == 0 ? "OK" : "FAIL", (unsigned)s_nfc_opctl);
        u8g2_DrawStr(&m1_u8g2, 2, 24, buf);
        snprintf(buf, sizeof(buf), "AUX:0x%02X efd:%d osc:%d",
                 (unsigned)s_nfc_aux, (s_nfc_aux >> 6) & 1, (s_nfc_aux >> 4) & 1);
        u8g2_DrawStr(&m1_u8g2, 2, 34, buf);
        snprintf(buf, sizeof(buf), "RFID delta:%d det:%d", s_rfid_raw, s_rfid_detect);
        u8g2_DrawStr(&m1_u8g2, 2, 44, buf);
        u8g2_DrawStr(&m1_u8g2, 2, 56, "Hold near reader. BACK=exit");
    }
}

void m1_field_detector_run(void)
{
    S_M1_Buttons_Status btn;
    S_M1_Main_Q_t       q_item;

    s_nfc_weight = s_rfid_weight = 0;
    s_rfid_freq_hz = 0;
    s_nfc_aux = s_nfc_opctl = s_rfid_raw = s_rfid_detect = 0;

    s_init_result = m1_field_detect_start();
    m1_buzzer_set(2000, 50);   /* init confirm beep */

    while (1)
    {
        uint32_t freq = 0;
        bool nfc  = m1_field_detect_nfc();     /* call first — caches regs */
        s_nfc_aux   = m1_field_detect_nfc_raw();
        s_nfc_opctl = m1_field_detect_nfc_opctl();
        s_rfid_detect = m1_field_detect_rfid(&freq);
        s_rfid_raw    = m1_field_detect_rfid_raw();

        if (s_rfid_detect) { s_rfid_weight = FIELD_PERSIST_TICKS; s_rfid_freq_hz = freq; }
        else if (s_rfid_weight > 0) s_rfid_weight--;

        if (nfc) s_nfc_weight = FIELD_PERSIST_TICKS;
        else if (s_nfc_weight > 0) s_nfc_weight--;

        /* RGB feedback: blue=NFC, green=RFID. */
        if (nfc || s_rfid_detect) {
            lp5814_led_on_Red(0);
            lp5814_led_on_Green(s_rfid_detect ? 80 : 0);
            lp5814_led_on_Blue(nfc ? 80 : 0);
        } else if (!s_nfc_weight && !s_rfid_weight) {
            lp5814_all_off_RGB();
        }

        m1_u8g2_firstpage();
        do {
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            draw_screen();
        } while (m1_u8g2_nextpage());

        /* Poll input with the detection cadence; BACK exits. */
        if (xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(POLL_INTERVAL_MS)) == pdTRUE
            && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &btn, 0);
            if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK) {
                xQueueReset(main_q_hdl);
                break;
            }
        }
    }

    lp5814_all_off_RGB();
    m1_field_detect_stop();
}
