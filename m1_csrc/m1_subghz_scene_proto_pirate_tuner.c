/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_proto_pirate_tuner.c
 * @brief  Proto Pirate — Timing Tuner scene (fully asynchronous).
 *
 * Captures raw OOK pulse durations from the radio and compares them against
 * a built-in table of automotive/garage protocol timing definitions.
 *
 * Layout (128×64):
 *   Row  0–10: Header bar — "Timing Tuner" + selected protocol name
 *   Row 11–12: Divider
 *   Row 13–23: Protocol ref:  "Ref  S:NNN  L:NNN ±NNN µs"
 *   Row 24–34: Measured short: "Meas S: avg (min-max) N=n"
 *   Row 35–45: Measured long:  "Meas L: avg (min-max) N=n"
 *   Row 46–54: Verdict line:   ">> MATCH <<" / error string
 *   Row 55–63: Bottom bar — sample count + RSSI bar
 *
 * Async model:
 *   - SubGhzEventRxData   : copy pulse_times[] → local ring buffer, analyse
 *   - SubGhzEventTick     : 200 ms — update RSSI, advance animation frame
 *   - UP / DOWN           : cycle through protocol reference table entries
 *   - BACK                : tear down RX, pop scene
 *
 * No blocking loops — all work happens in event handlers.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"
#include "m1_led_indicator.h"
#include "m1_lp5814.h"
#include "subghz_proto_pirate_timing.h"

/* Shared radio state (defined in m1_sub_ghz.c / m1_sub_ghz_decenc.c) */
extern SubGHz_DecEnc_t subghz_decenc_ctl;
extern S_M1_SubGHz_Scan_Config subghz_scan_config;

/* Radio control helpers (defined in m1_sub_ghz.c) */
extern void menu_sub_ghz_init(void);
extern void sub_ghz_set_opmode_ext(uint8_t opmode, uint8_t band,
                                   uint8_t channel, uint8_t tx_power);
extern void sub_ghz_rx_init_ext(void);
extern void sub_ghz_rx_start_ext(void);
extern void sub_ghz_rx_pause_ext(void);
extern void sub_ghz_rx_deinit_ext(void);
extern int16_t subghz_read_rssi_ext(void);
extern void subghz_apply_config_ext(uint8_t freq_idx, uint8_t mod_idx);
extern void SI446x_Change_Modem_OOK_PDTC(uint8_t value);
extern void subghz_pulse_handler_reset(void);

#define OOK_PDTC_VALUE      0x6C

/*============================================================================*/
/* Scene-local state                                                          */
/*============================================================================*/

/** Ring buffer for accumulated pulse samples. */
#define TUNER_SAMPLE_MAX    512

static uint16_t s_samples[TUNER_SAMPLE_MAX];
static uint16_t s_sample_head;   /**< Write index (wraps modulo TUNER_SAMPLE_MAX) */
static uint16_t s_sample_count;  /**< Total samples stored (capped at TUNER_SAMPLE_MAX) */

/** Currently selected protocol reference index */
static uint8_t s_proto_idx;

/** Analysis results (updated every time new pulse data arrives) */
static pptime_stats_t   s_stats;
static pptime_match_result_t s_verdict;

/** Animation frame counter (driven by tick) */
static uint8_t s_anim;

/** Cached RSSI (driven by tick) */
static int16_t s_rssi;

/** True once the radio has been started successfully */
static bool s_rx_active;

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

/** Push up to `count` pulse durations into the local ring buffer. */
static void push_pulses(const uint16_t *durations, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++)
    {
        s_samples[s_sample_head] = durations[i];
        s_sample_head = (uint16_t)((s_sample_head + 1U) % TUNER_SAMPLE_MAX);
        if (s_sample_count < TUNER_SAMPLE_MAX)
            s_sample_count++;
    }
}

/** Run timing analysis over the ring buffer with the selected protocol ref. */
static void run_analysis(void)
{
    /* Build a contiguous view of the ring buffer.  Because it may be split
     * at the wrap point, copy the logical sequence into a temporary buffer
     * before passing to pptime_analyze.  Use a small VLA-sized stack buffer
     * (max 512 × 2 = 1 KB) rather than a heap allocation. */
    uint16_t linear[TUNER_SAMPLE_MAX];
    uint16_t count = s_sample_count;
    uint16_t start = (s_sample_count < TUNER_SAMPLE_MAX)
                     ? 0U
                     : (uint16_t)((s_sample_head) % TUNER_SAMPLE_MAX);

    for (uint16_t i = 0; i < count; i++)
        linear[i] = s_samples[(start + i) % TUNER_SAMPLE_MAX];

    const pptime_proto_ref_t *ref =
        (s_proto_idx < pptime_proto_table_count)
        ? &pptime_proto_table[s_proto_idx]
        : NULL;

    pptime_analyze(linear, count, ref, &s_stats);
    s_verdict = pptime_match(&s_stats, ref);
}

/** Start RX with current app config (freq/mod).  Returns true on success. */
static bool start_rx(SubGhzApp *app)
{
    menu_sub_ghz_init();
    subghz_pulse_handler_reset();
    subghz_apply_config_ext(app->freq_idx, app->mod_idx);
    subghz_decenc_ctl.pulse_det_stat = PULSE_DET_ACTIVE;
    sub_ghz_set_opmode_ext(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
    SI446x_Change_Modem_OOK_PDTC(OOK_PDTC_VALUE);
    sub_ghz_rx_init_ext();
    sub_ghz_rx_start_ext();

    m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
    return true;
}

/** Stop RX cleanly. */
static void stop_rx(void)
{
    if (!s_rx_active)
        return;
    subghz_decenc_ctl.pulse_det_stat = PULSE_DET_IDLE;
    sub_ghz_rx_pause_ext();
    sub_ghz_rx_deinit_ext();
    sub_ghz_set_opmode_ext(SUB_GHZ_OPMODE_ISOLATED, subghz_scan_config.band, 0, 0);
    m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
    s_rx_active = false;
}

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

static void draw(SubGhzApp *app)
{
    (void)app;

    const pptime_proto_ref_t *ref =
        (s_proto_idx < pptime_proto_table_count)
        ? &pptime_proto_table[s_proto_idx]
        : NULL;

    char line[40];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* ---- Header: title + protocol name -------------------------------- */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 2, 9, "Timing Tuner");

    /* Small arrows to indicate UP/DOWN cycling */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 90, 9, ref ? ref->name : "---");

    u8g2_DrawHLine(&m1_u8g2, 0, 11, M1_LCD_DISPLAY_WIDTH);

    /* ---- Protocol reference ------------------------------------------- */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    if (ref)
        snprintf(line, sizeof(line), "Ref S:%u L:%u d:%u",
                 ref->te_short, ref->te_long, ref->te_delta);
    else
        snprintf(line, sizeof(line), "Ref: none");
    u8g2_DrawStr(&m1_u8g2, 2, 21, line);

    /* ---- Measured: short pulses --------------------------------------- */
    if (s_stats.n_short > 0)
        snprintf(line, sizeof(line), "S %ld (%ld-%ld) n=%u",
                 (long)s_stats.avg_short,
                 (long)s_stats.min_short,
                 (long)s_stats.max_short,
                 (unsigned)s_stats.n_short);
    else
        snprintf(line, sizeof(line), "S --");
    u8g2_DrawStr(&m1_u8g2, 2, 31, line);

    /* ---- Measured: long pulses ---------------------------------------- */
    if (s_stats.n_long > 0)
        snprintf(line, sizeof(line), "L %ld (%ld-%ld) n=%u",
                 (long)s_stats.avg_long,
                 (long)s_stats.min_long,
                 (long)s_stats.max_long,
                 (unsigned)s_stats.n_long);
    else
        snprintf(line, sizeof(line), "L --");
    u8g2_DrawStr(&m1_u8g2, 2, 41, line);

    /* ---- Verdict ------------------------------------------------------ */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    const char *verdict = pptime_match_str(s_verdict);

    if (s_verdict == PPTIME_MATCH_OK)
    {
        /* Invert-color highlight for a match */
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        u8g2_DrawBox(&m1_u8g2, 0, 43, M1_LCD_DISPLAY_WIDTH, 10);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        u8g2_DrawFrame(&m1_u8g2, 0, 43, M1_LCD_DISPLAY_WIDTH, 10);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        u8g2_DrawStr(&m1_u8g2, 20, 51, verdict);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    }
    else
    {
        u8g2_DrawStr(&m1_u8g2, 2, 51, verdict);
    }

    /* ---- Bottom bar: sample count + listening indicator --------------- */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    static const char *anim_chars[] = { "|", "/", "-", "\\" };
    snprintf(line, sizeof(line), "%s n=%u RSSI:%d",
             s_rx_active ? anim_chars[s_anim & 0x03u] : "!",
             (unsigned)s_sample_count,
             (int)s_rssi);
    u8g2_DrawStr(&m1_u8g2, 2, 62, line);

    /* Scrollbar hint: U/D for protocol cycling */
    u8g2_DrawStr(&m1_u8g2, 114, 62, "\x18\x19"); /* up/down arrows (U+2191/2193 in font) */

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Reset state */
    memset(s_samples, 0, sizeof(s_samples));
    s_sample_head  = 0;
    s_sample_count = 0;
    s_proto_idx    = 0;
    memset(&s_stats, 0, sizeof(s_stats));
    s_verdict      = PPTIME_MATCH_NO_DATA;
    s_anim         = 0;
    s_rssi         = -120;

    /* Start radio */
    s_rx_active = start_rx(app);

    /* 200 ms ticks for RSSI + animation refresh */
    subghz_scene_set_tick_period(app, 200U);

    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        /* ---- Navigation ----------------------------------------------- */
        case SubGhzEventBack:
            stop_rx();
            subghz_scene_set_tick_period(app, 0);
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            if (s_proto_idx > 0)
                s_proto_idx--;
            else
                s_proto_idx = (uint8_t)(pptime_proto_table_count - 1U);
            run_analysis();
            app->need_redraw = true;
            return true;

        case SubGhzEventDown:
            s_proto_idx = (uint8_t)((s_proto_idx + 1U) % pptime_proto_table_count);
            run_analysis();
            app->need_redraw = true;
            return true;

        /* ---- New pulse packet received ---------------------------------- */
        case SubGhzEventRxData:
        {
            uint16_t count = subghz_decenc_ctl.npulsecount;
            if (count > 0 && count <= PACKET_PULSE_COUNT_MAX)
                push_pulses(subghz_decenc_ctl.pulse_times, count);

            /* Reset the ISR buffer for the next packet */
            subghz_pulse_handler_reset();
            subghz_decenc_ctl.pulse_det_stat = PULSE_DET_ACTIVE;

            run_analysis();
            app->need_redraw = true;
            return true;
        }

        /* ---- Periodic tick (RSSI + animation) -------------------------- */
        case SubGhzEventTick:
            if (s_rx_active)
                s_rssi = subghz_read_rssi_ext();
            s_anim++;
            app->need_redraw = true;
            return true;

        default:
            break;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    (void)app;
    stop_rx();
    subghz_scene_set_tick_period(app, 0);
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_proto_pirate_tuner_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
