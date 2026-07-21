/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_read.c
 * @brief  Sub-GHz Read Scene — Flipper-consistent protocol-aware receiver.
 *
 * Automatically starts listening on scene entry (no manual OK press needed).
 * Decoded signals are shown in a scrollable history list with LED + buzzer
 * feedback on each decode.
 *
 * Layout (128x64):
 *   Row 0-9:    Status bar (freq, mod, state)
 *   Row 10-13:  RSSI bar
 *   Row 14-51:  Content area (history list or "listening...")
 *   Row 52-63:  Bottom bar (button hints)
 *
 * Navigation:
 *   OK    = View detail (history) / Open history (live) / Info (detail)
 *   BACK  = Close view layer / Exit to menu
 *   L/R   = Cycle frequency preset (retunes live during RX)
 *   U/D   = Scroll history list
 *   DOWN  = Config (no signals) / Save (detail view)
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_tasks.h"
#include "m1_system.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_lp5814.h"
#include "m1_buzzer.h"
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_api.h"
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"
#include "m1_ring_buffer.h"
#include "m1_storage.h"
#include "m1_sdcard_man.h"
#include "m1_file_util.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"
#include "flipper_subghz.h"
#include "subghz_autosave.h"

/*============================================================================*/
/* Extern references to shared radio state (defined in m1_sub_ghz.c)          */
/*============================================================================*/

/* Frequency & modulation preset tables */
extern const char *subghz_freq_labels[];
extern const char *subghz_mod_labels[];

/* Protocol name table */

/* Hopper: region-aware frequency getter (reads ism_band_region at runtime) */
#include "subghz_freq_presets.h"   /* SUBGHZ_HOPPER_FREQ_COUNT, subghz_get_hopper_freqs() */
extern const uint32_t *subghz_get_hopper_freqs_ext(void);

/* RSSI read helper */
extern int16_t subghz_read_rssi_ext(void);

/* Apply config to radio hardware */
extern void subghz_apply_config_ext(uint8_t freq_idx, uint8_t mod_idx);

/* Get frequency in Hz for a preset index */
extern uint32_t subghz_get_freq_hz_ext(uint8_t freq_idx);

/* Live retune to arbitrary frequency (Hz) without full RX re-init */
extern void subghz_retune_freq_hz_ext(uint32_t freq_hz);

/* Radio control */
extern void sub_ghz_rx_init_ext(void);
extern void sub_ghz_rx_start_ext(void);
extern void sub_ghz_rx_pause_ext(void);
extern void sub_ghz_rx_deinit_ext(void);
extern void sub_ghz_set_opmode_ext(uint8_t opmode, uint8_t band, uint8_t channel, uint8_t tx_power);
extern uint8_t sub_ghz_ring_buffers_init_ext(void);
extern void sub_ghz_ring_buffers_deinit_ext(void);
extern void sub_ghz_tx_raw_deinit_ext(void);

/* Shared radio state */
extern S_M1_SubGHz_Scan_Config subghz_scan_config;
extern SubGHz_DecEnc_t subghz_decenc_ctl;
extern uint8_t subghz_record_mode_flag;

/* Decoder polling */
extern bool subghz_decenc_read(SubGHz_Dec_Info_t *out, bool raw_mode);

/* Phase 12 — receiver history quality-of-life toggles (m1_sub_ghz.c) */
extern bool subghz_get_remove_duplicates_ext(void);
extern bool subghz_get_delete_old_signals_ext(void);

/* Decoder state reset */
extern void subghz_pulse_handler_reset(void);

/* SI446x control */
extern void SI446x_Change_Modem_OOK_PDTC(uint8_t value);

/* Save format accessor */
extern uint8_t subghz_get_save_fmt_ext(void);

/*============================================================================*/
/* Autosave — save decoded signals to SD automatically                        */
/*============================================================================*/

/**
 * @brief  Auto-save a decoded signal to the autosave directory.
 *
 * Uses subghz_autosave.h for filename generation and duplicate detection.
 * Skips save if the key is zero, the signal is a duplicate, or the
 * directory cannot be created.
 */
static void autosave_signal(const SubGHz_History_Entry_t *entry)
{
    if (entry == NULL || entry->info.key == 0)
        return;

    /* Duplicate detection: skip if same protocol+key already saved */
    if (subghz_autosave_is_duplicate(
            protocol_text[entry->info.protocol], entry->info.key))
        return;

    /* Ensure parent and autosave directories exist */
    FRESULT res = fs_directory_ensure("/SUBGHZ");
    if (res != FR_OK)
        return;
    res = fs_directory_ensure(SUBGHZ_AUTOSAVE_DIR_PATH);
    if (res != FR_OK)
        return;

    /* Build filename */
    uint8_t fmt = subghz_get_save_fmt_ext();
    char path[80];
    if (subghz_autosave_make_path(path, sizeof(path),
            protocol_text[entry->info.protocol],
            entry->info.key,
            HAL_GetTick(),
            fmt == 1) == 0)
        return;

    bool saved;
    if (fmt == 1)
        saved = flipper_subghz_save_m1native_key(path,
            entry->frequency, "FuriHalSubGhzPresetOok650Async",
            protocol_text[entry->info.protocol],
            entry->info.bit_len, entry->info.key, entry->info.te);
    else
        saved = flipper_subghz_save_key(path,
            entry->frequency, "FuriHalSubGhzPresetOok650Async",
            protocol_text[entry->info.protocol],
            entry->info.bit_len, entry->info.key, entry->info.te);

    /* Only record as "seen" after a successful save */
    if (saved)
        subghz_autosave_record(
            protocol_text[entry->info.protocol], entry->info.key);
}

/*============================================================================*/
/* Display constants                                                          */
/*============================================================================*/

#define CONTENT_Y_START   15  /* Below RSSI bar */
#define CONTENT_Y_END     51  /* Above bottom bar */
#define HISTORY_ROW_H      8  /* Pixels per history row */
#define HISTORY_VISIBLE    3  /* Number of visible history items */

/* Hopper */
#define HOPPER_DWELL_MS        150

/* SI4463 config */
#define OOK_PDTC_VALUE  0x6C

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void start_rx(SubGhzApp *app);
static void resume_rx(SubGhzApp *app);
static void stop_rx(SubGhzApp *app);

static void scene_on_enter(SubGhzApp *app)
{
    /* Ensure we're in protocol-decode mode (not raw) regardless of previous
     * scene state.  Without this, a stale subghz_record_mode_flag=1 causes
     * the event loop to skip the pulse handler entirely. */
    subghz_record_mode_flag = 0;

    /* Detect whether we're returning from a child scene (Config,
     * ReceiverInfo, SaveName) vs. entering fresh from the Menu.
     * The explicit resume_from_child flag is set when Read pushes
     * a child scene, ensuring we don't accidentally resume when
     * re-entering Read from Menu (where history may still exist
     * from a previous session). */
    bool is_resume = app->resume_from_child;
    app->resume_from_child = false;

    /* Reset view state but preserve history across child-scene navigation.
     * History is only cleared on the first enter from the Menu scene
     * (when count is 0) or can be cleared explicitly by the user. */
    app->history_sel    = 0;
    app->history_scroll = 0;
    app->history_view   = false;
    app->detail_view    = false;
    app->rssi           = -120;

    /* If history already has signals (returning from child scene),
     * keep has_decoded and last_decoded so the display shows them.
     * Only reset when starting fresh. */
    if (!is_resume)
    {
        app->has_decoded = false;
        memset(&app->last_decoded, 0, sizeof(app->last_decoded));
        subghz_autosave_dup_reset();
    }

    /* Auto-start RX (Flipper-consistent: entering Read immediately starts
     * listening — no need to press OK first). */
    if (is_resume)
        resume_rx(app);
    else
        start_rx(app);
    app->need_redraw = true;
}

/**
 * @brief  Full RX start — used on first entry from Menu.
 *
 * Resets the pulse handler state, does a full radio power-cycle via
 * menu_sub_ghz_init(), and initializes hopper from scratch.
 */
static void start_rx(SubGhzApp *app)
{
    /* Reset pulse handler state to ensure clean decode session */
    subghz_pulse_handler_reset();

    /* Apply config to radio */
    subghz_apply_config_ext(app->freq_idx, app->mod_idx);

    /* Ensure radio is in a clean, known state — matches the Spectrum
     * Analyzer pattern of always doing a full radio reset + config load
     * before starting RX.  This guarantees the SI4463 recovers properly
     * even if a previous blocking delegate powered the radio off via
     * menu_sub_ghz_exit(). */
    menu_sub_ghz_init();

    /* Track the active frequency in Hz */
    app->current_freq_hz = subghz_get_freq_hz_ext(app->freq_idx);

    /* Initialize hopper if enabled */
    app->hopper_active = app->hopping;
    app->hopper_idx = 0;
    if (app->hopper_active)
    {
        app->hopper_freq = subghz_get_hopper_freqs_ext()[0];
        app->current_freq_hz = app->hopper_freq;
    }

    /* Start RX */
    subghz_decenc_ctl.pulse_det_stat = PULSE_DET_ACTIVE;
    sub_ghz_set_opmode_ext(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
    SI446x_Change_Modem_OOK_PDTC(OOK_PDTC_VALUE);
    sub_ghz_rx_init_ext();
    sub_ghz_rx_start_ext();

    app->read_state = SubGhzReadStateRx;

    /* LED: cyan fast blink to indicate active listening */
    m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
}

/**
 * @brief  Lightweight RX resume — used when returning from child scenes.
 *
 * Unlike start_rx(), this skips:
 *   - subghz_pulse_handler_reset() — preserves partial decode state
 *   - menu_sub_ghz_init() — expensive full radio reset is unnecessary
 *     since stop_rx() only puts the SI4463 to SLEEP (not power-off)
 *   - Hopper index reset — preserves position in hop sequence
 *
 * Config changes made in the Config scene (freq/mod) are applied via
 * subghz_apply_config_ext() which updates the internal radio state.
 */
static void resume_rx(SubGhzApp *app)
{
    /* Apply config to radio (picks up any Config scene changes) */
    subghz_apply_config_ext(app->freq_idx, app->mod_idx);

    /* Track the active frequency in Hz */
    app->current_freq_hz = subghz_get_freq_hz_ext(app->freq_idx);

    /* Restore hopper from saved position (not reset to 0) */
    app->hopper_active = app->hopping;
    if (app->hopper_active)
        app->current_freq_hz = app->hopper_freq;

    /* Start RX — radio was in ISOLATED mode from stop_rx(),
     * so we just need to switch back to RX mode + init capture.
     * When hopping is active, explicitly retune to the saved hopper
     * frequency so the hardware matches the restored app/UI state. */
    subghz_decenc_ctl.pulse_det_stat = PULSE_DET_ACTIVE;
    sub_ghz_set_opmode_ext(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
    if (app->hopper_active)
        subghz_retune_freq_hz_ext(app->hopper_freq);
    SI446x_Change_Modem_OOK_PDTC(OOK_PDTC_VALUE);
    sub_ghz_rx_init_ext();
    sub_ghz_rx_start_ext();

    app->read_state = SubGhzReadStateRx;

    /* LED: cyan fast blink to indicate active listening */
    m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
}

static void stop_rx(SubGhzApp *app)
{
    app->hopper_active = false;
    sub_ghz_rx_pause_ext();
    sub_ghz_rx_deinit_ext();
    sub_ghz_set_opmode_ext(SUB_GHZ_OPMODE_ISOLATED, subghz_scan_config.band, 0, 0);
    subghz_decenc_ctl.pulse_det_stat = PULSE_DET_IDLE;

    /* LED off */
    m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);

    app->read_state = SubGhzReadStateIdle;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        case SubGhzEventBack:
            if (app->detail_view)
            {
                /* Exit signal detail → back to history list */
                app->detail_view = false;
                app->need_redraw = true;
            }
            else if (app->history_view)
            {
                /* Exit history list → back to live view */
                app->history_view = false;
                app->need_redraw = true;
            }
            else
            {
                /* Stop RX and exit to menu.
                 * Flipper-consistent: BACK always exits the receiver scene.
                 * RX is stopped in scene_on_exit. */
                subghz_scene_pop(app);
            }
            return true;

        case SubGhzEventOk:
            if (app->read_state == SubGhzReadStateRx)
            {
                if (app->history_view && !app->detail_view &&
                    app->history.count > 0)
                {
                    /* Open detail view of selected signal */
                    app->detail_view = true;
                    app->need_redraw = true;
                }
                else if (app->detail_view)
                {
                    /* Push Receiver Info scene (full detail + save/send) */
                    app->resume_from_child = true;
                    subghz_scene_push(app, SubGhzSceneReceiverInfo);
                }
                else if (!app->history_view && app->has_decoded)
                {
                    /* Live decode view: OK opens history list */
                    app->history_view = true;
                    app->history_sel = 0;
                    app->history_scroll = 0;
                    app->need_redraw = true;
                }
            }
            else if (app->read_state == SubGhzReadStateIdle)
            {
                /* Restart RX (e.g., after manual stop) */
                start_rx(app);
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventLeft:
            /* Open Config (consistent with Read Raw scene UX: LEFT button
             * with arrowleft icon opens Config).  Available whenever
             * we're not already in a sub-view. */
            if (!app->history_view && !app->detail_view)
            {
                app->resume_from_child = true;
                subghz_scene_push(app, SubGhzSceneConfig);
            }
            return true;

        case SubGhzEventRight:
            /* Unused — frequency selection is via Config (LEFT button) */
            return true;

        case SubGhzEventUp:
            if (app->history_view && !app->detail_view)
            {
                /* Scroll up in history */
                if (app->history_sel > 0)
                    app->history_sel--;
                if (app->history_sel < app->history_scroll)
                    app->history_scroll = app->history_sel;
                app->need_redraw = true;
            }
            else if (!app->history_view && app->history.count > 0)
            {
                /* Open history list (Flipper: UP = scroll/open list) */
                app->history_view = true;
                app->history_sel = 0;
                app->history_scroll = 0;
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventDown:
            if (app->history_view && !app->detail_view)
            {
                /* Scroll down in history */
                if (app->history_sel + 1 < app->history.count)
                    app->history_sel++;
                if (app->history_sel >= app->history_scroll + HISTORY_VISIBLE)
                    app->history_scroll = app->history_sel - HISTORY_VISIBLE + 1;
                app->need_redraw = true;
            }
            else if (app->detail_view)
            {
                /* Save signal from detail view */
                app->resume_from_child = true;
                subghz_scene_push(app, SubGhzSceneSaveName);
            }
            /* In live view: DOWN is unused — Config is on LEFT */
            return true;

        case SubGhzEventHopperTick:
            if (app->hopper_active && app->read_state == SubGhzReadStateRx)
            {
                /* Always advance to the next hopper frequency unconditionally.
                 * Cycling is time-based (200 ms dwell per frequency) so the
                 * frequency display always rotates through the hop list,
                 * regardless of ambient RSSI. */
                app->hopper_idx = (app->hopper_idx + 1) % SUBGHZ_HOPPER_FREQ_COUNT;
                app->hopper_freq = subghz_get_hopper_freqs_ext()[app->hopper_idx];
                app->current_freq_hz = app->hopper_freq;
                subghz_retune_freq_hz_ext(app->hopper_freq);
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventRxData:
            /* Decode event — pulse handler already detected a full decode */
            if (app->read_state == SubGhzReadStateRx)
            {
                SubGHz_Dec_Info_t decoded;
                if (subghz_decenc_read(&decoded, false) && decoded.key != 0)
                {
                    /* Use the actual active frequency (preset or hopper) */
                    uint32_t freq = app->current_freq_hz;

                    /* Snapshot ring state and settings so we can detect when
                     * add_ex() drops the new decode (delete_old_signals=false
                     * + ring full).  Read settings once to avoid TOCTOU if
                     * another task changes them between the two checks. */
                    uint8_t head_before  = app->history.head;
                    uint8_t count_before = app->history.count;
                    bool remove_dupes    = subghz_get_remove_duplicates_ext();
                    bool delete_old      = subghz_get_delete_old_signals_ext();

                    subghz_history_add_ex(&app->history, &decoded, freq,
                                          remove_dupes, delete_old);

                    bool inserted = (app->history.head  != head_before) ||
                                    (app->history.count != count_before);
                    /* A merge can only happen when remove_duplicates=true;
                     * gate the field comparison on that flag so a drop that
                     * leaves the top entry matching the new decode
                     * (remove_duplicates=false + ring full + delete_old=false)
                     * is never misidentified as a merge. */
                    bool merged = false;
                    if (!inserted && remove_dupes && count_before > 0)
                    {
                        const SubGHz_History_Entry_t *top =
                            subghz_history_get(&app->history, 0);
                        if (top &&
                            top->info.protocol == decoded.protocol &&
                            top->info.key      == decoded.key &&
                            top->info.bit_len  == decoded.bit_len)
                        {
                            merged = true;
                        }
                    }

                    /* Only treat as a real decode if inserted or merged.
                     * A dropped decode must not update selection or autosave,
                     * which would otherwise re-save an unrelated old entry. */
                    if (inserted || merged)
                    {
                        app->last_decoded = decoded;
                        app->has_decoded = true;

                        /* Auto-switch to history view once we have signals
                         * (Flipper-consistent: receiver always shows the list) */
                        if (app->history.count > 0 && !app->detail_view)
                        {
                            app->history_view = true;
                            /* Auto-select newest signal (index 0 = most recent) */
                            app->history_sel = 0;
                            app->history_scroll = 0;
                        }

                        /* Decode feedback: green LED flash + sound */
                        m1_led_fast_blink(LED_BLINK_ON_GREEN, LED_FASTBLINK_PWM_H, LED_FASTBLINK_ONTIME_H);
                        if (app->sound)
                            m1_buzzer_notification();

                        /* Autosave: write decoded signal to SD if enabled.
                         * Only on a real new insertion — merges already
                         * re-saved the same key earlier. */
                        if (app->autosave && inserted)
                        {
                            const SubGHz_History_Entry_t *newest =
                                subghz_history_get(&app->history, 0);
                            autosave_signal(newest);
                        }

                        /* LED will auto-restore to RX color on next redraw cycle
                         * (fast blink timer handles the flash duration) */
                    }
                }

                app->need_redraw = true;
            }
            return true;

        default:
            break;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    /* Ensure radio is stopped when leaving scene */
    if (app->read_state == SubGhzReadStateRx)
        stop_rx(app);
}

static void draw(SubGhzApp *app)
{
    char line[40];

    /* Update RSSI during active RX (called periodically from event loop) */
    if (app->read_state == SubGhzReadStateRx)
        app->rssi = subghz_read_rssi_ext();

    m1_u8g2_firstpage();

    /* --- Status bar --- */
    const char *freq = subghz_freq_labels ? subghz_freq_labels[app->freq_idx] : "???";
    const char *mod  = subghz_mod_labels  ? subghz_mod_labels[app->mod_idx]   : "???";
    const char *state = NULL;

    switch (app->read_state)
    {
        case SubGhzReadStateIdle:    state = "IDLE"; break;
        case SubGhzReadStateRx:      state = "RX";   break;
        case SubGhzReadStateStopped: state = "STOP"; break;
    }

    if (app->hopper_active && app->read_state == SubGhzReadStateRx)
    {
        /* Show current hopper frequency instead of preset.
         * Use integer arithmetic — embedded printf (nano.specs) has no %f support. */
        snprintf(line, sizeof(line), "%lu.%02lu",
                 (unsigned long)(app->hopper_freq / 1000000UL),
                 (unsigned long)((app->hopper_freq % 1000000UL) / 10000UL));
        subghz_status_bar_draw(line, mod, state, false);
    }
    else
    {
        subghz_status_bar_draw(freq, mod, state, false);
    }

    /* --- RSSI bar (only when active) --- */
    if (app->read_state == SubGhzReadStateRx)
        subghz_rssi_bar_draw(app->rssi);

    /* --- Content area --- */
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    if (app->detail_view && app->history.count > 0)
    {
        /* Signal detail view */
        const SubGHz_History_Entry_t *e = subghz_history_get(&app->history, app->history_sel);
        if (e)
        {
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, CONTENT_Y_START + 7, protocol_text[e->info.protocol]);

            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            snprintf(line, sizeof(line), "Key: 0x%lX %dbit", (uint32_t)e->info.key, e->info.bit_len);
            u8g2_DrawStr(&m1_u8g2, 2, CONTENT_Y_START + 17, line);

            snprintf(line, sizeof(line), "TE:%d RSSI:%ddBm", e->info.te, e->info.rssi);
            u8g2_DrawStr(&m1_u8g2, 2, CONTENT_Y_START + 26, line);

            if (e->count > 1)
            {
                snprintf(line, sizeof(line), "Received x%d", e->count);
                u8g2_DrawStr(&m1_u8g2, 2, CONTENT_Y_START + 35, line);
            }
        }
    }
    else if (app->history_view && app->history.count > 0)
    {
        /* History list (Flipper-style: shows signal count in header) */
        snprintf(line, sizeof(line), "Signals: %d", app->history.count);
        u8g2_DrawStr(&m1_u8g2, 2, CONTENT_Y_START + 6, line);

        uint8_t vis = HISTORY_VISIBLE;
        if (vis > app->history.count) vis = app->history.count;

        for (uint8_t i = 0; i < vis; i++)
        {
            uint8_t idx = app->history_scroll + i;
            if (idx >= app->history.count) break;
            const SubGHz_History_Entry_t *e = subghz_history_get(&app->history, idx);
            if (!e) break;

            uint8_t y = CONTENT_Y_START + 8 + i * HISTORY_ROW_H;

            if (idx == app->history_sel)
            {
                u8g2_DrawRBox(&m1_u8g2, 0, y, M1_LCD_DISPLAY_WIDTH, HISTORY_ROW_H, 2);
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            }

            snprintf(line, sizeof(line), "%s 0x%lX",
                     protocol_text[e->info.protocol], (uint32_t)e->info.key);
            u8g2_DrawStr(&m1_u8g2, 2, y + 6, line);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        }
    }
    else if (app->read_state == SubGhzReadStateIdle)
    {
        /* Idle screen: show antenna icon + instructions (fallback if RX failed) */
        u8g2_DrawXBMP(&m1_u8g2, 0, 14, 50, 27, subghz_antenna_50x27);
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 54, 28, "Press OK");
        u8g2_DrawStr(&m1_u8g2, 54, 38, "to listen");
    }
    else if (app->read_state == SubGhzReadStateRx)
    {
        /* Actively listening, no decode yet — show waiting indicator */
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        if (app->hopper_active)
            u8g2_DrawStr(&m1_u8g2, 20, 30, "Scanning...");
        else
            u8g2_DrawStr(&m1_u8g2, 20, 30, "Listening...");
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 20, 42, "Waiting for signal");
    }

    /* --- Bottom bar --- */
    if (app->read_state == SubGhzReadStateIdle)
    {
        subghz_button_bar_draw(
            arrowleft_8x8, "Config",
            NULL, "Listen",
            NULL, NULL);
    }
    else if (app->detail_view)
    {
        subghz_button_bar_draw(
            arrowdown_8x8, "Save",
            NULL, "Info",
            NULL, NULL);
    }
    else if (app->history_view)
    {
        /* DOWN scrolls (implicit), OK opens detail */
        subghz_button_bar_draw(
            NULL, NULL,
            NULL, "View",
            NULL, NULL);
    }
    else
    {
        /* Active RX, no signals yet */
        subghz_button_bar_draw(
            arrowleft_8x8, "Config",
            NULL, NULL,
            NULL, NULL);
    }

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_read_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
