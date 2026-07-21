/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_decode_raw.c
 * @brief  Sub-GHz Read-Raw offline decode-results scene (Phase 6).
 *
 * Pushed by the MoreRAW scene when the user selects "Decode" on a loaded
 * RAW file.  Loads @ref SubGhzApp.raw_filepath, runs the offline RAW
 * decode engine (subghz_decode_raw_offline) across every registered
 * protocol, and presents the matched protocols as a scrollable list with
 * a detail view.
 *
 *   List view (default):
 *     - Header: "Decode Results"
 *     - Body: "Decoded: N" followed by scrollable rows showing
 *             "<protocol> 0x<key>" with selection highlight.
 *     - UP / DOWN scroll, OK opens the detail view for the selection.
 *     - "No protocols decoded" message when the engine returned 0 matches.
 *
 *   Detail view (OK from list):
 *     - Protocol name, Key + bit length, SN/Btn (when available),
 *       TE, Frequency.
 *     - OK = Send (inline async TX — no Transmitter scene push).
 *     - DOWN = Save (prompt filename → write persistent .sub key file,
 *                     then push to Saved file browser).
 *     - BACK returns to the list view; second BACK pops the scene.
 *
 * The scene loads the file eagerly on entry into a file-scope
 * @ref flipper_subghz_signal_t (16 KB raw-sample buffer) to avoid blowing
 * the RTOS task stack — matching the convention already used by Read Raw
 * and the SavedMenu decode path.
 *
 * Phase 6 split: this overlay was previously inlined inside
 * `m1_subghz_scene_read_raw.c` behind `rr_in_decode` / `rr_decode_*`
 * file-scope statics and a separate `do_rr_decode()` helper.  Extracting
 * it removes ~140 lines of overlay state from Read Raw and gives the
 * decode view its own dedicated scene lifetime.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "ff.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_lp5814.h"
#include "m1_scene.h"
#include "m1_virtual_kb.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_decenc.h"
#include "flipper_subghz.h"
#include "subghz_protocol_registry.h"
#include "subghz_raw_decoder.h"

extern SubGHz_DecEnc_t subghz_decenc_ctl;
extern void subghz_pulse_handler_reset(void);
extern uint8_t subghz_get_save_fmt_ext(void);
extern bool subghz_protocol_is_static_ext(uint16_t protocol);

/*============================================================================*/
/* Scene state                                                                */
/*============================================================================*/

#define DECODE_RAW_MAX     16   /**< max distinct protocols shown */
/** Temp .sub file written for Transmitter — must be unlinked by this scene
 *  on return from Transmitter (resume_from_child path) and on real exit. */
#define DECODE_TMP_PATH    "/SUBGHZ/_decode_tmp.sub"

static SubGhzRawDecodeResult decode_results[DECODE_RAW_MAX];
static uint8_t  decode_count;
static uint8_t  decode_sel;
static uint8_t  decode_scroll;
static bool     decode_detail;
static bool     s_save_failed;  /**< Brief "Save failed" overlay flag */

/* Inline TX state — replaces the Transmitter scene push so Send happens
 * directly from the decode detail view without an extra screen. */
static bool     s_tx_active;    /**< True while async TX is in flight */
static uint8_t  s_tx_anim;     /**< Animation phase counter (0-3) */
static char     s_tx_tmp[80];  /**< Temp .sgh path to unlink after TX */

/* Static to avoid a 16 KB stack allocation (raw_data[8192] lives inside the
 * struct).  This scene only runs from the main RTOS task while the scene is
 * active; non-reentrancy of a file-scope static is safe. */
static flipper_subghz_signal_t s_sig;

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

static void unlink_tmp(void)
{
    f_unlink(DECODE_TMP_PATH);
}

/** Tear down any in-flight inline TX and return to the decode detail view. */
static void inline_tx_teardown(void)
{
    if (!s_tx_active)
        return;
    if (sub_ghz_replay_async_is_active())
        sub_ghz_replay_abort();
    if (s_tx_tmp[0] != '\0')
    {
        (void)f_unlink(s_tx_tmp);
        s_tx_tmp[0] = '\0';
    }
    s_tx_active = false;
}

/** Load the file referenced by @p app->raw_filepath and run offline decode.
 *  Results are stored in @ref decode_results / @ref decode_count.  Failures
 *  leave @ref decode_count == 0 — the draw path will render the "No
 *  protocols decoded" message in that case. */
static void load_and_decode(const SubGhzApp *app)
{
    decode_count  = 0;
    decode_sel    = 0;
    decode_scroll = 0;
    decode_detail = false;
    s_save_failed = false;
    s_tx_active   = false;
    s_tx_tmp[0]   = '\0';

    if (app->raw_filepath[0] == '\0')
        return;

    char full_path[80];
    snprintf(full_path, sizeof(full_path), "0:%s", app->raw_filepath);

    memset(&s_sig, 0, sizeof(s_sig));
    if (!flipper_subghz_load(full_path, &s_sig))
        return;

    if (s_sig.type != FLIPPER_SUBGHZ_TYPE_RAW || s_sig.raw_count == 0)
        return;

    subghz_pulse_handler_reset();
    subghz_decenc_ctl.ndecodedrssi = 0;

    decode_count = subghz_decode_raw_offline(
        s_sig.raw_data, s_sig.raw_count, s_sig.frequency,
        decode_results, DECODE_RAW_MAX,
        subghz_registry_decode_try_fn, NULL);

    /* Filter out low-confidence results — momentum parity.
     * Real remote transmissions repeat the same packet multiple times,
     * so a genuine protocol match will have repeat_count >= 2.  When
     * we have confirmed results (repeat_count >= 2), discard any
     * single-hit results which are likely false positives from timing
     * coincidences across similar OOK protocol families. */
    {
        bool has_confirmed = false;
        for (uint8_t i = 0; i < decode_count; i++)
        {
            if (decode_results[i].repeat_count >= 2)
            {
                has_confirmed = true;
                break;
            }
        }
        if (has_confirmed)
        {
            uint8_t dst = 0;
            for (uint8_t src = 0; src < decode_count; src++)
            {
                if (decode_results[src].repeat_count >= 2)
                {
                    if (dst != src)
                        decode_results[dst] = decode_results[src];
                    dst++;
                }
            }
            decode_count = dst;
        }
    }
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    load_and_decode(app);
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* Any keypress dismisses the "Save failed" overlay.  The flag is only
     * set while in the detail view, so clearing is safe. */
    if (s_save_failed)
    {
        s_save_failed = false;
        app->need_redraw = true;
        return true;
    }

    /* ---- Inline TX event handling ---- */
    if (s_tx_active)
    {
        switch (event)
        {
            case SubGhzEventTxComplete:
            {
                sub_ghz_replay_async_status_t st =
                    sub_ghz_replay_continue_async(false);
                if (st == SUBGHZ_REPLAY_ASYNC_DONE ||
                    st == SUBGHZ_REPLAY_ASYNC_ERROR)
                {
                    inline_tx_teardown();
                    subghz_scene_set_tick_period(app, 0);
                }
                app->need_redraw = true;
                return true;
            }
            case SubGhzEventBack:
                inline_tx_teardown();
                subghz_scene_set_tick_period(app, 0);
                app->need_redraw = true;
                return true;
            case SubGhzEventTick:
                s_tx_anim = (uint8_t)((s_tx_anim + 1U) & 0x03U);
                app->need_redraw = true;
                return true;
            default:
                return true;  /* swallow all other events during TX */
        }
    }

    switch (event)
    {
        case SubGhzEventBack:
            if (decode_detail)
            {
                decode_detail = false;
                app->need_redraw = true;
            }
            else
            {
                subghz_scene_pop(app);
            }
            return true;

        case SubGhzEventOk:
            if (decode_detail && decode_count > 0)
            {
                /* Send inline: prepare + start async TX directly from
                 * this scene — no Transmitter scene push. */
                const SubGhzRawDecodeResult *d = &decode_results[decode_sel];
                if (!subghz_protocol_is_static_ext(d->protocol))
                    return true;

                uint16_t te = d->te;
                if (te == 0 && d->protocol < subghz_protocol_registry_count)
                    te = subghz_protocols_list[d->protocol].te_short;

                const char *preset = (s_sig.preset[0] != '\0')
                                     ? s_sig.preset : "FuriHalSubGhzPresetOok650Async";

                if (!flipper_subghz_save_key(DECODE_TMP_PATH,
                        d->frequency, preset,
                        protocol_text[d->protocol],
                        d->bit_len, d->key, te))
                    return true;

                const char *tmp = NULL;
                uint8_t prep = sub_ghz_replay_prepare_flipper(
                    DECODE_TMP_PATH, &tmp);
                if (prep != 0)
                {
                    unlink_tmp();
                    return true;
                }
                if (tmp != NULL)
                {
                    strncpy(s_tx_tmp, tmp, sizeof(s_tx_tmp) - 1);
                    s_tx_tmp[sizeof(s_tx_tmp) - 1] = '\0';
                }

                uint8_t start_ret = sub_ghz_replay_start_async();
                if (start_ret != 0)
                {
                    if (s_tx_tmp[0] != '\0')
                    {
                        (void)f_unlink(s_tx_tmp);
                        s_tx_tmp[0] = '\0';
                    }
                    unlink_tmp();
                    return true;
                }

                s_tx_active = true;
                s_tx_anim   = 0;
                subghz_scene_set_tick_period(app, 150U);
                app->need_redraw = true;
            }
            else if (!decode_detail && decode_count > 0)
            {
                decode_detail = true;
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventDown:
            if (decode_detail && decode_count > 0)
            {
                /* Save decoded signal */
                const SubGhzRawDecodeResult *d = &decode_results[decode_sel];
                char default_name[32];
                snprintf(default_name, sizeof(default_name), "%.12s_%lX",
                         protocol_text[d->protocol],
                         (unsigned long)(uint32_t)d->key);

                char new_name[32];
                if (!m1_vkb_get_filename("Save signal as:", default_name, new_name))
                {
                    app->need_redraw = true;
                    return true;
                }

                if (new_name[0] == '\0')
                {
                    app->need_redraw = true;
                    return true;
                }

                uint16_t te = d->te;
                if (te == 0 && d->protocol < subghz_protocol_registry_count)
                    te = subghz_protocols_list[d->protocol].te_short;

                const char *preset = (s_sig.preset[0] != '\0')
                                     ? s_sig.preset : "FuriHalSubGhzPresetOok650Async";

                uint8_t fmt = subghz_get_save_fmt_ext();
                const char *ext = (fmt == 1) ? ".sgh" : ".sub";
                char save_path[80];
                snprintf(save_path, sizeof(save_path), "/SUBGHZ/%s%s", new_name, ext);

                f_mkdir("/SUBGHZ");
                bool saved;
                if (fmt == 1)
                    saved = flipper_subghz_save_m1native_key(save_path,
                                d->frequency, preset,
                                protocol_text[d->protocol],
                                d->bit_len, d->key, te);
                else
                    saved = flipper_subghz_save_key(save_path,
                                d->frequency, preset,
                                protocol_text[d->protocol],
                                d->bit_len, d->key, te);

                if (saved)
                {
                    /* Navigate to the Saved file browser so the user
                     * lands with the newly saved file visible —
                     * matching Momentum workflow. */
                    m1_led_fast_blink(LED_BLINK_ON_GREEN,
                                     LED_FASTBLINK_PWM_H,
                                     LED_FASTBLINK_ONTIME_H);
                    subghz_scene_push(app, SubGhzSceneSaved);
                }
                else
                {
                    s_save_failed = true;
                    app->need_redraw = true;
                }
            }
            else if (!decode_detail && decode_count > 0)
            {
                if (decode_sel + 1 < decode_count)
                    decode_sel++;
                uint8_t vis = M1_MENU_VIS(decode_count);
                if (decode_sel >= decode_scroll + vis)
                    decode_scroll = (uint8_t)(decode_sel - vis + 1);
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventUp:
            if (!decode_detail && decode_count > 0)
            {
                if (decode_sel > 0)
                    decode_sel--;
                if (decode_sel < decode_scroll)
                    decode_scroll = decode_sel;
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
    (void)app;
    inline_tx_teardown();
    unlink_tmp();
    m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
}

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

static void draw(SubGhzApp *app)
{
    (void)app;
    char line[48];
    const uint8_t item_h   = m1_menu_item_h();
    const uint8_t text_ofs = item_h - 1;

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Title */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 120, "Decode Results", TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, M1_LCD_DISPLAY_WIDTH);

    if (decode_count == 0)
    {
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 10, 32, "No protocols");
        u8g2_DrawStr(&m1_u8g2, 10, 44, "decoded");
        m1_u8g2_nextpage();
        return;
    }

    if (decode_detail)
    {
        /* Detail view — full info for the selected result */
        const SubGhzRawDecodeResult *d = &decode_results[decode_sel];

        if (s_tx_active)
        {
            /* Inline TX overlay — "Sending..." with animation dots */
            static const char *frames[4] = {".  ", ".. ", "...", " .."};
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 22, protocol_text[d->protocol]);
            m1_draw_text(&m1_u8g2, 2, 36, 124, "Sending",
                         TEXT_ALIGN_CENTER);
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            m1_draw_text(&m1_u8g2, 2, 46, 124, frames[s_tx_anim & 0x03U],
                         TEXT_ALIGN_CENTER);
            subghz_button_bar_draw(arrowleft_8x8, "Stop",
                                   NULL, NULL,
                                   NULL, NULL);
            m1_u8g2_nextpage();
            return;
        }

        if (s_save_failed)
        {
            /* Brief "Save failed" overlay — any key will dismiss it */
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
            m1_draw_text(&m1_u8g2, 0, 30, M1_LCD_DISPLAY_WIDTH,
                         "Save failed", TEXT_ALIGN_CENTER);
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            m1_draw_text(&m1_u8g2, 0, 42, M1_LCD_DISPLAY_WIDTH,
                         "Press any key", TEXT_ALIGN_CENTER);
            m1_u8g2_nextpage();
            return;
        }

        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 2, 22, protocol_text[d->protocol]);

        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

        snprintf(line, sizeof(line), "Key: 0x%lX  %dbit",
                 (uint32_t)d->key, d->bit_len);
        u8g2_DrawStr(&m1_u8g2, 2, 30, line);

        /* Show SN and Btn when available — matching Momentum's richer
         * decode detail that exposes these fields for protocols that
         * populate them. */
        uint8_t info_y = 38;
        if (d->serial_number != 0 || d->button_id != 0)
        {
            if (d->serial_number != 0 && d->button_id != 0)
                snprintf(line, sizeof(line), "SN: 0x%lX  Btn: %u",
                         (unsigned long)d->serial_number,
                         (unsigned)d->button_id);
            else if (d->serial_number != 0)
                snprintf(line, sizeof(line), "SN: 0x%lX",
                         (unsigned long)d->serial_number);
            else
                snprintf(line, sizeof(line), "Btn: %u",
                         (unsigned)d->button_id);
            u8g2_DrawStr(&m1_u8g2, 2, info_y, line);
            info_y += 8;
        }

        /* Display TE — use protocol-specified value when available */
        if (info_y <= 38)
        {
            uint16_t te_display = d->te;
            if (te_display == 0 && d->protocol < subghz_protocol_registry_count)
                te_display = subghz_protocols_list[d->protocol].te_short;
            snprintf(line, sizeof(line), "TE: %d us", te_display);
            u8g2_DrawStr(&m1_u8g2, 2, info_y, line);
            info_y += 8;
        }

        /* Integer arithmetic — embedded nano.specs has no %f support */
        snprintf(line, sizeof(line), "Freq: %lu.%02lu MHz",
                 (unsigned long)(d->frequency / 1000000UL),
                 (unsigned long)((d->frequency % 1000000UL) / 10000UL));
        u8g2_DrawStr(&m1_u8g2, 2, info_y, line);

        /* Bottom bar — Send (OK) / Save (DOWN) */
        bool can_send = subghz_protocol_is_static_ext(d->protocol);
        subghz_button_bar_draw(
            arrowdown_8x8, "Save",
            can_send ? ok_circle_8x8 : NULL, can_send ? "Send" : NULL,
            NULL, NULL);
    }
    else
    {
        /* List view */
        u8g2_SetFont(&m1_u8g2, m1_menu_font());
        uint8_t vis = M1_MENU_VIS(decode_count);

        for (uint8_t i = 0; i < vis && (decode_scroll + i) < decode_count; i++)
        {
            uint8_t idx = (uint8_t)(decode_scroll + i);
            const SubGhzRawDecodeResult *d = &decode_results[idx];
            uint8_t y = (uint8_t)(M1_MENU_AREA_TOP + i * item_h);

            if (idx == decode_sel)
            {
                u8g2_DrawRBox(&m1_u8g2, M1_MENU_HIGHLIGHT_X, y,
                              M1_MENU_TEXT_W, item_h, 2);
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            }

            snprintf(line, sizeof(line), "%s 0x%lX",
                     protocol_text[d->protocol], (uint32_t)d->key);
            u8g2_DrawStr(&m1_u8g2, 4, y + text_ofs, line);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        }

        /* Scrollbar */
        if (decode_count > 0)
        {
            uint8_t sb_area_h   = vis * item_h;
            uint8_t sb_handle_h = sb_area_h / decode_count;
            if (sb_handle_h < 6)
                sb_handle_h = 6;
            uint8_t sb_travel_h = (sb_area_h > sb_handle_h)
                                  ? (sb_area_h - sb_handle_h) : 0;
            uint8_t sb_handle_y = M1_MENU_AREA_TOP;
            if (decode_count > 1)
                sb_handle_y += (uint8_t)((uint16_t)sb_travel_h * decode_sel
                               / (decode_count - 1));

            u8g2_DrawVLine(&m1_u8g2,
                           M1_MENU_SCROLLBAR_X + M1_MENU_SCROLLBAR_W / 2,
                           M1_MENU_AREA_TOP, sb_area_h);
            u8g2_DrawRBox(&m1_u8g2,
                          M1_MENU_SCROLLBAR_X, sb_handle_y,
                          M1_MENU_SCROLLBAR_W, sb_handle_h, 1);
        }
    }

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_decode_raw_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
