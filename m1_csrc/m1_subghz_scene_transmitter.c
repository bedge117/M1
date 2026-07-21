/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_transmitter.c
 * @brief  Sub-GHz Transmitter scene — generic key-file replay (Phase 3b-2b-i scaffold).
 *
 * Pushes onto the scene stack from any caller that wants to TX a PACKET /
 * key / Flipper `.sub` file with the canonical async-TX state machine
 * (no blocking wrappers).  The caller fills three input fields in
 * `SubGhzApp` and then calls `subghz_scene_push(app, SubGhzSceneTransmitter)`:
 *
 *   `app->tx_path[]`           Required, absolute path of the file to send.
 *   `app->tx_repeat_count`     Engine tail count (>=1; typical 1 static, 3 rolling).
 *   `app->tx_mode`             0 = SINGLE, 1 = ENDLESS.
 *
 * The scene drives the @ref subghz_transmitter_ctl controller from its
 * `on_event` handler.  Hardware actions (prepare → start_async →
 * continue_async / abort) are performed in response to the controller's
 * returned action, following the async / non-blocking RTOS pattern used
 * throughout the scene engine.
 *
 * State machine (controller-owned):
 *   READY    → OK pressed       → TX_START   → prepare_flipper + start_async
 *   TX       → TxComplete       → TX_NEXT_BURST → continue_async(repeat?)
 *            → TxComplete (last)→ TX_TEARDOWN → unlink temp + abort + park
 *            → BACK             → TX_TEARDOWN → unlink temp + abort + park
 *   EXITING  → (next event)     → EXIT_SCENE → subghz_scene_pop()
 *
 * Notes for callers migrating off the blocking wrappers (Phase 3b-2b-ii+):
 *   - Saved (PACKET path):    set tx_path = saved file, mode=SINGLE,
 *                              repeat_count = file's repeat or 1.
 *   - Playlist:               loop over entries, push Transmitter per
 *                              entry, wait for EXIT_SCENE callback hook
 *                              (added in 3b-2b-iii).
 *   - Remote:                 push Transmitter on button press.
 *   - Bind Wizard:            push Transmitter for each step's TX.
 *
 * This file (Phase 3b-2b-i) only adds the scene scaffold; no callers are
 * migrated yet.  Compiles + registers + boots cleanly; the scene becomes
 * reachable in 3b-2b-ii.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "ff.h"
#include "stm32h5xx_hal.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_lp5814.h"
#include "m1_scene.h"
#include "m1_sub_ghz.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"
#include "subghz_transmitter_ctl.h"
#include "subghz_endless_tx.h"
#include "subghz_button_caps.h"
#include "subghz_button_override.h"
#include "uiView.h"

/*============================================================================*/
/* Scene-local state                                                          */
/*============================================================================*/

/* Scene-local statics are acceptable here because only one Transmitter
 * scene is ever active at a time (single-threaded UI task), and the
 * scene-state slot framework reserves persistence for *cross-scene*
 * state — not in-flight resources like the temp-file path we own from
 * prepare_flipper(). */

/** Controller — owns the READY/TX/EXITING phase + burst accounting. */
static subghz_transmitter_ctl_t s_ctl;

/** Temp file path returned by sub_ghz_replay_prepare_flipper().
 *  Empty string means "no temp file to unlink" (either prepare failed or
 *  it was already unlinked).  The scene owns this — every exit path
 *  (TX completion, BACK abort, scene_on_exit) unlinks if non-empty. */
static char s_tx_unlink_path[80];

/** Snapshot of the user-visible filename for the bottom-line display.
 *  Captured at on_enter so the controller's READY → TX → EXITING
 *  transitions don't have to keep re-deriving it from the path. */
static char s_display_name[32];

/** Last error string (NULL = no error).  Set by the prepare/start error
 *  handlers so draw() can render the message instead of the normal
 *  Send/Cancel screen.  Cleared at on_enter and on every OK_PRESS retry. */
static const char *s_last_error;
static char        s_last_error_buf[32];

/** Animation tick counter — drives the simple "sending" indicator. */
static uint8_t s_anim_phase;

/** Display tick cadence (ms) — see subghz_scene_set_tick_period(). */
#define TX_ANIM_TICK_MS  150U

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

static void capture_display_name(const char *path)
{
    const char *fname = (path != NULL) ? path : "";
    const char *slash = strrchr(fname, '/');
    if (slash != NULL) fname = slash + 1;
    strncpy(s_display_name, fname, sizeof(s_display_name) - 1);
    s_display_name[sizeof(s_display_name) - 1] = '\0';

    /* Strip extension for compactness on the 128×64 display. */
    char *dot = strrchr(s_display_name, '.');
    if (dot != NULL) *dot = '\0';
}

static void unlink_temp_if_any(void)
{
    if (s_tx_unlink_path[0] != '\0')
    {
        (void)f_unlink(s_tx_unlink_path);
        s_tx_unlink_path[0] = '\0';
    }
}

/** Map sub_ghz_replay_prepare_flipper() / start_async() error codes to a
 *  short user-readable string.  Codes match the table in
 *  m1_subghz_scene_read_raw.c::SubGhzEventOk so users see consistent
 *  text across both TX entry points. */
static const char *describe_prepare_error(uint8_t code)
{
    switch (code)
    {
        case 1: return "File/IO error";
        case 2: return "Missing data/frequency";
        case 3: return "Unsupported freq";
        case 4: /* fall through */
        case 5: return "Memory error";
        case 6: return "Dynamic/RAW-only";
        case 7: return "Unsupported protocol";
        default:
            snprintf(s_last_error_buf, sizeof(s_last_error_buf),
                     "Error code: %u", (unsigned)code);
            return s_last_error_buf;
    }
}

/** Translate a controller action into the corresponding hardware action.
 *  Returns true if the scene should redraw. */
static bool perform_action(SubGhzApp *app, subghz_transmitter_action_t action)
{
    switch (action)
    {
        case SUBGHZ_TXCTL_ACT_TX_START:
        {
            /* Prepare the temp file from the source key/PACKET/Flipper file.
             * Note: the Transmitter scene is the canonical entry point for
             * non-RAW files; RAW files go through the Read Raw scene's
             * LoadKeyTX state instead.
             *
             * Phase 4c: when the protocol supports button override, set
             * the per-prepare slot before invoking the converter.  The
             * converter latches and clears the slot internally, so the
             * call is self-contained — no need to clear afterwards.
             * For protocols without override support, the controller
             * keeps button_index at 0 and the override is a no-op. */
            if (s_ctl.allow_button_cycle)
            {
                sub_ghz_replay_set_button_override(
                    (int8_t)s_ctl.button_index);
            }
            const char *tmp = NULL;
            uint8_t prep_ret = sub_ghz_replay_prepare_flipper(
                app->tx_path, &tmp);
            if (prep_ret != 0)
            {
                /* Prepare failed before any hardware was touched.
                 * Stash the error string; draw() will render it and the
                 * user can press BACK to exit.  Subsequent OK / TX
                 * events are gated in scene_on_event() while
                 * s_last_error is set. */
                s_last_error = describe_prepare_error(prep_ret);
                return true;
            }
            if (tmp != NULL)
            {
                strncpy(s_tx_unlink_path, tmp,
                        sizeof(s_tx_unlink_path) - 1);
                s_tx_unlink_path[sizeof(s_tx_unlink_path) - 1] = '\0';
            }

            /* Arm the radio.  start_async() returns immediately; further
             * progress arrives as Q_EVENT_SUBGHZ_TX → SubGhzEventTxComplete.
             *
             * On failure, start_async() has already torn down internal
             * state and powered off the radio (see m1_sub_ghz.h docs).
             * Clean up our scene-local temp file and surface the error.
             * No teardown event sent to the controller — the next BACK
             * keypress will trigger the controller's BACK→TX_TEARDOWN
             * which is a no-op against an already-quiesced engine. */
            uint8_t start_ret = sub_ghz_replay_start_async();
            if (start_ret != 0)
            {
                unlink_temp_if_any();
                s_last_error = (start_ret == 4 || start_ret == 5)
                                   ? "Memory error"
                                   : describe_prepare_error(start_ret);
            }
            return true;
        }

        case SUBGHZ_TXCTL_ACT_TX_NEXT_BURST:
        {
            /* Engine wants another burst.  In SINGLE mode this fires
             * the tail-count bursts; in ENDLESS mode it loops until
             * OK is released.  Always pass repeat_on_idle=true here —
             * the controller drives mode-specific stopping by either
             * NOT calling us (SINGLE finished, returns TX_TEARDOWN
             * directly) or by feeding OK_RELEASE first (ENDLESS).
             *
             * Note: in this scaffold we don't yet wire OK_RELEASE; the
             * controller's SINGLE-mode tail-count accounting still
             * works correctly and the scene completes as expected. */
            (void)sub_ghz_replay_continue_async(true);
            return false;  /* anim handled on tick */
        }

        case SUBGHZ_TXCTL_ACT_TX_TEARDOWN:
        {
            sub_ghz_replay_abort();
            unlink_temp_if_any();
            /* Acknowledge teardown — the controller transitions to
             * EXITING; the very next event returns EXIT_SCENE. */
            subghz_transmitter_action_t next =
                subghz_transmitter_ctl_event(
                    &s_ctl, SUBGHZ_TXCTL_EVT_TEARDOWN_DONE);
            return perform_action(app, next);
        }

        case SUBGHZ_TXCTL_ACT_EXIT_SCENE:
            subghz_scene_pop(app);
            return false;  /* scene is gone; no redraw */

        case SUBGHZ_TXCTL_ACT_CYCLE_BUTTON_PREV:
        case SUBGHZ_TXCTL_ACT_CYCLE_BUTTON_NEXT:
            /* Phase 4c — the controller has already mutated
             * `s_ctl.button_index` to the new value.  The TX_START
             * branch above will pass it to
             * sub_ghz_replay_set_button_override() on the next OK
             * press, and the converter will mutate the parsed
             * key_value to encode the requested button before
             * encoding the OOK PWM waveform.  Nothing to do here
             * except request a redraw so the "Btn X/Y" indicator
             * reflects the new value. */
            return true;

        case SUBGHZ_TXCTL_ACT_NONE:
        default:
            return false;
    }
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Reset all scene-local state. */
    s_tx_unlink_path[0] = '\0';
    s_last_error        = NULL;
    s_anim_phase        = 0;

    capture_display_name(app->tx_path);

    /* Clamp inputs to safe ranges.  Controller also clamps repeat_count
     * internally; we mirror that here so the display value is consistent. */
    uint16_t repeat = (app->tx_repeat_count == 0U) ? 1U : app->tx_repeat_count;
    subghz_endless_tx_mode_t mode =
        (app->tx_mode == 1U) ? SUBGHZ_TX_MODE_ENDLESS
                              : SUBGHZ_TX_MODE_SINGLE;

    /* Init controller.  Phase 4b: query the button-cycling capability
     * helper from the protocol name supplied by the caller.  Phase 4c:
     * AND-gate the cycling enable against subghz_button_override_supports()
     * so the UI only offers cycling for protocols where pressing
     * LEFT/RIGHT actually mutates the transmitted key.  Protocols whose
     * button field is encoded via opaque manufacturer lookup tables
     * (Nice FloR-S, FAAC SLH) or embedded inside the encrypted rolling
     * payload (CAME Atomo, Alutech AT-4N, KingGates Stylo4k, DITEC GOL4,
     * Toyota, Scher-Khan) are listed in `subghz_button_caps` for future
     * use but do NOT yet have a working per-button re-encoder — those
     * fall back to a single-button non-cycling display here.
     *
     * When tx_protocol_name is empty or unknown the helper returns
     * {false, 0} which leaves LEFT/RIGHT inert — matching the legacy
     * single-button behaviour for static OOK files and for callers
     * (Playlist / Remote) where cycling is not part of the UX. */
    subghz_button_caps_t caps =
        subghz_button_caps_for_protocol(app->tx_protocol_name);
    bool override_ok =
        subghz_button_override_supports(app->tx_protocol_name);
    bool cycling_enabled = caps.supports_cycling && override_ok;
    subghz_transmitter_ctl_init(&s_ctl, mode, repeat,
                                cycling_enabled,
                                cycling_enabled ? caps.button_count : 1U);

    /* Cadence for animation + auto-dismiss after error. */
    subghz_scene_set_tick_period(app, TX_ANIM_TICK_MS);

    /* Default to "natural completion".  The BACK event handler overrides
     * this to false before initiating teardown so parents can detect
     * user-initiated aborts on pop-back. */
    app->tx_completed_naturally = true;

    app->need_redraw = true;

    /* Auto-start hint (Phase 3b-2b-iv) — synthesize an OK_PRESS so TX
     * begins immediately without the READY → press-OK confirmation
     * step.  Used by the Remote scene where pressing a mapped button
     * must fire its signal in one press.  Cleared after consumption
     * so any later pop-back into a new push has to opt in again. */
    if (app->tx_autostart && app->tx_path[0] != '\0')
    {
        app->tx_autostart = false;
        subghz_transmitter_action_t a =
            subghz_transmitter_ctl_event(&s_ctl,
                                          SUBGHZ_TXCTL_EVT_OK_PRESS);
        (void)perform_action(app, a);
    }
    else
    {
        /* Defensive: ensure the flag is clear even on non-autostart
         * pushes so a stale "true" from an earlier caller can't leak. */
        app->tx_autostart = false;
    }
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* If an error is currently displayed the controller may be in TX
     * phase with no hardware armed (prepare/start failed after
     * OK_PRESS already advanced READY→TX).  Any BACK/OK exits the
     * scene; other events are ignored.  BACK is routed through the
     * controller so it drives the EXITING transition correctly
     * (TX_TEARDOWN against the already-quiesced engine is a no-op
     * inside abort/unlink_temp_if_any). */
    if (s_last_error != NULL)
    {
        if (event == SubGhzEventBack || event == SubGhzEventOk)
        {
            s_last_error = NULL;
            /* Treat dismissing an error screen as a user-initiated exit
             * so a parent (e.g. Playlist) sees this as an abort and
             * stops the sequence rather than advancing. */
            app->tx_completed_naturally = false;
            subghz_transmitter_action_t a =
                subghz_transmitter_ctl_event(&s_ctl,
                                              SUBGHZ_TXCTL_EVT_BACK);
            if (perform_action(app, a)) app->need_redraw = true;
            return true;
        }
        return false;
    }

    switch (event)
    {
        case SubGhzEventOk:
        {
            /* Empty path — show error and let user back out. */
            if (app->tx_path[0] == '\0')
            {
                s_last_error = "No file loaded";
                app->need_redraw = true;
                return true;
            }
            s_last_error = NULL;
            subghz_transmitter_action_t a =
                subghz_transmitter_ctl_event(&s_ctl,
                                              SUBGHZ_TXCTL_EVT_OK_PRESS);
            if (perform_action(app, a)) app->need_redraw = true;
            return true;
        }

        case SubGhzEventTxComplete:
        {
            subghz_transmitter_action_t a =
                subghz_transmitter_ctl_event(
                    &s_ctl, SUBGHZ_TXCTL_EVT_TX_BURST_COMPLETE);
            if (perform_action(app, a)) app->need_redraw = true;
            return true;
        }

        case SubGhzEventBack:
        {
            /* User-initiated exit (either abort during TX or back from
             * READY).  Signal "not natural" so parent scenes know to
             * stop a sequence rather than advance to next item. */
            app->tx_completed_naturally = false;
            subghz_transmitter_action_t a =
                subghz_transmitter_ctl_event(&s_ctl,
                                              SUBGHZ_TXCTL_EVT_BACK);
            if (perform_action(app, a)) app->need_redraw = true;
            return true;
        }

        case SubGhzEventLeft:
        {
            subghz_transmitter_action_t a =
                subghz_transmitter_ctl_event(&s_ctl,
                                              SUBGHZ_TXCTL_EVT_LEFT);
            if (perform_action(app, a)) app->need_redraw = true;
            return true;
        }

        case SubGhzEventRight:
        {
            subghz_transmitter_action_t a =
                subghz_transmitter_ctl_event(&s_ctl,
                                              SUBGHZ_TXCTL_EVT_RIGHT);
            if (perform_action(app, a)) app->need_redraw = true;
            return true;
        }

        case SubGhzEventTick:
            /* Advance the simple "..." animation while TX is in flight.
             * No redraw needed in READY — display is static there. */
            if (subghz_transmitter_ctl_is_tx(&s_ctl))
            {
                s_anim_phase = (uint8_t)((s_anim_phase + 1U) & 0x03U);
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
    /* Defensive: if we're somehow still mid-TX (e.g. forced scene
     * teardown from the main loop), abort and unlink the temp file. */
    if (sub_ghz_replay_async_is_active())
    {
        sub_ghz_replay_abort();
    }
    unlink_temp_if_any();
    /* Phase 4c: clear any pending button-override slot.  The converter
     * also resets on entry, but clearing here prevents a stale set
     * (e.g. user pressed RIGHT then BACK without OK) from leaking
     * into an unrelated subsequent prepare from any other caller. */
    sub_ghz_replay_set_button_override(-1);
    /* Controller / tick cadence are reset automatically by the scene
     * manager on pop (Phase 3b-2a invariant). */
}

/*============================================================================*/
/* Drawing                                                                    */
/*============================================================================*/

static void draw_dots(uint8_t phase)
{
    /* Three dots, one of which is bold per phase 0..3 (4-step cycle). */
    static const char *frames[4] = {".  ", ".. ", "...", " ..", };
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 42, 124, frames[phase & 0x03U],
                 TEXT_ALIGN_CENTER);
}

static void draw(SubGhzApp *app)
{
    (void)app;
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Title row */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 124, "Transmit", TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);

    if (s_last_error != NULL)
    {
        /* Error layout: filename, then error string in the body. */
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        m1_draw_text(&m1_u8g2, 2, 24, 124, s_display_name,
                     TEXT_ALIGN_CENTER);
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        m1_draw_text(&m1_u8g2, 2, 40, 124, "Send failed",
                     TEXT_ALIGN_CENTER);
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        m1_draw_text(&m1_u8g2, 2, 50, 124, s_last_error,
                     TEXT_ALIGN_CENTER);
        subghz_button_bar_draw(arrowleft_8x8, "Back",
                               NULL, NULL,
                               NULL, NULL);
        m1_u8g2_nextpage();
        return;
    }

    if (subghz_transmitter_ctl_is_tx(&s_ctl))
    {
        /* TX in flight: filename + dots animation + burst counter. */
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        m1_draw_text(&m1_u8g2, 2, 24, 124, s_display_name,
                     TEXT_ALIGN_CENTER);

        draw_dots(s_anim_phase);

        char counter[24];
        uint16_t emitted = subghz_transmitter_ctl_bursts_emitted(&s_ctl);
        if (s_ctl.allow_button_cycle && s_ctl.button_count > 1U)
        {
            snprintf(counter, sizeof(counter), "Burst %u  Btn %u/%u",
                     (unsigned)emitted,
                     (unsigned)(s_ctl.button_index + 1U),
                     (unsigned)s_ctl.button_count);
        }
        else
        {
            snprintf(counter, sizeof(counter), "Burst %u", (unsigned)emitted);
        }
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        m1_draw_text(&m1_u8g2, 2, 51, 124, counter, TEXT_ALIGN_CENTER);

        subghz_button_bar_draw(arrowleft_8x8, "Stop",
                               NULL, NULL,
                               NULL, NULL);
    }
    else
    {
        /* READY: filename + Send / Back hint.  When the loaded protocol
         * supports button cycling (Phase 4b), render the current
         * "Btn X/Y" indicator on the body line and surface LEFT/RIGHT
         * as button-cycle hints; otherwise show the static
         * "Press OK to send" copy. */
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        m1_draw_text(&m1_u8g2, 2, 30, 124, s_display_name,
                     TEXT_ALIGN_CENTER);

        if (s_ctl.allow_button_cycle && s_ctl.button_count > 1U)
        {
            char btn_line[16];
            snprintf(btn_line, sizeof(btn_line), "Btn %u/%u",
                     (unsigned)(s_ctl.button_index + 1U),
                     (unsigned)s_ctl.button_count);
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            m1_draw_text(&m1_u8g2, 2, 44, 124, btn_line,
                         TEXT_ALIGN_CENTER);

            /* LEFT/RIGHT cycle the button index; OK sends. */
            subghz_button_bar_draw(arrowleft_8x8, NULL,
                                   ok_circle_8x8, "Send",
                                   NULL, arrowright_8x8);
        }
        else
        {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            m1_draw_text(&m1_u8g2, 2, 44, 124, "Press OK to send",
                         TEXT_ALIGN_CENTER);

            subghz_button_bar_draw(arrowleft_8x8, "Back",
                                   ok_circle_8x8, "Send",
                                   NULL, NULL);
        }
    }

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_transmitter_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
