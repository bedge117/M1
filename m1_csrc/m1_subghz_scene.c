/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene.c
 * @brief  Sub-GHz Scene Manager — stack-based scene dispatch engine.
 *
 * Manages a scene stack where BACK always pops to the previous scene.
 * Each scene registers on_enter/on_event/on_exit/draw callbacks via
 * the SubGhzSceneHandlers table.  The main loop translates FreeRTOS
 * queue events into SubGhzEvent values and dispatches to the active scene.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_tasks.h"
#include "m1_system.h"
#include "m1_settings.h"   /* m1_orient_enter_landscape/restore for per-scene orientation */
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_api.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_subghz_scene.h"
#include "subghz_scene_polish.h"
#include "m1_esp32_hal.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_lp5814.h"
#include "m1_buzzer.h"
#include "uiView.h"

#define M1_LOGDB_TAG  "SubGhzScene"

/* Shared RX state (defined in m1_sub_ghz.c / m1_sub_ghz_decenc.c) */
extern SubGHz_DecEnc_t subghz_decenc_ctl;
extern uint8_t subghz_record_mode_flag;

/* Persisted radio config accessors (defined in m1_sub_ghz.c) */
extern uint8_t subghz_get_freq_idx_ext(void);
extern uint8_t subghz_get_mod_idx_ext(void);
extern bool    subghz_get_hopping_ext(void);
extern bool    subghz_get_sound_ext(void);
extern bool    subghz_get_autosave_ext(void);
extern uint8_t subghz_get_tx_power_idx_ext(void);

/*============================================================================*/
/* Scene handler registry — indexed by SubGhzSceneId                          */
/*============================================================================*/

static const SubGhzSceneHandlers *scene_registry[SubGhzSceneCount] = {
    [SubGhzSceneMenu]             = &subghz_scene_menu_handlers,
    [SubGhzSceneRead]             = &subghz_scene_read_handlers,
    [SubGhzSceneReadRaw]          = &subghz_scene_read_raw_handlers,
    [SubGhzSceneReceiverInfo]     = &subghz_scene_receiver_info_handlers,
    [SubGhzSceneConfig]           = &subghz_scene_config_handlers,
    [SubGhzSceneSaveName]         = &subghz_scene_save_name_handlers,
    [SubGhzSceneSaveSuccess]      = &subghz_scene_save_success_handlers,
    [SubGhzSceneNeedSaving]       = &subghz_scene_need_saving_handlers,
    [SubGhzSceneSaved]            = &subghz_scene_saved_handlers,
    [SubGhzSceneFreqAnalyzer]     = &subghz_scene_freq_analyzer_handlers,
    [SubGhzScenePlaylist]         = &subghz_scene_playlist_handlers,
    [SubGhzSceneSpectrumAnalyzer] = &subghz_scene_spectrum_analyzer_handlers,
    [SubGhzSceneRssiMeter]        = &subghz_scene_rssi_meter_handlers,
    [SubGhzSceneFreqScanner]      = &subghz_scene_freq_scanner_handlers,
    [SubGhzSceneWeatherStation]   = &subghz_scene_weather_station_handlers,
    [SubGhzSceneBruteForce]       = &subghz_scene_brute_force_handlers,
    [SubGhzSceneRemote]           = &subghz_scene_remote_handlers,
    [SubGhzSceneBindWizard]       = &subghz_scene_bind_wizard_handlers,
    [SubGhzSceneTransmitter]      = &subghz_scene_transmitter_handlers,
    [SubGhzSceneSavedMenu]        = &subghz_scene_saved_menu_handlers,
    [SubGhzSceneDelete]           = &subghz_scene_delete_handlers,
    [SubGhzSceneMoreRaw]          = &subghz_scene_more_raw_handlers,
    [SubGhzSceneDecodeRaw]        = &subghz_scene_decode_raw_handlers,
    [SubGhzSceneSetType]          = &subghz_scene_set_type_handlers,
    [SubGhzSceneSetKey]           = &subghz_scene_set_key_handlers,
    [SubGhzSceneSetSerial]        = &subghz_scene_set_serial_handlers,
    [SubGhzSceneSetButton]        = &subghz_scene_set_button_handlers,
    [SubGhzSceneSetCounter]       = &subghz_scene_set_counter_handlers,
    [SubGhzSceneSetMfKey]         = &subghz_scene_set_mfkey_handlers,
    [SubGhzSceneSignalSettings]   = &subghz_scene_signal_settings_handlers,
    [SubGhzSceneProtoPirateMenu]  = &subghz_scene_proto_pirate_menu_handlers,
    [SubGhzSceneProtoPirateTuner] = &subghz_scene_proto_pirate_tuner_handlers,
};

/*============================================================================*/
/* Scene Manager API                                                          */
/*============================================================================*/

void subghz_scene_init(SubGhzApp *app)
{
    memset(app, 0, sizeof(*app));

    /* Per-scene state slots — explicit init for clarity (memset above
     * already zeroes them, but the contract is documented as init()). */
    subghz_scene_state_init(&app->scene_state);

    /* Load persisted radio config (set at boot from settings.cfg,
     * defaults to compile-time values if settings file is absent) */
    app->freq_idx     = subghz_get_freq_idx_ext();
    app->mod_idx      = subghz_get_mod_idx_ext();
    app->hopping      = subghz_get_hopping_ext();
    app->sound        = subghz_get_sound_ext();
    app->autosave     = subghz_get_autosave_ext();
    app->tx_power_idx = subghz_get_tx_power_idx_ext();

    app->running      = true;
    app->scene_depth  = 0;
}

static const SubGhzSceneHandlers *get_handlers(SubGhzSceneId id)
{
    if (id < SubGhzSceneCount && scene_registry[id])
        return scene_registry[id];
    return NULL;
}

SubGhzSceneId subghz_scene_current(const SubGhzApp *app)
{
    if (app->scene_depth == 0)
        return SubGhzSceneMenu;  /* fallback */
    return app->scene_stack[app->scene_depth - 1];
}

void subghz_scene_set_state(SubGhzApp *app, SubGhzSceneId scene, uint32_t value)
{
    if (app == NULL) return;
    (void)subghz_scene_state_set(&app->scene_state, (uint8_t)scene, value);
}

uint32_t subghz_scene_get_state(const SubGhzApp *app, SubGhzSceneId scene)
{
    if (app == NULL) return 0;
    return subghz_scene_state_get(&app->scene_state, (uint8_t)scene);
}

/*============================================================================*/
/* Polish API (Phase 3b-2a — Momentum parity)                                 */
/*============================================================================*/

/**
 * @brief  Reset per-scene tick state.  Called from every transition helper
 *         (push / pop / replace / search_and_pop_to) so a freshly-active
 *         scene starts with ticks disabled and its first tick will fire a
 *         full `tick_period_ms` after it opts in via `set_tick_period`.
 */
static void reset_tick_state(SubGhzApp *app)
{
    app->tick_period_ms = 0;
    app->last_tick_ms   = HAL_GetTick();
}

bool subghz_scene_search_and_pop_to(SubGhzApp *app, SubGhzSceneId target)
{
    if (app == NULL || app->scene_depth == 0) return false;

    /* SubGhzSceneId is an enum (compiler-chosen storage, typically int), so
     * the scene stack cannot be aliased as `uint8_t *` — that would scan the
     * raw bytes of the first few enum entries instead of one slot per entry.
     * Copy the slots into a byte buffer for the pure-logic helper, which
     * operates on uint8_t arrays. */
    uint8_t bytes[SUBGHZ_SCENE_STACK_MAX];
    for (uint8_t i = 0; i < app->scene_depth; ++i)
        bytes[i] = (uint8_t)app->scene_stack[i];

    int idx = subghz_scene_stack_find(bytes, app->scene_depth, (uint8_t)target);
    if (idx < 0) return false;  /* not found */

    uint8_t new_depth = (uint8_t)(idx + 1);

    /* Target already on top — nothing to pop or re-enter.  Reported as
     * success so callers do not fall back to subghz_scene_pop() and
     * accidentally pop the target itself. */
    if (new_depth == app->scene_depth) return true;

    /* Pop scenes one at a time, calling on_exit for each, until target on top.
     * We do NOT call on_enter on intermediate parents — they are skipped over,
     * not re-entered.  The final on_enter on `target` reactivates the scene. */
    while (app->scene_depth > new_depth)
    {
        const SubGhzSceneHandlers *cur = get_handlers(subghz_scene_current(app));
        if (cur && cur->on_exit)
            cur->on_exit(app);
        app->scene_depth--;
    }

    /* Reset tick cadence BEFORE on_enter so target scenes that opt into
     * ticks during entry (via subghz_scene_set_tick_period) keep their rate. */
    reset_tick_state(app);

    const SubGhzSceneHandlers *next = get_handlers(target);
    if (next && next->on_enter)
        next->on_enter(app);

    app->need_redraw = true;
    return true;
}

bool subghz_scene_send_custom_event(SubGhzApp *app,
                                     subghz_scene_custom_payload_t payload)
{
    if (app == NULL) return false;
    app->custom_event_payload = payload;
    return subghz_scene_send_event(app, SubGhzEventCustom);
}

subghz_scene_custom_payload_t subghz_scene_custom_payload(const SubGhzApp *app)
{
    if (app == NULL) return 0;
    return app->custom_event_payload;
}

void subghz_scene_set_tick_period(SubGhzApp *app, uint32_t period_ms)
{
    if (app == NULL) return;
    app->tick_period_ms = period_ms;
    app->last_tick_ms   = HAL_GetTick();
}

void subghz_scene_push(SubGhzApp *app, SubGhzSceneId scene)
{
    /* Exit current scene if any */
    if (app->scene_depth > 0)
    {
        const SubGhzSceneHandlers *cur = get_handlers(subghz_scene_current(app));
        if (cur && cur->on_exit)
            cur->on_exit(app);
    }

    /* Push new scene */
    if (app->scene_depth < SUBGHZ_SCENE_STACK_MAX)
    {
        app->scene_stack[app->scene_depth++] = scene;
    }

    /* Reset tick cadence BEFORE on_enter so the child can opt back in via
     * subghz_scene_set_tick_period() during its own entry handler. */
    reset_tick_state(app);

    /* Enter new scene */
    const SubGhzSceneHandlers *next = get_handlers(scene);
    if (next && next->on_enter)
        next->on_enter(app);

    app->need_redraw = true;
}

void subghz_scene_pop(SubGhzApp *app)
{
    if (app->scene_depth == 0)
    {
        app->running = false;
        return;
    }

    /* Exit current scene */
    const SubGhzSceneHandlers *cur = get_handlers(subghz_scene_current(app));
    if (cur && cur->on_exit)
        cur->on_exit(app);

    app->scene_depth--;

    if (app->scene_depth == 0)
    {
        app->running = false;
        return;
    }

    /* Reset tick cadence BEFORE on_enter so the resumed parent can opt back
     * in via subghz_scene_set_tick_period() during its own entry handler. */
    reset_tick_state(app);

    /* Re-enter the scene that's now on top */
    const SubGhzSceneHandlers *prev = get_handlers(subghz_scene_current(app));
    if (prev && prev->on_enter)
        prev->on_enter(app);

    app->need_redraw = true;
}

void subghz_scene_replace(SubGhzApp *app, SubGhzSceneId scene)
{
    /* Exit current scene */
    if (app->scene_depth > 0)
    {
        const SubGhzSceneHandlers *cur = get_handlers(subghz_scene_current(app));
        if (cur && cur->on_exit)
            cur->on_exit(app);
        app->scene_stack[app->scene_depth - 1] = scene;
    }
    else
    {
        app->scene_stack[app->scene_depth++] = scene;
    }

    /* Reset tick cadence BEFORE on_enter — replacement is a fresh scene
     * and must be able to opt into ticks during its entry handler. */
    reset_tick_state(app);

    /* Enter replacement scene */
    const SubGhzSceneHandlers *next = get_handlers(scene);
    if (next && next->on_enter)
        next->on_enter(app);

    app->need_redraw = true;
}

bool subghz_scene_send_event(SubGhzApp *app, SubGhzEvent event)
{
    const SubGhzSceneHandlers *cur = get_handlers(subghz_scene_current(app));
    if (cur && cur->on_event)
        return cur->on_event(app, event);
    return false;
}

void subghz_scene_draw(SubGhzApp *app)
{
    const SubGhzSceneHandlers *cur = get_handlers(subghz_scene_current(app));
    if (cur && cur->draw)
        cur->draw(app);
}

/*============================================================================*/
/* Main event loop                                                            */
/*============================================================================*/

/**
 * @brief  Translate a FreeRTOS button event into SubGhzEvent.
 */
static SubGhzEvent translate_button(void)
{
    S_M1_Buttons_Status btn;
    BaseType_t ret = xQueueReceive(button_events_q_hdl, &btn, 0);
    if (ret != pdTRUE)
        return SubGhzEventNone;

    if (btn.event[BUTTON_OK_KP_ID]    == BUTTON_EVENT_CLICK) return SubGhzEventOk;
    if (btn.event[BUTTON_BACK_KP_ID]  == BUTTON_EVENT_CLICK) return SubGhzEventBack;
    if (btn.event[BUTTON_LEFT_KP_ID]  == BUTTON_EVENT_CLICK) return SubGhzEventLeft;
    if (btn.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK) return SubGhzEventRight;
    if (btn.event[BUTTON_UP_KP_ID]    == BUTTON_EVENT_CLICK) return SubGhzEventUp;
    if (btn.event[BUTTON_DOWN_KP_ID]  == BUTTON_EVENT_CLICK) return SubGhzEventDown;

    return SubGhzEventNone;
}

/* Which Sub-GHz scenes are bespoke landscape-only activities (analyzers, read,
 * transmit, meters). Everything else (menus, setters, confirms) stays portrait. */
static bool subghz_scene_is_landscape(SubGhzSceneId id)
{
    switch (id)
    {
    case SubGhzSceneRead:
    case SubGhzSceneReadRaw:
    case SubGhzSceneReceiverInfo:
    case SubGhzSceneFreqAnalyzer:
    case SubGhzSceneSpectrumAnalyzer:
    case SubGhzSceneRssiMeter:
    case SubGhzSceneFreqScanner:
    case SubGhzSceneWeatherStation:
    case SubGhzSceneBruteForce:
    case SubGhzSceneRemote:
    case SubGhzSceneTransmitter:
    case SubGhzSceneDecodeRaw:
    case SubGhzSceneProtoPirateTuner:
        return true;
    default:
        return false;
    }
}

/* Follow the active scene's orientation. Idempotent — safe to call every frame;
 * it only flips on an actual change. Portrait-only (guarded inside the helpers). */
static bool s_subghz_forced_landscape = false;
static void subghz_apply_scene_orient(SubGhzSceneId id)
{
    bool want = subghz_scene_is_landscape(id);
    if (want && !s_subghz_forced_landscape)
    {
        m1_orient_enter_landscape();
        s_subghz_forced_landscape = true;
    }
    else if (!want && s_subghz_forced_landscape)
    {
        m1_orient_restore();
        s_subghz_forced_landscape = false;
    }
}

void subghz_scene_app_run(void)
{
    /* SubGhzApp is 5+ KB — far larger than the menu task's 4 KB stack.
     * Declare it static so it lives in BSS instead of on the stack.
     * subghz_scene_init() calls memset(app, 0, sizeof(*app)) on every
     * entry, so stale state from a previous session is never a concern. */
    static SubGhzApp app;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;

    /* Initialize scene manager and app context */
    subghz_scene_init(&app);
    s_subghz_forced_landscape = false;   /* start on the (portrait) menu scene */

    /* Sub-GHz is timing-sensitive and does not use the ESP32.  If a prior
     * WiFi/BLE tool left the ESP32 SPI/EXTI transport active, tear it down
     * before arming the SI4463/TIM1 capture path. */
    m1_esp32_deinit();

    /* Initialize radio hardware */
    menu_sub_ghz_init();

    /* Initialize protocol decoder function pointers and pulse tables */
    subghz_decenc_init();

    /* Push initial scene */
    subghz_scene_push(&app, SubGhzSceneMenu);

    /* Draw initial screen */
    subghz_scene_draw(&app);
    app.need_redraw = false;

    /* Time-based hopper tick: tracks wall-clock time so the hopper advances
     * every 200 ms even when the queue is saturated with Q_EVENT_SUBGHZ_RX
     * noise pulses.  At 315/433 MHz the RF noise floor generates thousands of
     * edges per second, keeping xQueueReceive() returning pdTRUE continuously
     * and preventing the queue-timeout path from ever firing.
     *
     * prev_hopper_active tracks the last iteration's state so we can detect
     * the false→true activation edge and reset the dwell timer, preventing
     * an immediate first tick when hopping is enabled after a period of
     * portMAX_DELAY blocking (during which the else branch never ran). */
    uint32_t last_hop_tick_ms = HAL_GetTick();
    bool     prev_hopper_active = false;

    /* Main event loop */
    while (app.running)
    {
        /* Determine if active RX needs periodic refresh */
        bool rx_active = false;
        SubGhzSceneId cur_scene = subghz_scene_current(&app);
        subghz_apply_scene_orient(cur_scene);   /* landscape for activity scenes, portrait for menus */
        if (cur_scene == SubGhzSceneRead && app.read_state == SubGhzReadStateRx)
            rx_active = true;
        else if (cur_scene == SubGhzSceneReadRaw &&
                 (app.raw_state == SubGhzReadRawStateRecording ||
                  app.raw_state == SubGhzReadRawStateStart ||
                  subghz_read_raw_state_is_tx(app.raw_state)))
            rx_active = true;  /* periodic refresh: RSSI bar in Recording; signal-detection poll in Start; sine-wave animation in TX states */

        /* Wait for event: use 200ms poll during active RX for RSSI refresh.
         * If a scene has opted into periodic ticks via set_tick_period(),
         * cap the wait so we can dispatch SubGhzEventTick on cadence. */
        uint32_t wait_ms = 0;  /* 0 = portMAX_DELAY (no upper bound) */
        if (app.hopper_active || rx_active)
            wait_ms = 200U;
        if (app.tick_period_ms > 0U &&
            (wait_ms == 0U || app.tick_period_ms < wait_ms))
            wait_ms = app.tick_period_ms;

        TickType_t wait = (wait_ms == 0U) ?
            portMAX_DELAY : pdMS_TO_TICKS(wait_ms);

        ret = xQueueReceive(main_q_hdl, &q_item, wait);

        if (ret != pdTRUE)
        {
            /* Periodic display refresh during active RX (RSSI update) */
            if (rx_active)
                app.need_redraw = true;
        }
        else
        {
            SubGhzEvent evt = SubGhzEventNone;

            switch (q_item.q_evt_type)
            {
                case Q_EVENT_KEYPAD:
                    evt = translate_button();
                    break;
                case Q_EVENT_SUBGHZ_RX:
                    if (subghz_record_mode_flag)
                    {
                        /* Raw recording mode — pass through to scene
                         * (ring buffer data handled by Read Raw scene) */
                        evt = SubGhzEventRxData;
                    }
                    else if (subghz_decenc_ctl.subghz_pulse_handler)
                    {
                        /* Protocol decode mode — batch-drain all pending
                         * pulse events in a tight loop.  The ISR queues
                         * every individual edge as a separate event; at
                         * 433 MHz with AM650 the noise floor alone can
                         * generate thousands of edges/sec.  Processing
                         * only one pulse per main-loop iteration cannot
                         * keep up, the 256-deep queue overflows, and real
                         * signal pulses are silently dropped — so nothing
                         * ever decodes.  Draining in batch keeps the queue
                         * from overflowing and feeds complete packets to
                         * the protocol decoders. */
                        for (;;)
                        {
                            S_M1_Main_Q_t peek;
                            uint16_t dur = q_item.q_data.ir_rx_data.ir_edge_te;
                            uint8_t pdet = subghz_decenc_ctl.subghz_pulse_handler(dur);

                            /* Sync ISR state machine with pulse handler
                             * output.  Without this, the ISR stays in
                             * NORMAL forever and floods the queue with
                             * noise pulses between packets. */
                            if (pdet == PULSE_DET_EOP || pdet == PULSE_DET_IDLE)
                                subghz_decenc_ctl.pulse_det_stat = pdet;

                            /* Decode ready? Stop draining — let the scene
                             * handle the result before processing more. */
                            if (subghz_decenc_ctl.subghz_data_ready &&
                                subghz_decenc_ctl.subghz_data_ready())
                            {
                                evt = SubGhzEventRxData;
                                break;
                            }

                            /* Peek at next queue item; continue only if
                             * it is another pulse event.  Non-pulse events
                             * (keypad, TX-complete) stay in the queue for
                             * the outer loop to handle. */
                            if (xQueuePeek(main_q_hdl, &peek, 0) != pdTRUE)
                                break;          /* queue empty */
                            if (peek.q_evt_type != Q_EVENT_SUBGHZ_RX)
                                break;          /* next event is not a pulse */
                            xQueueReceive(main_q_hdl, &q_item, 0);
                        }
                    }
                    break;
                case Q_EVENT_SUBGHZ_TX:
                    evt = SubGhzEventTxComplete;
                    break;
                default:
                    break;
            }

            if (evt != SubGhzEventNone)
                subghz_scene_send_event(&app, evt);
        }

        /* Time-based hopper tick: fire every 200 ms of real wall-clock time.
         * This runs after every queue receive (or timeout), so the hopper
         * advances correctly even when the queue is saturated with noise.
         * On the false→true activation edge the timer is reset to now so
         * the first hop always waits a full 200 ms dwell. */
        if (app.hopper_active)
        {
            uint32_t now = HAL_GetTick();
            if (!prev_hopper_active)
            {
                /* Fresh activation edge — start the dwell interval from now */
                last_hop_tick_ms = now;
            }
            else if ((now - last_hop_tick_ms) >= 200U)
            {
                last_hop_tick_ms = now;
                subghz_scene_send_event(&app, SubGhzEventHopperTick);
            }
        }
        prev_hopper_active = app.hopper_active;

        /* Dispatch SubGhzEventTick on cadence.  Pure-logic helper handles
         * uint32_t wraparound at the 49.7-day HAL_GetTick rollover. */
        if (app.tick_period_ms > 0U)
        {
            uint32_t now = HAL_GetTick();
            if (subghz_scene_tick_due(now, app.last_tick_ms, app.tick_period_ms))
            {
                app.last_tick_ms = now;
                subghz_scene_send_event(&app, SubGhzEventTick);
            }
        }

        /* Redraw if needed */
        if (app.need_redraw)
        {
            subghz_scene_draw(&app);
            app.need_redraw = false;
        }
    }

    /* Cleanup */
    if (s_subghz_forced_landscape) { m1_orient_restore(); s_subghz_forced_landscape = false; }
    menu_sub_ghz_exit();
    xQueueReset(button_events_q_hdl);
    xQueueReset(main_q_hdl);
}
