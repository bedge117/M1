/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_read_raw_state.h
 * @brief  Pure-logic enum + inline helpers for the Sub-GHz Read Raw scene
 *         state machine.  Extracted from m1_subghz_scene.h so that host
 *         unit tests can exercise the dispatch logic without pulling in the
 *         full scene context (SubGhzApp, decenc state, etc.).
 *
 *  Convention: this header has no dependencies beyond <stdbool.h>/<stdint.h> so it can
 *  be included in both firmware and tests.  Do NOT add anything here that
 *  references hardware, FreeRTOS, or the scene framework.
 */

#ifndef M1_SUBGHZ_READ_RAW_STATE_H_
#define M1_SUBGHZ_READ_RAW_STATE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SubGhzReadRawStateStart = 0,   /**< Fresh entry — no capture exists */
    SubGhzReadRawStateRecording,   /**< Actively recording raw RF data */
    SubGhzReadRawStateIdle,        /**< Recording done — capture file on SD */
    SubGhzReadRawStateLoaded,      /**< Pre-existing RAW file loaded from Saved browser (Momentum: LoadKeyIDLE) */

    /* --- Momentum-aligned async TX states --- */
    SubGhzReadRawStateTX,                  /**< Async TX from freshly-recorded capture (Idle → TX → Idle) */
    SubGhzReadRawStateTXRepeat,            /**< Hold-to-repeat from Idle (TX → TXRepeat while OK held) */
    SubGhzReadRawStateLoadKeyTX,           /**< Async TX from pre-loaded file (Loaded → LoadKeyTX → Loaded) */
    SubGhzReadRawStateLoadKeyTXRepeat,     /**< Hold-to-repeat from Loaded (LoadKeyTX → LoadKeyTXRepeat while OK held) */
} SubGhzReadRawState;

/**
 * @brief  Pure-logic test: true if the state is one of the four TX states.
 *
 * Extracted as a named function so callers do not inline four-way OR checks
 * that would silently drift from the enum when new TX states are added.
 */
static inline bool subghz_read_raw_state_is_tx(SubGhzReadRawState s)
{
    return s == SubGhzReadRawStateTX
        || s == SubGhzReadRawStateTXRepeat
        || s == SubGhzReadRawStateLoadKeyTX
        || s == SubGhzReadRawStateLoadKeyTXRepeat;
}

/**
 * @brief  Pure-logic: post-TX return state.
 *
 * @param  tx_state  Any of the four TX states (or a non-TX state).
 * @retval  SubGhzReadRawStateLoaded  when the TX originated from a pre-loaded file
 *                                     (LoadKeyTX / LoadKeyTXRepeat).
 * @retval  SubGhzReadRawStateIdle    otherwise (TX / TXRepeat — freshly-recorded
 *                                     capture).  Also the safe fall-through when
 *                                     called with a non-TX state.
 */
static inline SubGhzReadRawState subghz_read_raw_state_after_tx(SubGhzReadRawState tx_state)
{
    if (tx_state == SubGhzReadRawStateLoadKeyTX ||
        tx_state == SubGhzReadRawStateLoadKeyTXRepeat)
        return SubGhzReadRawStateLoaded;
    return SubGhzReadRawStateIdle;
}

/**
 * @brief  Pure-logic: TX → repeat-state transition for Phase 2 hold-to-repeat.
 *
 * Called from the Read Raw scene's SubGhzEventTxComplete handler when the OK
 * button is still held at the end of a TX burst.  Promotes the one-shot TX
 * states to their hold-to-repeat counterparts and leaves the already-repeating
 * states (or any non-TX state) unchanged.
 *
 * @param  s  Current scene state.
 * @retval  SubGhzReadRawStateTXRepeat         when s was TX or TXRepeat
 *          SubGhzReadRawStateLoadKeyTXRepeat  when s was LoadKeyTX or LoadKeyTXRepeat
 *          s (unchanged)                       for any non-TX state
 */
static inline SubGhzReadRawState subghz_read_raw_state_to_repeat(SubGhzReadRawState s)
{
    switch (s)
    {
        case SubGhzReadRawStateTX:
        case SubGhzReadRawStateTXRepeat:
            return SubGhzReadRawStateTXRepeat;
        case SubGhzReadRawStateLoadKeyTX:
        case SubGhzReadRawStateLoadKeyTXRepeat:
            return SubGhzReadRawStateLoadKeyTXRepeat;
        default:
            return s;
    }
}

/**
 * @brief  Pure-logic: gate the displayed "N spl." counter by signal-present.
 *
 * The waveform cursor in the Read Raw scene only advances when RSSI is above
 * the user-configured threshold (signal_present == true).  When no signal is
 * detected, the cursor freezes.  The displayed SPL counter must follow the
 * same gating — otherwise it ticks up from ambient RF noise that passes the
 * 80 µs noise filter in the TIM1 input-capture ISR, giving the false
 * impression that a capture is filling up.  See issue
 * "Read Raw is counting SPL when none are being collected".
 *
 * @param  current_displayed     Value currently shown in the status bar.
 * @param  new_total             Total samples written to SD since recording started
 *                               (from sub_ghz_raw_recording_get_total_samples_ext()).
 * @param  signal_seen_in_chunk  true if signal_present was observed at least once
 *                               since the previous Q_EVENT_SUBGHZ_RX flush.
 * @retval  new_total            when signal_seen_in_chunk is true (cursor advanced)
 * @retval  current_displayed    when signal_seen_in_chunk is false (cursor frozen)
 */
static inline uint32_t subghz_read_raw_display_count_update(
    uint32_t current_displayed,
    uint32_t new_total,
    bool     signal_seen_in_chunk)
{
    return signal_seen_in_chunk ? new_total : current_displayed;
}

#endif /* M1_SUBGHZ_READ_RAW_STATE_H_ */
