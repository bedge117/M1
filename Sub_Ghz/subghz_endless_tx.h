/* See COPYING.txt for license details. */

/**
 * @file   subghz_endless_tx.h
 * @brief  Sub-GHz TX repeat / endless-hold policy — pure logic, host-testable.
 *
 * Phase 3a of the Momentum parity migration.  Models the behaviour that the
 * upcoming Transmitter scene (Phase 3b) needs in order to support:
 *
 *   - **SINGLE** mode — transmit a key burst N times then stop.  N comes
 *     from the protocol's recommended repeat count (typical: 3 for rolling
 *     codes, 1 for static OOK).
 *
 *   - **ENDLESS** mode — keep transmitting while OK is held, with
 *     **graceful N-repeat finalisation on release**.  When OK is released
 *     mid-burst the engine does not cut the transmission immediately; it
 *     completes the in-flight burst and then sends N-1 more.  This avoids
 *     receivers seeing a half-key on the last hop and matches Momentum's
 *     `endless_tx` behaviour.
 *
 * The module is hardware-independent: it does NOT touch the SI4463, DMA,
 * TIM1, RTOS queues, or any HAL.  The Transmitter scene will drive it from
 * its `on_event` handler when `Q_EVENT_SUBGHZ_TX` arrives, then act on the
 * returned action (re-arm next burst, or transition to idle).
 *
 * Event vocabulary (driven by the scene):
 *   START          — caller has armed the first burst.  State → RUNNING.
 *   BURST_COMPLETE — DMA TC for the previous burst has fired.
 *   OK_RELEASED    — physical OK button transitioned to released.
 *   ABORT          — user pressed BACK or scene is exiting.
 *
 * Action vocabulary (consumed by the scene):
 *   TX_NEXT  — arm another burst now.
 *   IDLE_NOW — tear down TX and return to idle/listen.
 *   STAY     — no-op, keep waiting for the next event.
 */

#ifndef SUBGHZ_ENDLESS_TX_H_
#define SUBGHZ_ENDLESS_TX_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  TX request mode.
 */
typedef enum {
    SUBGHZ_TX_MODE_SINGLE = 0,  /**< Transmit `repeat_count` bursts then stop. */
    SUBGHZ_TX_MODE_ENDLESS,     /**< Repeat while OK is held; finalise gracefully. */
} subghz_endless_tx_mode_t;

/**
 * @brief  Engine state.
 */
typedef enum {
    SUBGHZ_ETX_IDLE = 0,        /**< Not yet started, or already finished. */
    SUBGHZ_ETX_RUNNING,         /**< Bursts in flight; in ENDLESS mode the user
                                     is still pressing OK. */
    SUBGHZ_ETX_FINALIZING,      /**< ENDLESS mode only — OK was released and
                                     the engine is sending the remaining N-1
                                     "tail" bursts before idling. */
    SUBGHZ_ETX_DONE,            /**< All scheduled bursts completed. */
    SUBGHZ_ETX_ABORTED,         /**< Engine was aborted by the caller. */
} subghz_endless_tx_state_t;

/**
 * @brief  Events the engine reacts to.  Driven by the scene/caller.
 */
typedef enum {
    SUBGHZ_ETX_EVT_START = 0,         /**< Initial burst armed by caller.    */
    SUBGHZ_ETX_EVT_BURST_COMPLETE,    /**< DMA TC for previous burst fired.  */
    SUBGHZ_ETX_EVT_OK_RELEASED,       /**< Physical OK transitioned to off.  */
    SUBGHZ_ETX_EVT_ABORT,             /**< Cancel — BACK / scene exit.       */
} subghz_endless_tx_event_t;

/**
 * @brief  Actions the engine returns.  Consumed by the scene/caller.
 */
typedef enum {
    SUBGHZ_ETX_ACT_STAY = 0,    /**< No-op — keep waiting. */
    SUBGHZ_ETX_ACT_TX_NEXT,     /**< Arm another burst now. */
    SUBGHZ_ETX_ACT_IDLE_NOW,    /**< Tear down TX and return to idle. */
} subghz_endless_tx_action_t;

/**
 * @brief  Engine context.  Embed in scene context or stack-allocate.
 *         All fields are observable for tests/diagnostics but should be
 *         treated as private by scene code (use accessors below).
 */
typedef struct {
    subghz_endless_tx_state_t state;
    subghz_endless_tx_mode_t  mode;

    /** SINGLE mode: total bursts to transmit (clamped to >= 1).
     *  ENDLESS mode: tail-burst count for graceful finalisation
     *  (clamped to >= 1). */
    uint16_t repeat_count;

    /** Bursts emitted so far (incremented when caller acts on TX_NEXT
     *  and on START).  Test-visible. */
    uint16_t bursts_emitted;

    /** Bursts remaining in the current phase:
     *   - SINGLE running:        N-1, N-2, …, 0 (then IDLE_NOW)
     *   - ENDLESS running:       always 0 (loops while held)
     *   - ENDLESS finalising:    N-1, …, 0 (then IDLE_NOW)
     */
    uint16_t bursts_remaining;
} subghz_endless_tx_t;

/*----------------------------------------------------------------------------*/
/* API                                                                        */
/*----------------------------------------------------------------------------*/

/**
 * @brief  Initialise the engine.  Safe to call with NULL (no-op).
 *
 * After this call the engine is in @ref SUBGHZ_ETX_IDLE.  Send
 * @ref SUBGHZ_ETX_EVT_START once the caller has armed the first burst.
 *
 * @param e             Engine context.
 * @param mode          SINGLE or ENDLESS.
 * @param repeat_count  Tail count.  Clamped to >= 1.  Typical values:
 *                      3 for rolling-code reliability; 1 for static keys.
 */
void subghz_endless_tx_init(subghz_endless_tx_t *e,
                             subghz_endless_tx_mode_t mode,
                             uint16_t repeat_count);

/**
 * @brief  Drive the engine with an event; receive the action to perform.
 *
 * Pure function: never blocks, never allocates, never touches hardware.
 * Returns @ref SUBGHZ_ETX_ACT_STAY for NULL engine pointers.
 *
 * @param e      Engine context.
 * @param event  The event that just occurred.
 * @return Action the caller must perform.
 */
subghz_endless_tx_action_t subghz_endless_tx_event(
    subghz_endless_tx_t *e,
    subghz_endless_tx_event_t event);

/**
 * @brief  Current state (read-only accessor).
 */
static inline subghz_endless_tx_state_t subghz_endless_tx_state(
    const subghz_endless_tx_t *e)
{
    return (e != NULL) ? e->state : SUBGHZ_ETX_IDLE;
}

/**
 * @brief  True iff the engine has reached a terminal state
 *         (@ref SUBGHZ_ETX_DONE or @ref SUBGHZ_ETX_ABORTED).
 */
static inline bool subghz_endless_tx_is_finished(const subghz_endless_tx_t *e)
{
    if (e == NULL) return true;
    return e->state == SUBGHZ_ETX_DONE || e->state == SUBGHZ_ETX_ABORTED;
}

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_ENDLESS_TX_H_ */
