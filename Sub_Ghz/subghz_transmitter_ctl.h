/* See COPYING.txt for license details. */

/**
 * @file   subghz_transmitter_ctl.h
 * @brief  Sub-GHz Transmitter scene controller — pure logic, host-testable.
 *
 * Phase 3b-1 of the Momentum parity migration.  Provides the high-level
 * state machine that the upcoming Transmitter scene (Phase 3b-2) will
 * drive from its `on_event` handler.  The controller sits one layer
 * **above** the @ref subghz_endless_tx engine (Phase 3a) and one layer
 * **below** the scene/RTOS code:
 *
 *     scene on_event  → subghz_transmitter_ctl_event()
 *                        ├─ owns READY / TX / EXITING phase
 *                        ├─ delegates burst accounting to subghz_endless_tx
 *                        └─ returns a high-level action for the scene
 *                           (TX_START / TX_NEXT_BURST / TX_TEARDOWN /
 *                            EXIT_SCENE / CYCLE_BUTTON_*)
 *
 * The controller is hardware-independent: no SI4463, no DMA, no RTOS, no
 * FAT FS.  All radio/file operations are performed by the scene in
 * response to the returned action.
 *
 * Scene phase vocabulary:
 *   READY    — Key file is loaded, radio is idle, awaiting OK_PRESS.
 *              BACK from this phase exits the scene immediately.
 *   TX       — endless_tx is driving bursts (the embedded engine is in
 *              RUNNING or FINALIZING).  BACK aborts the engine, returns
 *              TX_TEARDOWN, and parks in EXITING.
 *   EXITING  — Radio is being torn down; once teardown is acknowledged
 *              (by the next event call after TX_TEARDOWN), the controller
 *              returns EXIT_SCENE.
 *
 * Event vocabulary (driven by the scene):
 *   OK_PRESS         — User pressed OK; controller transitions READY → TX
 *                      and returns TX_START so the scene arms the radio.
 *   OK_RELEASE       — Hardware OK transitioned to released; forwarded
 *                      to endless_tx (only meaningful in ENDLESS mode).
 *   TX_BURST_COMPLETE — Q_EVENT_SUBGHZ_TX arrived for the previous burst.
 *                       Forwarded to endless_tx; the engine's action is
 *                       translated to TX_NEXT_BURST or TX_TEARDOWN.
 *   BACK             — Hardware BACK button.  In READY → EXIT_SCENE.
 *                      In TX → TX_TEARDOWN + park in EXITING.
 *   TEARDOWN_DONE    — Scene confirms TX teardown is finished; controller
 *                      returns EXIT_SCENE.
 *   LEFT / RIGHT     — Hardware LEFT/RIGHT.  In READY when
 *                      @ref subghz_transmitter_ctl_t.allow_button_cycle
 *                      is true, cycles button_index 0..button_count-1
 *                      and returns CYCLE_BUTTON_PREV/NEXT so the scene
 *                      can reload the key for the new button.  Otherwise
 *                      ignored.  (Phase 4 fully wires this up.)
 *
 * Design rationale: keeping this module pure (no globals, no HAL, no
 * radio code) means scene event routing is fully unit-testable on the
 * host.  Every event-→-action transition that the scene relies on can
 * be asserted in CI before the firmware is built.
 */

#ifndef SUBGHZ_TRANSMITTER_CTL_H_
#define SUBGHZ_TRANSMITTER_CTL_H_

#include <stdint.h>
#include <stdbool.h>
#include "subghz_endless_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Scene-level phase.
 */
typedef enum {
    SUBGHZ_TXCTL_PHASE_READY = 0,   /**< Key loaded, idle, awaiting OK. */
    SUBGHZ_TXCTL_PHASE_TX,          /**< endless_tx engine in flight.   */
    SUBGHZ_TXCTL_PHASE_EXITING,     /**< Teardown in progress; next
                                          event returns EXIT_SCENE.    */
} subghz_transmitter_phase_t;

/**
 * @brief  Events from the scene to the controller.
 */
typedef enum {
    SUBGHZ_TXCTL_EVT_OK_PRESS = 0,
    SUBGHZ_TXCTL_EVT_OK_RELEASE,
    SUBGHZ_TXCTL_EVT_TX_BURST_COMPLETE,
    SUBGHZ_TXCTL_EVT_BACK,
    SUBGHZ_TXCTL_EVT_TEARDOWN_DONE,
    SUBGHZ_TXCTL_EVT_LEFT,
    SUBGHZ_TXCTL_EVT_RIGHT,
} subghz_transmitter_event_t;

/**
 * @brief  Actions returned to the scene.
 */
typedef enum {
    SUBGHZ_TXCTL_ACT_NONE = 0,       /**< No-op.                          */
    SUBGHZ_TXCTL_ACT_TX_START,       /**< Arm radio + emit first burst.   */
    SUBGHZ_TXCTL_ACT_TX_NEXT_BURST,  /**< Emit next burst (radio armed).  */
    SUBGHZ_TXCTL_ACT_TX_TEARDOWN,    /**< Idle radio; awaiting TEARDOWN_DONE. */
    SUBGHZ_TXCTL_ACT_EXIT_SCENE,     /**< Pop the Transmitter scene.      */
    SUBGHZ_TXCTL_ACT_CYCLE_BUTTON_PREV, /**< Reload key for button_index-1. */
    SUBGHZ_TXCTL_ACT_CYCLE_BUTTON_NEXT, /**< Reload key for button_index+1. */
} subghz_transmitter_action_t;

/**
 * @brief  Controller context.  All fields are observable for diagnostics
 *         and tests; treat as private from scene code (use accessors).
 */
typedef struct {
    subghz_transmitter_phase_t phase;

    /** Embedded burst-policy engine.  Initialised by
     *  @ref subghz_transmitter_ctl_init() with the same `mode` and
     *  `repeat_count` passed in. */
    subghz_endless_tx_t etx;

    /** TX mode + repeat count — cached so OK_PRESS can re-init the
     *  engine cleanly for a second TX after the first completes. */
    subghz_endless_tx_mode_t mode;
    uint16_t repeat_count;

    /** True when the loaded protocol supports M1-replayable rolling-code
     *  TX with multiple buttons (KeeLoq, Nice FloR-S, CAME Atomo, FAAC,
     *  Alutech, etc.).  Phase 4 sets this from
     *  `SubGhzProtocolFlag_PwmKeyReplay` on Dynamic protocols. */
    bool    allow_button_cycle;

    /** Current button index in the cycle, 0..button_count-1.  Wraps. */
    uint8_t button_index;

    /** Total buttons available (typically 4 for KeeLoq-family remotes).
     *  Zero means "single button only" — LEFT/RIGHT are no-ops even if
     *  allow_button_cycle is true. */
    uint8_t button_count;
} subghz_transmitter_ctl_t;

/*----------------------------------------------------------------------------*/
/* API                                                                        */
/*----------------------------------------------------------------------------*/

/**
 * @brief  Initialise the controller.
 *
 * Leaves the controller in @ref SUBGHZ_TXCTL_PHASE_READY with the
 * embedded engine in @ref SUBGHZ_ETX_IDLE.  Caller passes the file's
 * recommended `repeat_count` (typical: 1 for static, 3 for rolling).
 *
 * Safe to call with NULL (no-op).
 *
 * @param c                   Controller context.
 * @param mode                SINGLE or ENDLESS.
 * @param repeat_count        Tail count, clamped to >= 1.
 * @param allow_button_cycle  True iff the loaded protocol supports
 *                            multi-button rolling-code remotes.
 * @param button_count        Number of buttons (e.g. 4 for KeeLoq).
 *                            Zero disables LEFT/RIGHT cycling.
 */
void subghz_transmitter_ctl_init(subghz_transmitter_ctl_t *c,
                                  subghz_endless_tx_mode_t mode,
                                  uint16_t repeat_count,
                                  bool allow_button_cycle,
                                  uint8_t button_count);

/**
 * @brief  Drive the controller with an event; return the action to perform.
 *
 * Pure function: no blocking, no allocation, no hardware access.  NULL
 * pointer returns @ref SUBGHZ_TXCTL_ACT_NONE.
 */
subghz_transmitter_action_t subghz_transmitter_ctl_event(
    subghz_transmitter_ctl_t *c,
    subghz_transmitter_event_t event);

/**
 * @brief  Current phase (read-only).
 */
static inline subghz_transmitter_phase_t subghz_transmitter_ctl_phase(
    const subghz_transmitter_ctl_t *c)
{
    return (c != NULL) ? c->phase : SUBGHZ_TXCTL_PHASE_READY;
}

/**
 * @brief  True iff the controller is currently driving a TX in flight
 *         (engine in RUNNING or FINALIZING).
 */
static inline bool subghz_transmitter_ctl_is_tx(
    const subghz_transmitter_ctl_t *c)
{
    return (c != NULL) && (c->phase == SUBGHZ_TXCTL_PHASE_TX);
}

/**
 * @brief  Number of bursts emitted so far in the current TX run.
 *         Useful for the burst-counter display in the scene.
 */
static inline uint16_t subghz_transmitter_ctl_bursts_emitted(
    const subghz_transmitter_ctl_t *c)
{
    return (c != NULL) ? c->etx.bursts_emitted : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_TRANSMITTER_CTL_H_ */
