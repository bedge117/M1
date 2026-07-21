/* See COPYING.txt for license details. */

/**
 * @file   subghz_txrx_state.h
 * @brief  Sub-GHz radio lifecycle state machine — pure logic, host-testable.
 *
 * Models the SI4463 radio lifecycle as an explicit state machine so that
 * legal transitions can be enforced at the scene/wrapper layer instead of
 * relying on scattered discipline rules.
 *
 * This module is hardware-independent: it does NOT touch the SI4463, TIM1,
 * DMA, or any HAL.  Hardware-coupled wrappers (introduced in later commits)
 * call into this state machine to validate transitions and report current
 * state.  This separation makes the lifecycle logic unit-testable on the
 * host and removes a long-standing class of "radio works in tool X but not
 * in tool Y" bugs.
 *
 * States:
 *   OFF       — Radio powered off (after menu_sub_ghz_exit, after init failure)
 *   IDLE      — Radio powered on but neither receiving nor transmitting
 *   RX_PASSIVE— Receiving for live RSSI only; TIM1 input-capture NOT armed
 *   RX_ACTIVE — Receiving with TIM1 IC armed; edges flow to ring buffer
 *   TX_BLOCK  — Blocking TX in progress (legacy wrappers)
 *   TX_ASYNC  — Async TX in progress (DMA TC posts Q_EVENT_SUBGHZ_TX)
 *
 * Legal transitions form this graph (any transition not listed is rejected):
 *
 *   OFF        → IDLE                      (power on)
 *   IDLE       → OFF                       (power off)
 *   IDLE       → RX_PASSIVE                (start passive RX)
 *   IDLE       → RX_ACTIVE                 (start full RX)
 *   IDLE       → TX_BLOCK                  (start blocking replay)
 *   IDLE       → TX_ASYNC                  (sub_ghz_replay_start_async)
 *   RX_PASSIVE → IDLE                      (stop)
 *   RX_PASSIVE → RX_ACTIVE                 (arm IC for capture)
 *   RX_PASSIVE → OFF                       (full deinit)
 *   RX_ACTIVE  → RX_PASSIVE                (pause IC, keep radio in RX)
 *   RX_ACTIVE  → IDLE                      (stop)
 *   RX_ACTIVE  → OFF                       (full deinit)
 *   TX_BLOCK   → OFF                       (legacy wrappers exit with radio off)
 *   TX_BLOCK   → IDLE                      (after menu_sub_ghz_init restore)
 *   TX_ASYNC   → IDLE                      (continue_async DONE/ERROR or abort)
 *   TX_ASYNC   → RX_PASSIVE                (scene resumes listen after TX)
 *
 * The state machine intentionally does NOT model:
 *   - Frequency / modulation changes (orthogonal to lifecycle)
 *   - Hopper state (handled by a separate timer in scenes)
 *   - TX repeat counting (encoder-internal)
 */

#ifndef SUBGHZ_TXRX_STATE_H_
#define SUBGHZ_TXRX_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Radio lifecycle states.
 */
typedef enum {
    SUBGHZ_TXRX_STATE_OFF = 0,    /**< Radio powered off */
    SUBGHZ_TXRX_STATE_IDLE,       /**< Powered on, idle */
    SUBGHZ_TXRX_STATE_RX_PASSIVE, /**< RX for RSSI only; IC not armed */
    SUBGHZ_TXRX_STATE_RX_ACTIVE,  /**< RX with TIM1 IC armed */
    SUBGHZ_TXRX_STATE_TX_BLOCK,   /**< Blocking TX (legacy wrappers) */
    SUBGHZ_TXRX_STATE_TX_ASYNC,   /**< Async TX (DMA-driven) */
    SUBGHZ_TXRX_STATE_COUNT
} subghz_txrx_state_t;

/**
 * @brief  Check whether a transition between two states is legal.
 *
 * Pure function — no side effects.  Used both by the state machine itself
 * and by tests/diagnostics that need to validate a planned transition
 * without committing it.
 *
 * @param  from   Current state
 * @param  to     Desired next state
 * @retval true   The transition is legal
 * @retval false  The transition is not legal (or either argument is invalid)
 */
bool subghz_txrx_state_transition_is_legal(subghz_txrx_state_t from,
                                            subghz_txrx_state_t to);

/**
 * @brief  Return true if the given state represents "radio actively in RX".
 *         (Either RX_PASSIVE or RX_ACTIVE.)
 */
bool subghz_txrx_state_is_rx(subghz_txrx_state_t s);

/**
 * @brief  Return true if the given state represents "radio actively in TX".
 *         (Either TX_BLOCK or TX_ASYNC.)
 */
bool subghz_txrx_state_is_tx(subghz_txrx_state_t s);

/**
 * @brief  Return true if the given state represents "radio is powered".
 *         (Anything other than OFF.)
 */
bool subghz_txrx_state_is_powered(subghz_txrx_state_t s);

/**
 * @brief  Human-readable name for a state — for logging and diagnostics.
 *         Returns "INVALID" for out-of-range values.  Never NULL.
 */
const char *subghz_txrx_state_name(subghz_txrx_state_t s);

/*============================================================================*/
/* Stateful façade — wraps a single subghz_txrx_state_t with transition       */
/* enforcement.  Use this when you want one place to track the current state. */
/*============================================================================*/

typedef struct {
    subghz_txrx_state_t state;
    uint32_t            illegal_transition_count; /**< Diagnostic: rejected transitions */
} subghz_txrx_state_ctx_t;

/**
 * @brief  Initialise a state context to OFF with zero diagnostic counters.
 */
void subghz_txrx_state_init(subghz_txrx_state_ctx_t *ctx);

/**
 * @brief  Attempt to transition the context to the requested state.
 *
 * If the transition is legal, the context is updated and the function
 * returns true.  Otherwise, the context is left unchanged, the diagnostic
 * counter is incremented, and the function returns false.
 *
 * A "transition" to the same state is treated as a no-op and returns true
 * without incrementing any counter.
 *
 * @param  ctx   State context (must be non-NULL)
 * @param  to    Desired next state
 * @retval true  Transition accepted (or no-op)
 * @retval false Transition rejected; ctx unchanged, counter incremented
 */
bool subghz_txrx_state_try_transition(subghz_txrx_state_ctx_t *ctx,
                                       subghz_txrx_state_t to);

/**
 * @brief  Force the context to a specific state regardless of legality.
 *
 * Use ONLY for error recovery paths where the hardware state is known
 * to have been forced (e.g. peripheral reset).  Normal code MUST use
 * subghz_txrx_state_try_transition().  Increments no counters.
 */
void subghz_txrx_state_force(subghz_txrx_state_ctx_t *ctx,
                              subghz_txrx_state_t to);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_TXRX_STATE_H_ */
