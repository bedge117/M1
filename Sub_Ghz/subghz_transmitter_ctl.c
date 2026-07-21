/* See COPYING.txt for license details. */

/**
 * @file   subghz_transmitter_ctl.c
 * @brief  Sub-GHz Transmitter scene controller — pure logic.
 *
 * See subghz_transmitter_ctl.h for the full design rationale.
 *
 * Transition matrix summary:
 *
 *   READY:
 *     OK_PRESS       → init engine + START → phase=TX, act=TX_START
 *     BACK           → act=EXIT_SCENE
 *     LEFT / RIGHT   → (allow_button_cycle && button_count>1) ?
 *                        cycle index + act=CYCLE_BUTTON_PREV/NEXT :
 *                        act=NONE
 *     others         → act=NONE
 *
 *   TX:
 *     TX_BURST_COMPLETE → forward to engine; map action:
 *         engine TX_NEXT → act=TX_NEXT_BURST
 *         engine IDLE_NOW → phase=EXITING (after teardown), act=TX_TEARDOWN
 *         engine STAY    → act=NONE  (e.g. FINALIZING entry)
 *     OK_RELEASE     → forward to engine; engine action mapped same way
 *     BACK           → forward ABORT; phase=EXITING; act=TX_TEARDOWN
 *     others         → act=NONE
 *
 *   EXITING:
 *     TEARDOWN_DONE  → act=EXIT_SCENE
 *     BACK           → act=NONE (already exiting)
 *     others         → act=NONE
 *
 * Two test invariants the dispatcher must always satisfy:
 *   - The controller never returns TX_NEXT_BURST while phase != TX.
 *   - The controller never returns TX_START while phase != READY.
 *   - EXIT_SCENE is only returned from READY (immediate BACK) or
 *     EXITING (after TEARDOWN_DONE).
 */

#include "subghz_transmitter_ctl.h"
#include <stddef.h>

void subghz_transmitter_ctl_init(subghz_transmitter_ctl_t *c,
                                  subghz_endless_tx_mode_t mode,
                                  uint16_t repeat_count,
                                  bool allow_button_cycle,
                                  uint8_t button_count)
{
    if (c == NULL) return;

    c->phase              = SUBGHZ_TXCTL_PHASE_READY;
    c->mode               = mode;
    c->repeat_count       = repeat_count;
    c->allow_button_cycle = allow_button_cycle;
    c->button_index       = 0;
    c->button_count       = button_count;

    /* Initialise the embedded burst-policy engine.  endless_tx clamps
     * repeat_count to >= 1 internally, so we don't repeat the guard. */
    subghz_endless_tx_init(&c->etx, mode, repeat_count);
}

/*----------------------------------------------------------------------------*/
/* Helpers                                                                    */
/*----------------------------------------------------------------------------*/

/**
 * @brief  Translate a @ref subghz_endless_tx_action_t into the scene-level
 *         action vocabulary.  Also updates the controller phase when
 *         the engine signals teardown.
 */
static subghz_transmitter_action_t map_engine_action(
    subghz_transmitter_ctl_t *c,
    subghz_endless_tx_action_t engine_action)
{
    switch (engine_action)
    {
        case SUBGHZ_ETX_ACT_TX_NEXT:
            return SUBGHZ_TXCTL_ACT_TX_NEXT_BURST;

        case SUBGHZ_ETX_ACT_IDLE_NOW:
            /* Engine has finished (DONE or ABORTED).  Park in EXITING
             * so the scene can perform radio teardown and then send
             * TEARDOWN_DONE for the final EXIT_SCENE. */
            c->phase = SUBGHZ_TXCTL_PHASE_EXITING;
            return SUBGHZ_TXCTL_ACT_TX_TEARDOWN;

        case SUBGHZ_ETX_ACT_STAY:
        default:
            return SUBGHZ_TXCTL_ACT_NONE;
    }
}

/**
 * @brief  Handle LEFT / RIGHT in READY phase — cycle button_index when
 *         allowed, otherwise no-op.  Returns the cycle action to emit.
 */
static subghz_transmitter_action_t cycle_button(
    subghz_transmitter_ctl_t *c, bool forward)
{
    /* Gating: feature enabled AND more than one button.  button_count==1
     * means there is nothing to cycle to. */
    if (!c->allow_button_cycle || c->button_count <= 1)
        return SUBGHZ_TXCTL_ACT_NONE;

    if (forward)
    {
        c->button_index = (uint8_t)((c->button_index + 1u) % c->button_count);
        return SUBGHZ_TXCTL_ACT_CYCLE_BUTTON_NEXT;
    }
    /* Backward — wrap with modular arithmetic that works for any count. */
    c->button_index = (uint8_t)((c->button_index + c->button_count - 1u)
                                % c->button_count);
    return SUBGHZ_TXCTL_ACT_CYCLE_BUTTON_PREV;
}

/*----------------------------------------------------------------------------*/
/* Event dispatcher                                                           */
/*----------------------------------------------------------------------------*/

subghz_transmitter_action_t subghz_transmitter_ctl_event(
    subghz_transmitter_ctl_t *c,
    subghz_transmitter_event_t event)
{
    if (c == NULL) return SUBGHZ_TXCTL_ACT_NONE;

    switch (c->phase)
    {
        case SUBGHZ_TXCTL_PHASE_READY:
            switch (event)
            {
                case SUBGHZ_TXCTL_EVT_OK_PRESS:
                    /* Re-init the engine so a second TX run from the same
                     * loaded key starts from a clean slate (bursts_emitted
                     * resets, FINALIZING tail counters cleared). */
                    subghz_endless_tx_init(&c->etx, c->mode, c->repeat_count);
                    (void)subghz_endless_tx_event(&c->etx,
                                                   SUBGHZ_ETX_EVT_START);
                    c->phase = SUBGHZ_TXCTL_PHASE_TX;
                    return SUBGHZ_TXCTL_ACT_TX_START;

                case SUBGHZ_TXCTL_EVT_BACK:
                    return SUBGHZ_TXCTL_ACT_EXIT_SCENE;

                case SUBGHZ_TXCTL_EVT_LEFT:
                    return cycle_button(c, /*forward=*/false);

                case SUBGHZ_TXCTL_EVT_RIGHT:
                    return cycle_button(c, /*forward=*/true);

                default:
                    /* OK_RELEASE / TX_BURST_COMPLETE / TEARDOWN_DONE in
                     * READY are spurious — ignore.  This commonly happens
                     * if a stale event arrives just after a previous TX
                     * has fully torn down. */
                    return SUBGHZ_TXCTL_ACT_NONE;
            }

        case SUBGHZ_TXCTL_PHASE_TX:
            switch (event)
            {
                case SUBGHZ_TXCTL_EVT_TX_BURST_COMPLETE:
                    return map_engine_action(c,
                        subghz_endless_tx_event(&c->etx,
                            SUBGHZ_ETX_EVT_BURST_COMPLETE));

                case SUBGHZ_TXCTL_EVT_OK_RELEASE:
                    return map_engine_action(c,
                        subghz_endless_tx_event(&c->etx,
                            SUBGHZ_ETX_EVT_OK_RELEASED));

                case SUBGHZ_TXCTL_EVT_BACK:
                    /* Cancel — abort the engine and tear down TX.
                     * map_engine_action() handles the EXITING transition. */
                    return map_engine_action(c,
                        subghz_endless_tx_event(&c->etx,
                            SUBGHZ_ETX_EVT_ABORT));

                default:
                    /* OK_PRESS / LEFT / RIGHT / TEARDOWN_DONE during TX
                     * are ignored — TX must finalise via BURST_COMPLETE
                     * or BACK. */
                    return SUBGHZ_TXCTL_ACT_NONE;
            }

        case SUBGHZ_TXCTL_PHASE_EXITING:
            switch (event)
            {
                case SUBGHZ_TXCTL_EVT_TEARDOWN_DONE:
                    return SUBGHZ_TXCTL_ACT_EXIT_SCENE;

                default:
                    /* BACK / others during teardown are ignored — exit
                     * is already in flight. */
                    return SUBGHZ_TXCTL_ACT_NONE;
            }
    }

    return SUBGHZ_TXCTL_ACT_NONE;
}
