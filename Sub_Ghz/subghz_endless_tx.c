/* See COPYING.txt for license details. */

/**
 * @file   subghz_endless_tx.c
 * @brief  Sub-GHz TX repeat / endless-hold policy — pure logic.
 */

#include "subghz_endless_tx.h"
#include <stddef.h>

void subghz_endless_tx_init(subghz_endless_tx_t *e,
                             subghz_endless_tx_mode_t mode,
                             uint16_t repeat_count)
{
    if (e == NULL) return;

    /* Defensive clamp — callers passing 0 would otherwise underflow the
     * remaining counter on the first BURST_COMPLETE in SINGLE mode and
     * never schedule the tail in ENDLESS mode. */
    if (repeat_count == 0) repeat_count = 1;

    e->state            = SUBGHZ_ETX_IDLE;
    e->mode             = mode;
    e->repeat_count     = repeat_count;
    e->bursts_emitted   = 0;
    e->bursts_remaining = 0;
}

subghz_endless_tx_action_t subghz_endless_tx_event(
    subghz_endless_tx_t *e,
    subghz_endless_tx_event_t event)
{
    if (e == NULL) return SUBGHZ_ETX_ACT_STAY;

    /* ABORT is universal — handle before per-state dispatch. */
    if (event == SUBGHZ_ETX_EVT_ABORT)
    {
        if (e->state == SUBGHZ_ETX_DONE || e->state == SUBGHZ_ETX_ABORTED)
            return SUBGHZ_ETX_ACT_STAY;  /* already terminal */
        e->state = SUBGHZ_ETX_ABORTED;
        return SUBGHZ_ETX_ACT_IDLE_NOW;
    }

    switch (e->state)
    {
        case SUBGHZ_ETX_IDLE:
            if (event == SUBGHZ_ETX_EVT_START)
            {
                /* Caller has armed the first burst — account for it and
                 * enter RUNNING.  Tail accounting is mode-specific:
                 *   SINGLE  — total N bursts, so N-1 still to emit.
                 *   ENDLESS — emits while held; no preset tail yet.
                 */
                e->bursts_emitted = 1;
                if (e->mode == SUBGHZ_TX_MODE_SINGLE)
                {
                    e->bursts_remaining =
                        (e->repeat_count > 1) ? (uint16_t)(e->repeat_count - 1) : 0;
                }
                else
                {
                    e->bursts_remaining = 0;
                }
                e->state = SUBGHZ_ETX_RUNNING;
            }
            /* All other events are ignored before START. */
            return SUBGHZ_ETX_ACT_STAY;

        case SUBGHZ_ETX_RUNNING:
            if (event == SUBGHZ_ETX_EVT_BURST_COMPLETE)
            {
                if (e->mode == SUBGHZ_TX_MODE_SINGLE)
                {
                    if (e->bursts_remaining > 0)
                    {
                        e->bursts_remaining--;
                        e->bursts_emitted++;
                        return SUBGHZ_ETX_ACT_TX_NEXT;
                    }
                    /* SINGLE mode is done. */
                    e->state = SUBGHZ_ETX_DONE;
                    return SUBGHZ_ETX_ACT_IDLE_NOW;
                }
                /* ENDLESS: still held, keep looping. */
                e->bursts_emitted++;
                return SUBGHZ_ETX_ACT_TX_NEXT;
            }
            if (event == SUBGHZ_ETX_EVT_OK_RELEASED)
            {
                if (e->mode == SUBGHZ_TX_MODE_SINGLE)
                {
                    /* Release is meaningless in SINGLE mode — keep counting. */
                    return SUBGHZ_ETX_ACT_STAY;
                }
                /* ENDLESS: transition to FINALIZING.  The tail starts at
                 * N-1 (current burst counts as the first tail burst).
                 * If repeat_count == 1, tail is empty and the *next*
                 * BURST_COMPLETE will drive us straight to DONE. */
                e->bursts_remaining =
                    (e->repeat_count > 1) ? (uint16_t)(e->repeat_count - 1) : 0;
                e->state = SUBGHZ_ETX_FINALIZING;
                return SUBGHZ_ETX_ACT_STAY;
            }
            return SUBGHZ_ETX_ACT_STAY;

        case SUBGHZ_ETX_FINALIZING:
            if (event == SUBGHZ_ETX_EVT_BURST_COMPLETE)
            {
                if (e->bursts_remaining > 0)
                {
                    e->bursts_remaining--;
                    e->bursts_emitted++;
                    return SUBGHZ_ETX_ACT_TX_NEXT;
                }
                e->state = SUBGHZ_ETX_DONE;
                return SUBGHZ_ETX_ACT_IDLE_NOW;
            }
            /* OK_RELEASED while already finalising is a no-op.  This can
             * happen with debounce noise or a second key event arriving
             * after the first triggered the transition. */
            return SUBGHZ_ETX_ACT_STAY;

        case SUBGHZ_ETX_DONE:
        case SUBGHZ_ETX_ABORTED:
            /* Terminal — ignore everything. */
            return SUBGHZ_ETX_ACT_STAY;
    }

    return SUBGHZ_ETX_ACT_STAY;
}
