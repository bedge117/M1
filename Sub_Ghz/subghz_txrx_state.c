/* See COPYING.txt for license details. */

/**
 * @file   subghz_txrx_state.c
 * @brief  Sub-GHz radio lifecycle state machine — pure logic, host-testable.
 *
 * See subghz_txrx_state.h for the state graph documentation.
 */

#include "subghz_txrx_state.h"
#include <stddef.h>

/*----------------------------------------------------------------------------*/
/* Transition table                                                           */
/*----------------------------------------------------------------------------*/

/* Bitmask of legal successor states per source state.  Indexed by
 * subghz_txrx_state_t value.  Bit N set => transition to state N is legal.
 * Same-state "transitions" are handled as no-ops by try_transition() and
 * are NOT encoded here (so that a buggy caller asking "is OFF→OFF legal?"
 * via the helper still returns false, forcing them to think about it).
 *
 * Keep in sync with the documentation block in the header. */
#define BIT(s) (1u << (s))

static const uint32_t k_legal_successors[SUBGHZ_TXRX_STATE_COUNT] = {
    /* OFF        → */ BIT(SUBGHZ_TXRX_STATE_IDLE),
    /* IDLE       → */ BIT(SUBGHZ_TXRX_STATE_OFF)
                       | BIT(SUBGHZ_TXRX_STATE_RX_PASSIVE)
                       | BIT(SUBGHZ_TXRX_STATE_RX_ACTIVE)
                       | BIT(SUBGHZ_TXRX_STATE_TX_BLOCK)
                       | BIT(SUBGHZ_TXRX_STATE_TX_ASYNC),
    /* RX_PASSIVE → */ BIT(SUBGHZ_TXRX_STATE_IDLE)
                       | BIT(SUBGHZ_TXRX_STATE_RX_ACTIVE)
                       | BIT(SUBGHZ_TXRX_STATE_OFF),
    /* RX_ACTIVE  → */ BIT(SUBGHZ_TXRX_STATE_RX_PASSIVE)
                       | BIT(SUBGHZ_TXRX_STATE_IDLE)
                       | BIT(SUBGHZ_TXRX_STATE_OFF),
    /* TX_BLOCK   → */ BIT(SUBGHZ_TXRX_STATE_OFF)
                       | BIT(SUBGHZ_TXRX_STATE_IDLE),
    /* TX_ASYNC   → */ BIT(SUBGHZ_TXRX_STATE_IDLE)
                       | BIT(SUBGHZ_TXRX_STATE_RX_PASSIVE),
};

/*----------------------------------------------------------------------------*/
/* Pure helpers                                                               */
/*----------------------------------------------------------------------------*/

bool subghz_txrx_state_transition_is_legal(subghz_txrx_state_t from,
                                            subghz_txrx_state_t to)
{
    if ((unsigned)from >= SUBGHZ_TXRX_STATE_COUNT) return false;
    if ((unsigned)to   >= SUBGHZ_TXRX_STATE_COUNT) return false;
    return (k_legal_successors[from] & BIT(to)) != 0u;
}

bool subghz_txrx_state_is_rx(subghz_txrx_state_t s)
{
    return s == SUBGHZ_TXRX_STATE_RX_PASSIVE
        || s == SUBGHZ_TXRX_STATE_RX_ACTIVE;
}

bool subghz_txrx_state_is_tx(subghz_txrx_state_t s)
{
    return s == SUBGHZ_TXRX_STATE_TX_BLOCK
        || s == SUBGHZ_TXRX_STATE_TX_ASYNC;
}

bool subghz_txrx_state_is_powered(subghz_txrx_state_t s)
{
    if ((unsigned)s >= SUBGHZ_TXRX_STATE_COUNT) return false;
    return s != SUBGHZ_TXRX_STATE_OFF;
}

const char *subghz_txrx_state_name(subghz_txrx_state_t s)
{
    switch (s) {
        case SUBGHZ_TXRX_STATE_OFF:        return "OFF";
        case SUBGHZ_TXRX_STATE_IDLE:       return "IDLE";
        case SUBGHZ_TXRX_STATE_RX_PASSIVE: return "RX_PASSIVE";
        case SUBGHZ_TXRX_STATE_RX_ACTIVE:  return "RX_ACTIVE";
        case SUBGHZ_TXRX_STATE_TX_BLOCK:   return "TX_BLOCK";
        case SUBGHZ_TXRX_STATE_TX_ASYNC:   return "TX_ASYNC";
        case SUBGHZ_TXRX_STATE_COUNT:      /* fallthrough */
        default:                            return "INVALID";
    }
}

/*----------------------------------------------------------------------------*/
/* Stateful façade                                                            */
/*----------------------------------------------------------------------------*/

void subghz_txrx_state_init(subghz_txrx_state_ctx_t *ctx)
{
    if (ctx == NULL) return;
    ctx->state = SUBGHZ_TXRX_STATE_OFF;
    ctx->illegal_transition_count = 0;
}

bool subghz_txrx_state_try_transition(subghz_txrx_state_ctx_t *ctx,
                                       subghz_txrx_state_t to)
{
    if (ctx == NULL) return false;
    if ((unsigned)to >= SUBGHZ_TXRX_STATE_COUNT) {
        ctx->illegal_transition_count++;
        return false;
    }
    if (ctx->state == to) {
        /* No-op transition — accepted, not counted. */
        return true;
    }
    if (!subghz_txrx_state_transition_is_legal(ctx->state, to)) {
        ctx->illegal_transition_count++;
        return false;
    }
    ctx->state = to;
    return true;
}

void subghz_txrx_state_force(subghz_txrx_state_ctx_t *ctx,
                              subghz_txrx_state_t to)
{
    if (ctx == NULL) return;
    if ((unsigned)to >= SUBGHZ_TXRX_STATE_COUNT) return;
    ctx->state = to;
}
