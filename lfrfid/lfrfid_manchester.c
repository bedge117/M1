/* See COPYING.txt for license details. */

/*
 * Generic Manchester decoder for LF RFID protocols.
 * Follows the same approach as the EM4100 decoder but
 * parameterized for reuse across multiple protocols.
 *
 * Copyright (C) 2026 Monstatek
 */
/*************************** I N C L U D E S **********************************/
#include <stdint.h>
#include <string.h>

#include "app_freertos.h"
#include "cmsis_os.h"
#include "main.h"

#include "lfrfid.h"
#include "lfrfid_manchester.h"

/*************************** D E F I N E S ************************************/

/* Tolerance macros — same approach as EM4100 decoder */
#define MANCH_HALF_TOL   (1.0f - 0.75f)
#define MANCH_FULL_TOL   (1.0f + 0.30f)
#define MANCH_MID_TOL    (1.0f + 0.60f)

#define MANCH_IS_HALF(t, base)  \
    (((t) > ((base) * MANCH_HALF_TOL)) && ((t) < ((base) * MANCH_MID_TOL)))

#define MANCH_IS_FULL(t, base)  \
    (((t) > ((base) * MANCH_MID_TOL)) && ((t) < ((2 * (base)) * MANCH_FULL_TOL)))

#define MANCH_IS_FULLx(t, base) \
    (((t) > ((base) * (1.0f + 0.20f))) && ((t) < ((2 * (base)) * MANCH_FULL_TOL)))

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
void manch_init(manch_decoder_t *d, uint16_t half_bit_us, uint16_t frame_bits)
{
    memset(d, 0, sizeof(*d));
    d->half_bit_us = half_bit_us;
    d->frame_bits = frame_bits;
}

/*============================================================================*/
void manch_reset(manch_decoder_t *d)
{
    memset(d->frame_buffer, 0, sizeof(d->frame_buffer));
    d->bit_count = 0;
    d->edge_count = 0;
}

/*============================================================================*/
void manch_push_bit(manch_decoder_t *d, uint8_t bit)
{
    uint8_t carry = 0;
    uint8_t new_carry;
    uint8_t bytes = (d->frame_bits + 7) / 8;

    if (bytes > MANCH_MAX_FRAME_BYTES)
        bytes = MANCH_MAX_FRAME_BYTES;

    for (int i = (int)bytes - 1; i >= 0; i--) {
        new_carry = (d->frame_buffer[i] >> 7) & 1;
        d->frame_buffer[i] = (uint8_t)((d->frame_buffer[i] << 1) | carry);
        carry = new_carry;
    }

    d->frame_buffer[bytes - 1] = (uint8_t)((d->frame_buffer[bytes - 1] & 0xFE) | (bit & 1));

    if (d->bit_count < d->frame_bits)
        d->bit_count++;
}

/*============================================================================*/
/*
 * Normalize events: expand full-period pulses into two half-period pulses.
 * Returns number of normalized events written to 'out'.
 */
/*============================================================================*/
static uint8_t manch_normalize(lfrfid_evt_t *out,
                               const lfrfid_evt_t *in,
                               uint8_t count,
                               uint16_t half_bit_us)
{
    uint8_t out_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        if (MANCH_IS_FULL(in[i].t_us, half_bit_us)) {
            /* Expand to two half-period events */
            out[out_count].t_us = half_bit_us;
            out[out_count].edge = in[i].edge;
            out_count++;
            out[out_count].t_us = half_bit_us;
            out[out_count].edge = in[i].edge;
            out_count++;
        } else {
            out[out_count] = in[i];
            out_count++;
        }
    }
    return out_count;
}

/*============================================================================*/
/*
 * Extract a Manchester bit from a pair of consecutive events.
 * Returns the bit value in *out_bit.
 * Returns true if a valid Manchester pair was decoded.
 */
/*============================================================================*/
static bool manch_bit_from_pair(const lfrfid_evt_t *e1,
                                const lfrfid_evt_t *e2,
                                uint16_t half_bit_us,
                                uint8_t *out_bit)
{
    if (!MANCH_IS_HALF(e1->t_us, half_bit_us) ||
        !MANCH_IS_HALF(e2->t_us, half_bit_us))
        return false;

    uint16_t sum = (uint16_t)(e1->t_us + e2->t_us);
    if (!MANCH_IS_FULLx(sum, half_bit_us))
        return false;

    if (e1->edge == 0 && e2->edge == 1) {
        *out_bit = 1;
        return true;
    }
    if (e1->edge == 1 && e2->edge == 0) {
        *out_bit = 0;
        return true;
    }

    return false;
}

/*============================================================================*/
bool manch_feed_events(manch_decoder_t *d, const lfrfid_evt_t *events, uint8_t count)
{
    lfrfid_evt_t temp[FRAME_CHUNK_SIZE * 2];
    uint8_t norm_count;

    norm_count = manch_normalize(temp, events, count, d->half_bit_us);

    /* Check overflow */
    if (d->edge_count + norm_count > sizeof(d->edge_buffer) / sizeof(lfrfid_evt_t)) {
        manch_reset(d);
        return false;
    }

    memcpy(&d->edge_buffer[d->edge_count], temp,
           norm_count * sizeof(lfrfid_evt_t));
    d->edge_count += norm_count;

    uint8_t consumed = 0;

    while (d->edge_count - consumed >= 2) {
        lfrfid_evt_t *e1 = &d->edge_buffer[consumed];
        lfrfid_evt_t *e2 = &d->edge_buffer[consumed + 1];
        consumed += 2;

        uint8_t bit = 2;
        if (manch_bit_from_pair(e1, e2, d->half_bit_us, &bit) && bit != 2) {
            manch_push_bit(d, bit);
        } else {
            consumed -= 1;
            memset(d->frame_buffer, 0, sizeof(d->frame_buffer));
            d->bit_count = 0;
        }
    }

    /* Shift remaining unconsumed events to front */
    if (consumed > 0 && d->edge_count > consumed) {
        uint8_t remaining = d->edge_count - consumed;
        memmove(d->edge_buffer, &d->edge_buffer[consumed],
                remaining * sizeof(lfrfid_evt_t));
        d->edge_count = remaining;
    } else if (consumed > 0) {
        d->edge_count = 0;
    }

    return false; /* caller checks manch_is_full() separately */
}
