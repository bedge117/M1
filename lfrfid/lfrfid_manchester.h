/* See COPYING.txt for license details. */

/*
 * Generic Manchester decoder for LF RFID protocols.
 * Adapted from the Flipper Zero firmware approach.
 *
 * Original project:
 * https://github.com/flipperdevices/flipperzero-firmware
 *
 * Copyright (C) Flipper Devices Inc.
 * Licensed under the GNU General Public License v3.0 (GPLv3).
 *
 * Modifications and additional implementation:
 * Copyright (C) 2026 Monstatek
 */

#ifndef LFRFID_MANCHESTER_H_
#define LFRFID_MANCHESTER_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Maximum frame size for Manchester-decoded protocols */
#define MANCH_MAX_FRAME_BYTES   16
#define MANCH_MAX_FRAME_BITS    (MANCH_MAX_FRAME_BYTES * 8)

/*============================================================================*/
/* Manchester decoder state                                                   */
/*============================================================================*/
typedef struct {
    uint16_t     half_bit_us;   /* half-bit period in microseconds */
    uint16_t     frame_bits;    /* total bits in protocol frame */
    uint8_t      frame_buffer[MANCH_MAX_FRAME_BYTES];
    uint16_t     bit_count;     /* bits accumulated so far */
    lfrfid_evt_t edge_buffer[FRAME_CHUNK_SIZE];
    uint8_t      edge_count;    /* buffered edges */
} manch_decoder_t;

/*============================================================================*/
/* Initialize/reset the Manchester decoder                                    */
/*============================================================================*/
void manch_init(manch_decoder_t *d, uint16_t half_bit_us, uint16_t frame_bits);
void manch_reset(manch_decoder_t *d);

/*============================================================================*/
/* Feed raw edge events and extract decoded bits into frame_buffer.           */
/* Returns true if frame_bits have been accumulated (frame complete).         */
/*============================================================================*/
bool manch_feed_events(manch_decoder_t *d, const lfrfid_evt_t *events, uint8_t count);

/*============================================================================*/
/* Push a single bit into the frame buffer (MSB shift register)               */
/*============================================================================*/
void manch_push_bit(manch_decoder_t *d, uint8_t bit);

/*============================================================================*/
/* Get bit from frame buffer at given position                                */
/*============================================================================*/
static inline uint8_t manch_get_bit(const manch_decoder_t *d, uint16_t pos)
{
    return (d->frame_buffer[pos / 8] >> (7 - (pos % 8))) & 1;
}

/*============================================================================*/
/* Check if frame is full                                                     */
/*============================================================================*/
static inline bool manch_is_full(const manch_decoder_t *d)
{
    return d && (d->bit_count >= d->frame_bits);
}

#endif /* LFRFID_MANCHESTER_H_ */
