/* See COPYING.txt for license details. */

/*
 * subghz_manchester_encoder.h
 *
 * Flipper-compatible Manchester encoder state machine.
 *
 * Mirrors Flipper's lib/toolbox/manchester_encoder.{h,c} — converts a
 * sequence of data bits into Manchester-coded half-bit symbols
 * (short/long × high/low), suitable for generating a transmit waveform.
 *
 * Usage:
 *   1. Call manchester_encoder_reset() before encoding.
 *   2. For each data bit, call manchester_encoder_advance().
 *      - If it returns true, the bit was consumed and *result holds the symbol.
 *      - If it returns false, call again with the SAME bit (the encoder
 *        needed an extra half-bit transition before consuming).
 *   3. After the last bit, call manchester_encoder_finish() to get the
 *      trailing symbol.
 *
 * Ported from Flipper Zero firmware:
 *   https://github.com/flipperdevices/flipperzero-firmware
 *   Original file: lib/toolbox/manchester_encoder.{h,c}
 *   Copyright (C) Flipper Devices Inc. — Licensed under GPLv3
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_MANCHESTER_ENCODER_H
#define SUBGHZ_MANCHESTER_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Encoder result enum — identical to Flipper                                 */
/*============================================================================*/

typedef enum {
    ManchesterEncoderResultShortLow  = 0b00,
    ManchesterEncoderResultLongLow   = 0b01,
    ManchesterEncoderResultLongHigh  = 0b10,
    ManchesterEncoderResultShortHigh = 0b11,
} ManchesterEncoderResult;

/*============================================================================*/
/* Encoder state                                                              */
/*============================================================================*/

typedef struct {
    bool    prev_bit;
    uint8_t step;
} ManchesterEncoderState;

/*============================================================================*/
/* API — all inline (no .c file needed)                                       */
/*============================================================================*/

/** Reset the encoder to its initial state. */
static inline void manchester_encoder_reset(ManchesterEncoderState *state)
{
    state->step     = 0;
    state->prev_bit = false;
}

/**
 * Advance the encoder by one data bit.
 *
 * @param state     Encoder state (modified in place).
 * @param curr_bit  The data bit to encode.
 * @param result    [out] The Manchester symbol produced.
 * @return true if the data bit was consumed (advance to the next bit).
 *         false if the encoder emitted a transition symbol and needs
 *         to be called again with the same bit.
 */
static inline bool manchester_encoder_advance(
    ManchesterEncoderState  *state,
    const bool               curr_bit,
    ManchesterEncoderResult *result)
{
    bool advance = false;

    switch (state->step) {
    case 0:
        /* First bit — emit opening half-symbol */
        state->prev_bit = curr_bit;
        *result = curr_bit ? ManchesterEncoderResultShortLow
                           : ManchesterEncoderResultShortHigh;
        state->step = 1;
        advance = true;
        break;

    case 1:
        /* Normal bit — combine with previous */
        *result = (ManchesterEncoderResult)((state->prev_bit << 1) | curr_bit);
        if (curr_bit == state->prev_bit) {
            /* Same-value transition: need an extra half-bit next round */
            state->step = 2;
        } else {
            state->prev_bit = curr_bit;
            advance = true;
        }
        break;

    case 2:
        /* Extra half-bit after a same-value pair */
        *result = curr_bit ? ManchesterEncoderResultShortLow
                           : ManchesterEncoderResultShortHigh;
        state->prev_bit = curr_bit;
        state->step = 1;
        advance = true;
        break;

    default:
        /* Should not happen — reset and retry */
        state->step = 0;
        advance = false;
        break;
    }

    return advance;
}

/**
 * Finish encoding — emit the trailing half-symbol.
 *
 * Call after the last data bit has been consumed.
 *
 * @param state  Encoder state.
 * @return The final Manchester symbol.
 */
static inline ManchesterEncoderResult manchester_encoder_finish(
    ManchesterEncoderState *state)
{
    state->step = 0;
    return (ManchesterEncoderResult)((state->prev_bit << 1) | state->prev_bit);
}

#endif /* SUBGHZ_MANCHESTER_ENCODER_H */
