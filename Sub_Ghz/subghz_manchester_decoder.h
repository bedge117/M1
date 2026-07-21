/* See COPYING.txt for license details. */

/*
 * subghz_manchester_decoder.h
 *
 * Flipper-compatible Manchester decoder state machine.
 *
 * Mirrors Flipper's lib/toolbox/manchester_decoder.{h,c} — provides a
 * compact, table-driven Manchester decoder that multiple Sub-GHz protocols
 * use (Marantec, Somfy Telis, Somfy Keytis, FAAC SLH, Revers RB2, etc.).
 *
 * The decoder is fed (short/long) × (high/low) events and produces decoded
 * bits.  Each call to manchester_advance() consumes one event and optionally
 * outputs one bit.
 *
 * Ported from Flipper Zero firmware:
 *   https://github.com/flipperdevices/flipperzero-firmware
 *   Original file: lib/toolbox/manchester_decoder.{h,c}
 *   Copyright (C) Flipper Devices Inc. — Licensed under GPLv3
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_MANCHESTER_DECODER_H
#define SUBGHZ_MANCHESTER_DECODER_H

#include <stdbool.h>
#include <stdint.h>

/*============================================================================*/
/* Manchester event / state enums — identical to Flipper                       */
/*============================================================================*/

typedef enum {
    ManchesterEventShortLow  = 0,
    ManchesterEventShortHigh = 2,
    ManchesterEventLongLow   = 4,
    ManchesterEventLongHigh  = 6,
    ManchesterEventReset     = 8,
} ManchesterEvent;

typedef enum {
    ManchesterStateStart1 = 0,
    ManchesterStateMid1   = 1,
    ManchesterStateMid0   = 2,
    ManchesterStateStart0 = 3,
} ManchesterState;

/*============================================================================*/
/* manchester_advance — inline, table-driven                                  */
/*                                                                            */
/* Flipper implements this in manchester_decoder.c with a static transitions  */
/* table.  We inline it here so there is no .c file to add to CMakeLists.     */
/* The transition table is 4 bytes — negligible code-size cost.               */
/*============================================================================*/

/**
 * Advance the Manchester state machine by one event.
 *
 * @param state       Current state.
 * @param event       Input event (Short/Long × High/Low, or Reset).
 * @param next_state  [out] Next state after this transition.
 * @param data        [out] Decoded bit (valid only when return is true).
 * @return true if a decoded bit was produced.
 */
static inline bool manchester_advance(
    ManchesterState  state,
    ManchesterEvent  event,
    ManchesterState *next_state,
    bool            *data)
{
    /*
     * Transition table — identical to Flipper's static const transitions[].
     *
     * Each byte encodes 4 two-bit next-states, indexed by (event >> 1):
     *   transitions[state] >> event  & 0x3  →  next_state
     *
     * Bit layout per byte:
     *   [7:6] = LongHigh(6>>1=3)  [5:4] = LongLow(4>>1=2)
     *   [3:2] = ShortHigh(2>>1=1) [1:0] = ShortLow(0>>1=0)
     */
    static const uint8_t transitions[4] = {
        0x01,  /* ManchesterStateStart1 */
        0x91,  /* ManchesterStateMid1   */
        0x9B,  /* ManchesterStateMid0   */
        0xFB,  /* ManchesterStateStart0 */
    };

    static const ManchesterState manchester_reset_state = ManchesterStateMid1;

    bool result = false;
    ManchesterState new_state;

    if (event == ManchesterEventReset) {
        new_state = manchester_reset_state;
    } else {
        new_state = (ManchesterState)((transitions[state] >> event) & 0x3);
        if (new_state == state) {
            /* Invalid transition — reset */
            new_state = manchester_reset_state;
        } else {
            if (new_state == ManchesterStateMid0) {
                if (data) *data = false;
                result = true;
            } else if (new_state == ManchesterStateMid1) {
                if (data) *data = true;
                result = true;
            }
        }
    }

    *next_state = new_state;
    return result;
}

#endif /* SUBGHZ_MANCHESTER_DECODER_H */
