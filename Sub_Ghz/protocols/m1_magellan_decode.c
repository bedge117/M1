/* See COPYING.txt for license details. */

/*
 * m1_magellan_decode.c
 *
 * M1 sub-ghz Magellan decoder
 * te_short=200µs, te_long=400µs, fixed 32-bit payload.
 *
 * BUG FIX: The original decoder delegated to subghz_decode_generic_pwm()
 * which uses STANDARD OOK PWM polarity:
 *   bit 0: SHORT HIGH + LONG LOW
 *   bit 1: LONG  HIGH + SHORT LOW
 *
 * Magellan uses INVERTED polarity relative to standard OOK PWM:
 *   bit 1: SHORT HIGH (200µs) + LONG  LOW (400µs)
 *   bit 0: LONG  HIGH (400µs) + SHORT LOW (200µs)
 *
 * The generic decoder also ignores the Magellan frame header that precedes
 * the 32 data bits.  The on-air Magellan frame is:
 *   1 × header burst  : 800µs HIGH + 200µs LOW
 *   12 × toggle pairs : 200µs HIGH + 200µs LOW (each)
 *   1 × header end    : 200µs HIGH + 400µs LOW
 *   1 × start bit     : 1200µs HIGH + 400µs LOW   ← key landmark
 *   32 × data bits
 *   1 × stop bit      : 200µs HIGH + 40000µs LOW
 *
 * This decoder scans for the 1200µs start bit HIGH pulse and begins
 * decoding data from the following pair, using inverted polarity.
 */

#include "m1_sub_ghz_decenc.h"

/* Fixed Magellan timings (µs) — derived from the Momentum firmware reference
 * implementation (Next-Flip/Momentum-Firmware lib/subghz/protocols/magellan.c):
 *   te_short = 200, te_long = 400, te_delta = 100
 * The delta (100µs) is applied uniformly to both SHORT and LONG pulse comparisons,
 * exactly matching the Momentum reference decoder. */
#define MAG_TE_SHORT        200u
#define MAG_TE_LONG         400u
#define MAG_TE_DELTA        100u   /* pulse timing tolerance — matches Momentum te_delta */

/* Start bit: 3 × TE_LONG = 1200µs HIGH + TE_LONG LOW.
 * Momentum uses te_delta*3 = 300µs tolerance on the HIGH phase → [900, 1500]µs,
 * and te_delta*2 = 200µs tolerance on the LOW phase → [200, 600]µs.
 * Both phases must match to avoid false starts on noisy captures. */
#define MAG_START_HI_MIN    900u
#define MAG_START_HI_MAX   1500u
#define MAG_START_LO_TOL   (MAG_TE_DELTA * 2u)   /* = 200µs; LOW must be ≈ te_long */

#define MAG_DATA_BITS       32u

uint8_t subghz_decode_magellan(uint16_t p, uint16_t pulsecount)
{
    uint16_t i;
    uint16_t data_start = 0; /* index of first data HIGH pulse */

    /* Step 1 — find the start bit by scanning even (HIGH-pulse) indices.
     * Momentum reference: start bit is HIGH ≈ te_short*6 = 1200µs (±300µs)
     * followed by LOW ≈ te_long = 400µs (±200µs).  Both phases must match to
     * guard against false starts on noisy or truncated captures.
     * Fallback: if no start bit is found the decoder starts from index 0
     * (supports bare-data test buffers with no frame header). */
    for (i = 0; i + 1 < pulsecount; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];
        if (t_hi >= MAG_START_HI_MIN && t_hi <= MAG_START_HI_MAX &&
            get_diff(t_lo, MAG_TE_LONG) < MAG_START_LO_TOL)
        {
            data_start = i + 2; /* skip start-bit HIGH + LOW */
            break;
        }
    }

    /* Step 2 — decode 32 data bits with INVERTED Magellan polarity:
     *   bit 1: SHORT HIGH (≈200µs) + LONG  LOW (≈400µs)   [Momentum: te_short + te_long]
     *   bit 0: LONG  HIGH (≈400µs) + SHORT LOW (≈200µs)   [Momentum: te_long  + te_short]
     * Both HIGH and LOW phases are checked within MAG_TE_DELTA = 100µs (Momentum te_delta). */
    uint32_t code = 0;
    uint8_t  bits_count = 0;

    for (i = data_start; i + 1 < pulsecount && bits_count < MAG_DATA_BITS; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        if (get_diff(t_hi, MAG_TE_SHORT) < MAG_TE_DELTA &&
            get_diff(t_lo, MAG_TE_LONG)  < MAG_TE_DELTA)
        {
            code |= 1; /* bit 1: SHORT HIGH + LONG LOW (inverted Magellan) */
        }
        else if (get_diff(t_hi, MAG_TE_LONG)  < MAG_TE_DELTA &&
                 get_diff(t_lo, MAG_TE_SHORT) < MAG_TE_DELTA)
        {
            /* bit 0: LONG HIGH + SHORT LOW */
        }
        else
        {
            break; /* invalid pulse — abort */
        }
        bits_count++;
    }

    if (bits_count >= MAG_DATA_BITS)
    {
        subghz_decenc_ctl.n64_decodedvalue  = code;
        subghz_decenc_ctl.ndecodedbitlength = bits_count;
        subghz_decenc_ctl.ndecodeddelay     = 0;
        subghz_decenc_ctl.ndecodedprotocol  = p;
        return 0; /* success */
    }

    return 1; /* decode failed */
}
