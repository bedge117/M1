/* See COPYING.txt for license details. */

/*
 *  m1_generic_decode.c
 *
 *  Generic sub-ghz decoders for common encoding schemes.
 *  Protocols with standard OOK PWM encoding (1:2 or 1:3 ratio)
 *  or Manchester encoding can call these instead of duplicating logic.
 *
 *  Uses Flipper-compatible building blocks:
 *    - SubGhzBlockDecoder   (bit accumulation, state tracking)
 *    - SubGhzBlockGeneric   (decoded output + commit_to_m1 bridge)
 *    - DURATION_DIFF        (branchless absolute difference)
 *    - manchester_advance   (table-driven Manchester state machine)
 *  All included via subghz_protocol_registry.h.
 */

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"   /* includes all block headers */
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_GENERIC"

/*
 * Generic OOK PWM decoder (works for both 1:2 and 1:3 ratio).
 * Pulse pairs: short-high + long-low = bit 0, long-high + short-low = bit 1.
 * Reads timing from the protocol registry (SubGhzBlockConst).
 * Uses SubGhzBlockDecoder for bit accumulation and SubGhzBlockGeneric +
 * commit_to_m1() for output — demonstrating the Flipper-compatible pattern.
 * Returns 0 on success, 1 on failure.
 */
uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount)
{
    const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
    const uint16_t te_short = proto->timing.te_short;
    const uint16_t te_long  = proto->timing.te_long;
    const uint16_t te_delta = proto->timing.te_delta
        ? proto->timing.te_delta
        : (te_short * proto->timing.te_tolerance_pct / 100);
    const uint16_t min_bits = proto->timing.min_count_bit_for_found;
    const uint8_t  preamble = proto->timing.preamble_bits;

    SubGhzBlockDecoder decoder = {0};

    for (uint16_t i = preamble; i + 1 < pulsecount && decoder.decode_count_bit < min_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        if (DURATION_DIFF(t_hi, te_short) < te_delta &&
            DURATION_DIFF(t_lo, te_long)  < te_delta)
        {
            subghz_protocol_blocks_add_bit(&decoder, 0);
        }
        else if (DURATION_DIFF(t_hi, te_long)  < te_delta &&
                 DURATION_DIFF(t_lo, te_short) < te_delta)
        {
            subghz_protocol_blocks_add_bit(&decoder, 1);
        }
        else
        {
            break; /* invalid pulse pair */
        }
    }

    if (decoder.decode_count_bit >= min_bits)
    {
        SubGhzBlockGeneric generic = {0};
        generic.data           = decoder.decode_data;
        generic.data_count_bit = decoder.decode_count_bit;
        subghz_block_generic_commit_to_m1(&generic, p);
        return 0;
    }

    return 1;
}

/*
 * Generic Manchester decoder.
 * Uses Flipper's manchester_advance() state machine for decoding.
 * Reads timing from the protocol registry (SubGhzBlockConst).
 * Returns 0 on success, 1 on failure.
 */
uint8_t subghz_decode_generic_manchester(uint16_t p, uint16_t pulsecount)
{
    const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
    const uint16_t te_short = proto->timing.te_short;
    const uint16_t te_long  = proto->timing.te_long;
    const uint16_t te_delta = proto->timing.te_delta
        ? proto->timing.te_delta
        : (te_short * proto->timing.te_tolerance_pct / 100);
    const uint16_t min_bits = proto->timing.min_count_bit_for_found;
    const uint8_t  preamble = proto->timing.preamble_bits;

    SubGhzBlockDecoder decoder = {0};
    ManchesterState manchester_state = ManchesterStateMid1;

    for (uint16_t i = preamble; i < pulsecount && decoder.decode_count_bit < min_bits; i++)
    {
        uint16_t dur = subghz_decenc_ctl.pulse_times[i];

        /* Classify pulse as short or long, and determine level.
         * Odd indices (0-based) are LOW (spaces), even are HIGH (marks). */
        bool is_high = ((i & 1) == 0);

        ManchesterEvent event;
        if (DURATION_DIFF(dur, te_short) < te_delta)
        {
            event = is_high ? ManchesterEventShortHigh : ManchesterEventShortLow;
        }
        else if (DURATION_DIFF(dur, te_long) < te_delta)
        {
            event = is_high ? ManchesterEventLongHigh : ManchesterEventLongLow;
        }
        else
        {
            /* Invalid duration — reset state machine and try to continue */
            manchester_advance(manchester_state, ManchesterEventReset,
                               &manchester_state, NULL);
            continue;
        }

        bool bit_value;
        if (manchester_advance(manchester_state, event, &manchester_state, &bit_value))
        {
            subghz_protocol_blocks_add_bit(&decoder, bit_value ? 1 : 0);
        }
    }

    if (decoder.decode_count_bit >= min_bits)
    {
        SubGhzBlockGeneric generic = {0};
        generic.data           = decoder.decode_data;
        generic.data_count_bit = decoder.decode_count_bit;
        subghz_block_generic_commit_to_m1(&generic, p);
        return 0;
    }

    return 1;
}

/*
 * Generic PPM decoder (Pulse Position Modulation).
 * Constant short HIGH pulse, bit encoded in LOW gap duration.
 * bit 0: te_short HIGH + te_long LOW
 * bit 1: te_short HIGH + (te_long * 2) LOW
 * Registry timing: te_short = HIGH pulse, te_long = bit-0 gap.
 * Uses SubGhzBlockDecoder for bit accumulation and SubGhzBlockGeneric +
 * commit_to_m1() for output.
 * Returns 0 on success, 1 on failure.
 */
uint8_t subghz_decode_generic_ppm(uint16_t p, uint16_t pulsecount)
{
    const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
    const uint16_t te_short = proto->timing.te_short;
    const uint16_t te_long  = proto->timing.te_long;
    const uint16_t te_delta = proto->timing.te_delta
        ? proto->timing.te_delta
        : (te_short * proto->timing.te_tolerance_pct / 100);
    const uint16_t min_bits = proto->timing.min_count_bit_for_found;
    const uint8_t  preamble = proto->timing.preamble_bits;

    SubGhzBlockDecoder decoder = {0};

    for (uint16_t i = preamble; i + 1 < pulsecount && decoder.decode_count_bit < min_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        /* All HIGHs must be approximately te_short */
        if (DURATION_DIFF(t_hi, te_short) >= te_delta * 2)
            break;

        /* bit 0: short gap ≈ te_long */
        if (DURATION_DIFF(t_lo, te_long) < te_delta)
        {
            subghz_protocol_blocks_add_bit(&decoder, 0);
        }
        /* bit 1: long gap ≈ te_long * 2 */
        else if (DURATION_DIFF(t_lo, te_long * 2) < te_delta * 2)
        {
            subghz_protocol_blocks_add_bit(&decoder, 1);
        }
        else
        {
            break;
        }
    }

    if (decoder.decode_count_bit >= min_bits)
    {
        SubGhzBlockGeneric generic = {0};
        generic.data           = decoder.decode_data;
        generic.data_count_bit = decoder.decode_count_bit;
        subghz_block_generic_commit_to_m1(&generic, p);
        return 0;
    }

    return 1;
}
