/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/x10.c
 * Licensed under GPLv3
 *
 * m1_x10_decode.c
 * X10 home automation — 32 bits, PWM with 16x+8x preamble
 * te=600/1800µs
 * Validation: bytes 0^1 == 0xFF and bytes 2^3 == 0xFF
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_x10(uint16_t p, uint16_t pulsecount)
{
    const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
    const uint16_t te_short = proto->timing.te_short;
    const uint16_t te_long  = proto->timing.te_long;
    const uint16_t te_delta = proto->timing.te_delta
        ? proto->timing.te_delta
        : (te_short * proto->timing.te_tolerance_pct / 100);
    const uint16_t min_bits = proto->timing.min_count_bit_for_found;

    SubGhzBlockDecoder decoder = {0};
    uint16_t i = 0;

    /* Skip preamble: look for 9600µs HIGH (16*te_short) + 4800µs LOW (8*te_short) */
    for (; i + 1 < pulsecount; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        if (DURATION_DIFF(t_hi, te_short * 16) < te_delta * 7 &&
            DURATION_DIFF(t_lo, te_short * 8)  < te_delta * 5)
        {
            i += 2;  /* advance past preamble */
            break;
        }
    }

    /* Decode data bits: each bit = te_short HIGH + (te_short or te_long) LOW */
    for (; i + 1 < pulsecount && decoder.decode_count_bit < min_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        if (DURATION_DIFF(t_hi, te_short) >= te_delta)
            break;

        if (DURATION_DIFF(t_lo, te_short) < te_delta)
        {
            subghz_protocol_blocks_add_bit(&decoder, 0);
        }
        else if (DURATION_DIFF(t_lo, te_long) < te_delta * 2)
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
        uint64_t data = decoder.decode_data;
        /* Validate: byte0 ^ byte1 == 0xFF and byte2 ^ byte3 == 0xFF */
        if ((((data >> 24) ^ (data >> 16)) & 0xFF) == 0xFF &&
            (((data >> 8)  ^ data)         & 0xFF) == 0xFF)
        {
            SubGhzBlockGeneric generic = {0};
            generic.data           = data;
            generic.data_count_bit = decoder.decode_count_bit;
            generic.serial         = (uint32_t)((data >> 28) & 0x0F);
            generic.btn            = (uint8_t)(((data >> 24) & 0x07) | ((data >> 8) & 0xF8));
            subghz_block_generic_commit_to_m1(&generic, p);
            return 0;
        }
    }

    return 1;
}
