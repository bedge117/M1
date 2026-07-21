/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/oregon_v1.c
 * Licensed under GPLv3
 *
 * m1_oregon_v1_decode.c
 * Oregon Scientific v1 weather sensor — 32 bits, Manchester encoded
 * te=1465/2930µs
 * Preamble: 12 alternating bits, then sync pattern (4200+5780+5200µs)
 * Validation: byte-sum checksum (sum of bytes 0-2 with carry == byte 3)
 * Data is bit-reversed after decode.
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

/* Reverse bits in a 32-bit word */
static uint32_t reverse_bits_32(uint32_t val)
{
    val = ((val & 0xFFFF0000) >> 16) | ((val & 0x0000FFFF) << 16);
    val = ((val & 0xFF00FF00) >>  8) | ((val & 0x00FF00FF) <<  8);
    val = ((val & 0xF0F0F0F0) >>  4) | ((val & 0x0F0F0F0F) <<  4);
    val = ((val & 0xCCCCCCCC) >>  2) | ((val & 0x33333333) <<  2);
    val = ((val & 0xAAAAAAAA) >>  1) | ((val & 0x55555555) <<  1);
    return val;
}

uint8_t subghz_decode_oregon_v1(uint16_t p, uint16_t pulsecount)
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

    /*
     * Look for sync pattern:
     * ~4200µs gap (≈ 2.87 × te_short) + ~5780µs pulse (≈ 3.94 × te_short)
     * + ~5200µs gap (≈ 3.55 × te_short)
     * We look for a long pulse ≈ 4×te_short followed by a long gap.
     */
    for (; i + 1 < pulsecount; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        /* Sync pulse: ~5780µs HIGH */
        if (DURATION_DIFF(t_hi, te_short * 4) < te_delta * 4 &&
            DURATION_DIFF(t_lo, te_short * 3) < te_delta * 4)
        {
            i += 2;
            break;
        }
    }

    if (i >= pulsecount)
        return 1;

    /*
     * Manchester decode: each bit is encoded as two half-periods.
     * For Oregon v1, transitions at the middle of a bit period encode the data.
     * We use a simplified approach: look at pulse pairs.
     */
    ManchesterState manchester_state = ManchesterStateMid1;

    for (; i < pulsecount && decoder.decode_count_bit < min_bits; i++)
    {
        uint16_t duration = subghz_decenc_ctl.pulse_times[i];
        ManchesterEvent event;
        bool bit_val;
        bool level = (i % 2 == 0); /* even index = HIGH, odd = LOW */

        if (DURATION_DIFF(duration, te_short) < te_delta)
        {
            event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        }
        else if (DURATION_DIFF(duration, te_long) < te_delta)
        {
            event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        }
        else
        {
            break;
        }

        if (manchester_advance(manchester_state, event, &manchester_state, &bit_val))
        {
            /* Oregon V1 inverts the Manchester output */
            subghz_protocol_blocks_add_bit(&decoder, bit_val ^ 1);
        }
    }

    if (decoder.decode_count_bit >= min_bits)
    {
        uint64_t raw = decoder.decode_data;

        /* Reverse all 32 bits (Oregon v1 transmits LSB-first conceptually) */
        uint32_t data = reverse_bits_32((uint32_t)(raw & 0xFFFFFFFF));

        /* Checksum: sum bytes 0-2 (with carry folding) must equal byte 3 */
        uint16_t crc = (data & 0xFF) + ((data >> 8) & 0xFF) + ((data >> 16) & 0xFF);
        crc = (crc & 0xFF) + ((crc >> 8) & 0xFF);
        if ((uint8_t)crc != (uint8_t)((data >> 24) & 0xFF))
            return 1;

        SubGhzBlockGeneric generic = {0};
        generic.data           = (uint64_t)data;
        generic.data_count_bit = 32;
        generic.serial         = data & 0xFF; /* 8-bit device ID */
        subghz_block_generic_commit_to_m1(&generic, p);
        return 0;
    }

    return 1;
}
