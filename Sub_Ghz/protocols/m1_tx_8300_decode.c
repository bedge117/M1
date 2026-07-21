/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/tx_8300.c
 * Licensed under GPLv3
 *
 * m1_tx_8300_decode.c
 * TX-8300 weather sensor — 72+ bits, OOK variable-gap modulation
 * te=1940/3880µs
 * Message: 2 preamble bits + 32-bit data + 32-bit inverted copy + 8-bit checksum
 * Validation: package_1 == ~package_2, then Fletcher-8 checksum on the 32-bit data
 *
 * Uses 128-bit decoder to accumulate 72+ bits (exceeds 64-bit limit).
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_tx_8300(uint16_t p, uint16_t pulsecount)
{
    const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
    const uint16_t te_short = proto->timing.te_short;
    const uint16_t te_long  = proto->timing.te_long;
    const uint16_t te_delta = proto->timing.te_delta
        ? proto->timing.te_delta
        : (te_short * proto->timing.te_tolerance_pct / 100);
    const uint16_t min_bits = proto->timing.min_count_bit_for_found;

    SubGhzBlockDecoder decoder = {0};
    uint64_t head_bits = 0;

    for (uint16_t i = 0; i + 1 < pulsecount && decoder.decode_count_bit < min_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        /* All HIGH pulses should be ~ te_short */
        if (DURATION_DIFF(t_hi, te_short) >= te_delta * 2)
            break;

        /* bit 0: short gap (≈ te_short) */
        if (DURATION_DIFF(t_lo, te_short) < te_delta)
        {
            subghz_protocol_blocks_add_to_128_bit(&decoder, 0, &head_bits);
        }
        /* bit 1: long gap (≈ te_long) */
        else if (DURATION_DIFF(t_lo, te_long) < te_delta)
        {
            subghz_protocol_blocks_add_to_128_bit(&decoder, 1, &head_bits);
        }
        /* bit with long trailing gap (end of packet) */
        else if (t_lo > te_long * 3)
        {
            subghz_protocol_blocks_add_to_128_bit(&decoder, 0, &head_bits);
        }
        else
        {
            break;
        }
    }

    if (decoder.decode_count_bit >= min_bits)
    {
        /*
         * 128-bit accumulator layout (head_bits : decode_data):
         *   With exactly 72 bits captured:
         *   head_bits  = bits [71:64]  (8 bits in LSB)
         *   decode_data = bits [63:0]  (64 bits)
         *
         * Data layout (72 bits):
         *   [71:40] = package_1 (32 bits)
         *   [39:8]  = package_2 (32 bits)
         *   [7:0]   = checksum  (8 bits)
         */
        uint8_t  checksum_byte = (uint8_t)(decoder.decode_data & 0xFF);
        uint32_t package_2     = (uint32_t)((decoder.decode_data >> 8) & 0xFFFFFFFF);
        uint32_t package_1     = (uint32_t)(((head_bits & 0xFF) << 24) |
                                             (uint32_t)(decoder.decode_data >> 40));

        /* Validate: inverted copy */
        if (package_1 != (uint32_t)(~package_2))
            return 1;

        /* Validate: Fletcher-8 checksum on package_1 nibbles */
        uint16_t sum_x = 0, sum_y = 0;
        for (int i = 0; i < 32; i += 4)
        {
            sum_x += (package_1 >> i) & 0x0F;
            sum_y += (package_1 >> i) & 0x05;
        }
        uint8_t calc_crc = (uint8_t)((((~sum_x) & 0x0F) << 4) | ((~sum_y) & 0x0F));
        if (calc_crc != checksum_byte)
            return 1;

        /* Store package_1 as the primary decoded value */
        SubGhzBlockGeneric generic = {0};
        generic.data           = (uint64_t)package_1;
        generic.data_count_bit = 32;
        generic.serial         = (package_1 >> 12) & 0x7F; /* 7-bit ID */
        subghz_block_generic_commit_to_m1(&generic, p);
        return 0;
    }

    return 1;
}
