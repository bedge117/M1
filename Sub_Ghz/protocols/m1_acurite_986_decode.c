/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/acurite_986.c
 * Licensed under GPLv3
 *
 * m1_acurite_986_decode.c
 * Acurite 986 refrigerator/freezer sensor — 40 bits (5 bytes), PWM
 * te=800/1750µs
 * Bits transmitted LSB-first (byte-level bit reversal required).
 * Validation: CRC-8 polynomial 0x07, initial 0x00 on bytes 0-3 == byte 4
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

/* Reverse bits within a byte */
static uint8_t reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

uint8_t subghz_decode_acurite_986(uint16_t p, uint16_t pulsecount)
{
    const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
    const uint16_t te_short = proto->timing.te_short;
    const uint16_t te_long  = proto->timing.te_long;
    const uint16_t te_delta = proto->timing.te_delta
        ? proto->timing.te_delta
        : (te_short * proto->timing.te_tolerance_pct / 100);
    const uint16_t min_bits = proto->timing.min_count_bit_for_found;

    SubGhzBlockDecoder decoder = {0};

    for (uint16_t i = 0; i + 1 < pulsecount && decoder.decode_count_bit < min_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        /* bit 1: long HIGH + short LOW */
        if (DURATION_DIFF(t_hi, te_long)  < te_delta &&
            DURATION_DIFF(t_lo, te_short) < te_delta)
        {
            subghz_protocol_blocks_add_bit(&decoder, 1);
        }
        /* bit 0: short HIGH + long LOW */
        else if (DURATION_DIFF(t_hi, te_short) < te_delta &&
                 DURATION_DIFF(t_lo, te_long)  < te_delta)
        {
            subghz_protocol_blocks_add_bit(&decoder, 0);
        }
        /* Handle gap after last bit */
        else if (DURATION_DIFF(t_hi, te_short) < te_delta && t_lo > te_long * 3)
        {
            subghz_protocol_blocks_add_bit(&decoder, 0);
        }
        else if (DURATION_DIFF(t_hi, te_long) < te_delta && t_lo > te_long * 3)
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

        /* Extract 5 bytes, reversing each byte (LSB-first transmission) */
        uint8_t msg[5];
        for (int i = 0; i < 5; i++)
            msg[i] = reverse_byte((uint8_t)((data >> (32 - 8 * i)) & 0xFF));

        /* Validate: CRC-8 poly 0x07 init 0x00 on bytes 0-3 must equal byte 4 */
        uint8_t crc = subghz_protocol_blocks_crc8(msg, 4, 0x07, 0x00);
        if (crc != msg[4])
            return 1;

        SubGhzBlockGeneric generic = {0};
        generic.data           = data;
        generic.data_count_bit = decoder.decode_count_bit;
        generic.serial         = ((uint32_t)msg[1] << 8) | (uint32_t)msg[2]; /* 16-bit ID */
        subghz_block_generic_commit_to_m1(&generic, p);
        return 0;
    }

    return 1;
}
