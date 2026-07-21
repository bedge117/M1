/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/acurite_592txr.c
 * Licensed under GPLv3
 *
 * m1_acurite_592txr_decode.c
 * Acurite 592TXR weather sensor — 56 bits (7 bytes), PWM
 * te=200/400µs
 * Validation: sum checksum on bytes 0-5 == byte 6, plus parity on bytes 2-5
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_acurite_592txr(uint16_t p, uint16_t pulsecount)
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
        /* bit 0 or 1 with long gap (end of packet) */
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

        /* Extract 7 bytes from the 56-bit word */
        uint8_t msg[7];
        for (int i = 0; i < 7; i++)
            msg[i] = (uint8_t)((data >> (48 - 8 * i)) & 0xFF);

        /* Validate: sum of bytes 0-5 must equal byte 6 */
        uint8_t sum = subghz_protocol_blocks_add_bytes(msg, 6);
        if (sum != msg[6])
            return 1;

        /* Validate: even parity across bytes 2-5 */
        if (subghz_protocol_blocks_parity_bytes(&msg[2], 4))
            return 1;

        SubGhzBlockGeneric generic = {0};
        generic.data           = data;
        generic.data_count_bit = decoder.decode_count_bit;
        generic.serial         = (uint32_t)((data >> 40) & 0x3FFF); /* 14-bit ID */
        subghz_block_generic_commit_to_m1(&generic, p);
        return 0;
    }

    return 1;
}
