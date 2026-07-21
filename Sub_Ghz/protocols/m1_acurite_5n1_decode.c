/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/acurite_5n1.c
 * Licensed under GPLv3
 *
 * m1_acurite_5n1_decode.c
 * Acurite 5-in-1 weather station — 64 bits (8 bytes), OOK PWM
 * te_short=200µs, te_long=400µs, te_delta=90µs
 * Frequency: 433.92 MHz
 *
 * Two alternating message types:
 *   0x31 — wind direction + wind speed + rain counter
 *   0x38 — wind speed + temperature + humidity
 *
 * Packet layout (8 bytes = 64 bits):
 *   byte 0: battery(1) | 1(1) | 1(1) | 1(1) | channel(2) | winddir[8:7](2)
 *   byte 1: winddir[6:2](5) | windspd_hi(3)
 *   byte 2: windspd_lo(8)
 *   byte 3: msg_type(8)   -- 0x31 or 0x38
 *   byte 4: data_hi(8)    -- rain_hi / temp_hi
 *   byte 5: data_lo(8)    -- rain_lo / temp_lo+hum(4)
 *   byte 6: humidity(8)   -- only valid for 0x38 messages
 *   byte 7: checksum(8)   -- sum of bytes 0-6, low 7 bits (bit[7] = status)
 *
 * Validation: (sum of bytes 0-6) & 0x7F == byte[7] & 0x7F
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_acurite_5n1(uint16_t p, uint16_t pulsecount)
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

    if (decoder.decode_count_bit < min_bits)
        return 1;

    uint64_t data = decoder.decode_data;

    /* Extract 8 bytes from the 64-bit word (MSB first) */
    uint8_t msg[8];
    for (int i = 0; i < 8; i++)
        msg[i] = (uint8_t)((data >> (56 - 8 * i)) & 0xFF);

    /* Validate: sum of bytes 0-6 (low 7 bits) must equal byte 7 (low 7 bits) */
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++)
        sum += msg[i];
    if ((sum & 0x7F) != (msg[7] & 0x7F))
        return 1;

    /* Serial: upper 4 bits of byte 0 (battery + fixed 0b11) + channel bits */
    uint32_t serial = ((uint32_t)msg[0] >> 4) | (((uint32_t)msg[0] & 0x0C) << 4);

    SubGhzBlockGeneric generic = {0};
    generic.data           = data;
    generic.data_count_bit = decoder.decode_count_bit;
    generic.serial         = serial;
    subghz_block_generic_commit_to_m1(&generic, p);
    return 0;
}
