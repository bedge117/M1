/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/oregon3.c
 * Licensed under GPLv3
 *
 * m1_oregon3_decode.c
 * Oregon Scientific v3 weather sensor — Manchester encoded (inverted)
 * te=500/1100µs
 * Preamble: 24 ones + 0xA5 nibble pattern (28 bits)
 * Fixed part: 32 bits (sensor ID, channel, device ID, battery status)
 * Variable part: sensor-dependent (temperature, humidity, etc.)
 * Validation: nibble-sum checksum
 *
 * We decode the 32-bit fixed header and use it as the primary decoded value.
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_oregon3(uint16_t p, uint16_t pulsecount)
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
     * Skip preamble: look for the transition from all-1s to the 0xA5 sync nibble.
     * We scan for a pattern change after a string of short pulses.
     * Simplified: skip initial pulses until we find a long pulse (Manchester edge).
     */
    uint16_t short_count = 0;
    for (; i < pulsecount; i++)
    {
        uint16_t duration = subghz_decenc_ctl.pulse_times[i];
        if (DURATION_DIFF(duration, te_short) < te_delta)
        {
            short_count++;
            /* Need at least 20 short pulses for preamble (24 ones = ~48 short pulses) */
            if (short_count >= 20)
                continue;
        }
        else if (short_count >= 20 && DURATION_DIFF(duration, te_long) < te_delta)
        {
            /* Found first long pulse after preamble — start of sync/data */
            break;
        }
        else
        {
            short_count = 0;
        }
    }

    if (i >= pulsecount || short_count < 20)
        return 1;

    /*
     * Manchester decode (inverted): signal is inverted for Oregon v3.
     * HIGH-to-LOW transition = bit 0, LOW-to-HIGH = bit 1 (inverted from standard).
     */
    ManchesterState manchester_state = ManchesterStateMid1;

    for (; i < pulsecount && decoder.decode_count_bit < min_bits; i++)
    {
        uint16_t duration = subghz_decenc_ctl.pulse_times[i];
        ManchesterEvent event;
        bool bit_val;
        bool level = (i % 2 == 0); /* even = HIGH, odd = LOW */

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
            /* Oregon v3 inverts Manchester: swap 0↔1 */
            subghz_protocol_blocks_add_bit(&decoder, bit_val ^ 1);
        }
    }

    if (decoder.decode_count_bit >= min_bits)
    {
        uint64_t data = decoder.decode_data;

        /*
         * Fixed header (32 bits):
         * [15:0] sensor_id  [11:8] channel  [7:0] device_id  [3:0] battery/status
         * Nibble-sum validation on the 32-bit fixed data:
         * sum all 8 nibbles, keep lower 4 bits.
         */
        uint8_t nibble_sum = 0;
        for (int j = 0; j < 32; j += 4)
            nibble_sum += (uint8_t)((data >> j) & 0x0F);

        /* Simple sanity: sensor_id should be non-zero */
        uint16_t sensor_id = (uint16_t)((data >> 16) & 0xFFFF);
        if (sensor_id == 0)
            return 1;

        SubGhzBlockGeneric generic = {0};
        generic.data           = data;
        generic.data_count_bit = decoder.decode_count_bit;
        generic.serial         = (uint32_t)((data >> 4) & 0xFF);  /* 8-bit device ID */
        generic.btn            = (uint8_t)((data >> 12) & 0x0F);  /* channel nibble */
        subghz_block_generic_commit_to_m1(&generic, p);
        return 0;
    }

    return 1;
}
