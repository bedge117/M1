/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/honeywell_wdb.c
 * Licensed under GPLv3
 *
 * m1_honeywell_wdb_decode.c
 * Honeywell wireless doorbell (RCWL300A/330A/Series 3/5/9) — 48 bits, PWM
 * te=160/320µs, bit1=320H+160L, bit0=160H+320L
 * Parity: LSB of set-bit count across first 47 bits == bit 47
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_honeywell_wdb(uint16_t p, uint16_t pulsecount)
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

        /* bit 0: short HIGH + long LOW */
        if (DURATION_DIFF(t_hi, te_short) < te_delta &&
            DURATION_DIFF(t_lo, te_long)  < te_delta)
        {
            subghz_protocol_blocks_add_bit(&decoder, 0);
        }
        /* bit 1: long HIGH + short LOW */
        else if (DURATION_DIFF(t_hi, te_long)  < te_delta &&
                 DURATION_DIFF(t_lo, te_short) < te_delta)
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
        /* Parity check: LSB of popcount(bits 47..1) must equal bit 0 */
        uint64_t data = decoder.decode_data;
        uint8_t parity = subghz_protocol_blocks_get_parity(
            data >> 1, min_bits - 1);
        if (parity == (data & 1))
        {
            SubGhzBlockGeneric generic = {0};
            generic.data           = data;
            generic.data_count_bit = decoder.decode_count_bit;
            generic.serial         = (uint32_t)((data >> 28) & 0xFFFFF);
            subghz_block_generic_commit_to_m1(&generic, p);
            return 0;
        }
    }

    return 1;
}
