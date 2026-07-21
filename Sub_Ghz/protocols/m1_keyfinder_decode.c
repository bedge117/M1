/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/keyfinder.c
 * Licensed under GPLv3
 *
 * m1_keyfinder_decode.c
 * KeyFinder tag — 24 bits, inverted PWM
 * te=400/1200µs, bit1=400H+1200L, bit0=1200H+400L
 * GAP ≈ 400*10 = 4000µs between repetitions
 */
#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_keyfinder(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol;
    uint16_t bits_count = 0;
    uint16_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    if (tol < 150) tol = 150;

    for (uint16_t i = 0; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        /* bit 1: short HIGH + long LOW */
        if (get_diff(t_hi, te_short) < tol &&
            get_diff(t_lo, te_long) < tol)
        {
            code |= 1;
        }
        /* bit 0: long HIGH + short LOW */
        else if (get_diff(t_hi, te_long) < tol &&
                 get_diff(t_lo, te_short) < tol)
        {
            /* bit 0 */
        }
        else
        {
            break;
        }
        bits_count++;
    }

    if (bits_count >= max_bits)
    {
        subghz_decenc_ctl.n64_decodedvalue = code;
        subghz_decenc_ctl.ndecodedbitlength = bits_count;
        subghz_decenc_ctl.ndecodeddelay = 0;
        subghz_decenc_ctl.ndecodedprotocol = p;
        return 0;
    }

    return 1;
}
