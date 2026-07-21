/* See COPYING.txt for license details. */

/*
*  m1_legrand_decode.c
*  M1 sub-ghz Legrand 18-bit decoding
*  te_short=375us, te_long=1125us
*  Inverted PWM: bit0=1125L+375H, bit1=375L+1125H
*  Note: Legrand requires two consecutive matching packets; M1 returns match on first.
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_legrand(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint16_t bits_count = 0;
    uint16_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* Pairs arrive as (LOW, HIGH):
     * bit 0: long-LOW + short-HIGH
     * bit 1: short-LOW + long-HIGH */
    for (i = 0; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        if (get_diff(t_lo, te_long) < tol_l && get_diff(t_hi, te_short) < tol_s)
        {
            /* bit 0: long-LOW + short-HIGH */
        }
        else if (get_diff(t_lo, te_short) < tol_s && get_diff(t_hi, te_long) < tol_l)
        {
            code |= 1; /* bit 1: short-LOW + long-HIGH */
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
