/* See COPYING.txt for license details. */

/*
*  m1_holtek_base_decode.c
*  M1 sub-ghz Holtek 40-bit decoding
*  te_short=430us, te_long=870us
*  After start bit: (LOW,HIGH) pairs: bit0=430L+870H, bit1=870L+430H
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_holtek_base(uint16_t p, uint16_t pulsecount)
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

    /* pulse[0] = start bit HIGH; data starts at pulse[1] as (LOW,HIGH) pairs */
    i = 1;

    for (; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        if (get_diff(t_lo, te_short) < tol_s && get_diff(t_hi, te_long) < tol_l)
        {
            /* bit 0: short-low + long-high */
        }
        else if (get_diff(t_lo, te_long) < tol_l && get_diff(t_hi, te_short) < tol_s)
        {
            code |= 1; /* bit 1: long-low + short-high */
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
