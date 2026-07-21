/* See COPYING.txt for license details. */

/*
*  m1_dickert_mahs_decode.c
*  M1 sub-ghz Dickert_MAHS 36-bit decoding
*  te_short=400us, bit=(LOW,HIGH) pairs: bit1=800L+400H, bit0=400L+800H
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_dickert_mahs(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol;
    uint16_t i;
    uint16_t bits_count = 0;
    uint16_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol = (te_short * subghz_protocols_list[p].te_tolerance) / 100;

    /* Skip pulse[0]: start bit HIGH */
    i = 1;

    /* Decode (LOW, HIGH) pairs */
    for (; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        if (get_diff(t_lo, te_short) < tol && get_diff(t_hi, te_long) < tol)
        {
            /* bit 0: short-low + long-high */
        }
        else if (get_diff(t_lo, te_long) < tol && get_diff(t_hi, te_short) < tol)
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
