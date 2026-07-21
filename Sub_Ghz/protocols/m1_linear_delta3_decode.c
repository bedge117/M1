/* See COPYING.txt for license details. */

/*
*  m1_linear_delta3_decode.c
*  M1 sub-ghz LinearDelta3 8-bit decoding
*  te_short=500us, te_long=2000us
*  bit1=500H+3500L(7x500), bit0=2000H+2000L
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_linear_delta3(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint16_t bits_count = 0;
    uint16_t max_bits;
    uint16_t te_long_lo; /* 7 * te_short = 3500 */

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;   /* 500 */
    te_long  = subghz_protocols_list[p].te_long;    /* 2000 */
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;
    te_long_lo = te_short * 7; /* 3500us */

    /* Data bits as (HIGH, LOW) pairs:
     * bit 0: 2000H + 2000L
     * bit 1: 500H + 3500L */
    for (i = 0; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        if (get_diff(t_hi, te_long) < tol_l && get_diff(t_lo, te_long) < tol_l)
        {
            /* bit 0: long+long */
        }
        else if (get_diff(t_hi, te_short) < tol_s && get_diff(t_lo, te_long_lo) < (tol_s * 3))
        {
            code |= 1; /* bit 1: short + 7*short */
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
