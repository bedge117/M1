/* See COPYING.txt for license details. */

/*
*  m1_hollarm_decode.c
*  M1 sub-ghz Hollarm 42-bit decoding (PPM)
*  te_short=200us, te_long=1000us
*  bit0=200H+1000L, bit1=200H+1600L
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_hollarm(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint16_t bits_count = 0;
    uint16_t max_bits;
    uint16_t te_bit1; /* 8*te_short = 1600 */

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;   /* 200 */
    te_long  = subghz_protocols_list[p].te_long;    /* 1000 */
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;
    te_bit1 = te_short * 8; /* 1600us */

    /* Bits arrive as (HIGH=te_short, LOW=variable) pairs */
    for (i = 0; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        if (get_diff(t_hi, te_short) >= tol_s)
            break;

        code <<= 1;

        if (get_diff(t_lo, te_long) < tol_l)
        {
            /* bit 0 */
        }
        else if (get_diff(t_lo, te_bit1) < tol_l)
        {
            code |= 1; /* bit 1 */
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
