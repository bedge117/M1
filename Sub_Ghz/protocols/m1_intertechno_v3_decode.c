/* See COPYING.txt for license details. */

/*
*  m1_intertechno_v3_decode.c
*  M1 sub-ghz Intertechno_V3 32-bit decoding
*  te_short=275us, te_long=1375us
*  Each bit = 4 pulses: (275H+275L+275H+1375L)=0, (275H+1375L+275H+275L)=1
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_intertechno_v3(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint16_t bits_count = 0;
    uint16_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;  /* 275 */
    te_long  = subghz_protocols_list[p].te_long;   /* 1375 */
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* Each bit: pulse[i]=hi1, pulse[i+1]=lo1, pulse[i+2]=hi2, pulse[i+3]=lo2
     * bit0: short-hi + short-lo + short-hi + long-lo
     * bit1: short-hi + long-lo + short-hi + short-lo */
    for (i = 0; i + 3 < pulsecount && bits_count < max_bits; i += 4)
    {
        uint16_t hi1 = subghz_decenc_ctl.pulse_times[i];
        uint16_t lo1 = subghz_decenc_ctl.pulse_times[i + 1];
        uint16_t hi2 = subghz_decenc_ctl.pulse_times[i + 2];
        uint16_t lo2 = subghz_decenc_ctl.pulse_times[i + 3];

        /* Both high pulses must be te_short */
        if (get_diff(hi1, te_short) >= tol_s || get_diff(hi2, te_short) >= tol_s)
            break;

        code <<= 1;

        if (get_diff(lo1, te_short) < tol_s && get_diff(lo2, te_long) < tol_l)
        {
            /* bit 0 */
        }
        else if (get_diff(lo1, te_long) < tol_l && get_diff(lo2, te_short) < tol_s)
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
