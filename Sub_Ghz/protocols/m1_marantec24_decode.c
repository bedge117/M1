/* See COPYING.txt for license details. */

/*
*  m1_marantec24_decode.c
*  M1 sub-ghz Marantec24 24-bit decoding
*  te_short=800us, te_long=1600us
*  bit0=1600H+2400L(3x800), bit1=800H+3200L(2x1600)
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_marantec24(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint16_t bits_count = 0;
    uint16_t max_bits;
    uint16_t te_lo_0;  /* 3 * te_short = 2400 */
    uint16_t te_lo_1;  /* 2 * te_long  = 3200 */

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;  /* 800 */
    te_long  = subghz_protocols_list[p].te_long;   /* 1600 */
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;
    te_lo_0 = te_short * 3;   /* 2400 */
    te_lo_1 = te_long  * 2;   /* 3200 */

    /* Data bits as (HIGH, LOW) pairs:
     * bit 0: te_long HIGH + 3*te_short LOW
     * bit 1: te_short HIGH + 2*te_long LOW */
    for (i = 0; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        if (get_diff(t_hi, te_long) < tol_l && get_diff(t_lo, te_lo_0) < (tol_s * 2))
        {
            /* bit 0 */
        }
        else if (get_diff(t_hi, te_short) < tol_s && get_diff(t_lo, te_lo_1) < (tol_l * 2))
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
