/* See COPYING.txt for license details. */

/*
*  m1_nero_sketch_decode.c
*  M1 sub-ghz Nero Sketch 40-bit decoding
*  te_short=330us, te_long=660us
*  Preamble: 47x(330H+330L), start: 1320H+330L, data: OOK PWM
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_nero_sketch(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint16_t bits_count = 0;
    uint16_t max_bits;
    uint16_t te_start; /* 4 * te_short = 1320 */
    int16_t data_start = -1;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;
    te_start = te_short * 4; /* 1320us */

    /* Scan for start bit: te_short*4 HIGH followed by te_short LOW */
    for (i = 0; i + 1 < pulsecount; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        if (get_diff(t_hi, te_start) < (tol_s * 4) && get_diff(t_lo, te_short) < tol_s)
        {
            data_start = (int16_t)(i + 2);
            break;
        }
    }

    if (data_start < 0)
        return 1;

    /* Decode OOK PWM: bit0=short+long, bit1=long+short */
    for (i = (uint16_t)data_start; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        if (get_diff(t_hi, te_short) < tol_s && get_diff(t_lo, te_long) < tol_l)
        {
            /* bit 0 */
        }
        else if (get_diff(t_hi, te_long) < tol_l && get_diff(t_lo, te_short) < tol_s)
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
