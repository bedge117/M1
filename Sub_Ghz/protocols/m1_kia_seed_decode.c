/* See COPYING.txt for license details. */

/*
*  m1_kia_seed_decode.c
*  M1 sub-ghz KIA Seed 61-bit decoding
*  te_short=250us, te_long=500us
*  Preamble: >15 pairs (250H+250L), start: (500H+500L), data: bit0=(250H+250L), bit1=(500H+500L)
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_kia_seed(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint16_t bits_count = 0;
    uint16_t max_bits;
    uint16_t header_count = 0;
    int16_t data_start = -1;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* Scan for start bit (long+long after 15+ preamble short+short pairs) */
    for (i = 0; i + 1 < pulsecount; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        if (get_diff(t_hi, te_short) < tol_s && get_diff(t_lo, te_short) < tol_s)
        {
            header_count++;
        }
        else if (get_diff(t_hi, te_long) < tol_l && get_diff(t_lo, te_long) < tol_l
                 && header_count > 15)
        {
            /* Found start bit; add it as '1' per Flipper decoder, data begins next */
            data_start = (int16_t)(i + 2);
            code = 1;
            bits_count = 1;
            break;
        }
        else
        {
            header_count = 0;
        }
    }

    if (data_start < 0)
        return 1;

    /* Decode data bits */
    for (i = (uint16_t)data_start; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        if (get_diff(t_hi, te_short) < tol_s && get_diff(t_lo, te_short) < tol_s)
        {
            /* bit 0 */
        }
        else if (get_diff(t_hi, te_long) < tol_l && get_diff(t_lo, te_long) < tol_l)
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
