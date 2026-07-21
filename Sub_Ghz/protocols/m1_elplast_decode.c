/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/elplast.c
 * Licensed under GPLv3
 *
 * m1_elplast_decode.c
 * Elplast/P-11B/3BK remote — 18 bits, inverted PWM
 * te=230/1550µs, bit1=1550H+230L, bit0=230H+1550L
 * GAP ≈ 1550*8 = 12400µs between repetitions
 */
#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_elplast(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol;
    uint16_t bits_count = 0;
    uint16_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    if (tol < 160) tol = 160;  /* match Momentum te_delta=160 */

    for (uint16_t i = 0; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        code <<= 1;

        /* bit 1: long HIGH + short LOW */
        if (get_diff(t_hi, te_long) < tol &&
            get_diff(t_lo, te_short) < tol)
        {
            code |= 1;
        }
        /* bit 0: short HIGH + long LOW */
        else if (get_diff(t_hi, te_short) < tol &&
                 get_diff(t_lo, te_long) < tol)
        {
            /* bit 0 — no action */
        }
        /* bit 0 with GAP at end: short HIGH + long gap */
        else if (get_diff(t_hi, te_short) < tol &&
                 t_lo > te_long * 4)
        {
            /* last bit is 0, gap follows */
            bits_count++;
            break;
        }
        /* bit 1 with GAP at end: long HIGH + long gap */
        else if (get_diff(t_hi, te_long) < tol &&
                 t_lo > te_long * 4)
        {
            code |= 1;
            bits_count++;
            break;
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
