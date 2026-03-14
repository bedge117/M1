/* See COPYING.txt for license details. */

/*
*  m1_came_decode.c
*
*  M1 sub-ghz CAME 12-bit decoding
*  Encoding: short=high, long=low → bit 0; long=high, short=low → bit 1
*  te_short ~320μs, te_long ~640μs (1:2 ratio)
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_CAME"

uint8_t subghz_decode_came(uint16_t p, uint16_t pulsecount)
{
    uint32_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint8_t bits_count = 0;
    uint8_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* CAME: pairs of pulses encode bits
     * bit 0: short-high, long-low
     * bit 1: long-high, short-low  */
    for (i = 0; i + 1 < pulsecount && bits_count < max_bits; i += 2)
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
            break; /* invalid pulse pair */
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
