/* See COPYING.txt for license details. */

/*
*  m1_generic_decode.c
*
*  Generic sub-ghz decoders for common encoding schemes.
*  Protocols with standard OOK PWM encoding (1:2 or 1:3 ratio)
*  or Manchester encoding can call these instead of duplicating logic.
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_GENERIC"

/*
 * Generic OOK PWM decoder (works for both 1:2 and 1:3 ratio).
 * Pulse pairs: short-high + long-low = bit 0, long-high + short-low = bit 1.
 * Uses te_short/te_long/tolerance/preamble_bits/data_bits from protocol table.
 * Returns 0 on success, 1 on failure.
 */
uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount)
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

    /* Skip preamble if any */
    i = subghz_protocols_list[p].preamble_bits;

    for (; i + 1 < pulsecount && bits_count < max_bits; i += 2)
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

/*
 * Generic Manchester decoder.
 * Two consecutive short pulses = same bit as last, one long pulse = bit flip.
 * Uses te_short/te_long/tolerance/preamble_bits/data_bits from protocol table.
 * Returns 0 on success, 1 on failure.
 */
uint8_t subghz_decode_generic_manchester(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint16_t bit_count = 0;
    uint16_t max_bits;
    uint8_t last_level = 1;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* Skip preamble */
    i = subghz_protocols_list[p].preamble_bits;

    for (; i < pulsecount && bit_count < max_bits; i++)
    {
        uint16_t dur = subghz_decenc_ctl.pulse_times[i];

        if (get_diff(dur, te_short) < tol_s)
        {
            i++;
            if (i >= pulsecount) break;
            dur = subghz_decenc_ctl.pulse_times[i];
            if (get_diff(dur, te_short) < tol_s)
            {
                code = (code << 1) | last_level;
                bit_count++;
            }
            else
            {
                break;
            }
        }
        else if (get_diff(dur, te_long) < tol_l)
        {
            last_level ^= 1;
            code = (code << 1) | last_level;
            bit_count++;
        }
        else
        {
            break;
        }
    }

    if (bit_count >= max_bits)
    {
        subghz_decenc_ctl.n64_decodedvalue = code;
        subghz_decenc_ctl.ndecodedbitlength = bit_count;
        subghz_decenc_ctl.ndecodeddelay = 0;
        subghz_decenc_ctl.ndecodedprotocol = p;
        return 0;
    }

    return 1;
}
