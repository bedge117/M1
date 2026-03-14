/* See COPYING.txt for license details. */

/*
*  m1_bin_raw_decode.c
*
*  M1 sub-ghz BinRAW generic decoder (fallback)
*  Auto-detects timing element from pulse data and extracts binary bits.
*  This is a last-resort decoder that runs after all specific protocols fail.
*  It attempts to find a consistent timing pattern and extract structured
*  binary data from any OOK signal.
*
*  Algorithm:
*    1. Find the shortest pulse as the base timing unit (te)
*    2. Classify each pulse as short (1*te) or long (2-3*te)
*    3. Decode PWM: short+long = 0, long+short = 1
*    4. Require at least 16 bits for a valid decode
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_BINRAW"

#define BINRAW_MIN_BITS     16
#define BINRAW_MAX_BITS     64

uint8_t subghz_decode_bin_raw(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t i;
    uint8_t bits_count = 0;
    uint16_t min_pulse = 0xFFFF;
    uint16_t te_short, te_long, tol_s, tol_l;

    (void)p; /* BinRAW doesn't use the protocol table timings */

    if (pulsecount < BINRAW_MIN_BITS * 2)
        return 1;

    /* Step 1: Find the shortest pulse (excluding the last gap pulse) */
    for (i = 0; i + 1 < pulsecount; i++)
    {
        if (subghz_decenc_ctl.pulse_times[i] < min_pulse &&
            subghz_decenc_ctl.pulse_times[i] >= PACKET_PULSE_TIME_MIN)
        {
            min_pulse = subghz_decenc_ctl.pulse_times[i];
        }
    }

    if (min_pulse >= INTERPACKET_GAP_MIN || min_pulse < PACKET_PULSE_TIME_MIN)
        return 1;

    /* Step 2: Use shortest pulse as te_short, estimate te_long */
    te_short = min_pulse;
    te_long  = min_pulse * 3; /* Try 1:3 ratio first */
    tol_s = (te_short * PACKET_PULSE_TIME_TOLERANCE30) / 100;
    tol_l = (te_long  * PACKET_PULSE_TIME_TOLERANCE30) / 100;

    /* Step 3: Try PWM decode with detected timings */
    for (i = 0; i + 1 < pulsecount && bits_count < BINRAW_MAX_BITS; i += 2)
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
            code |= 1;
        }
        else
        {
            break;
        }
        bits_count++;
    }

    /* If 1:3 didn't work, try 1:2 ratio */
    if (bits_count < BINRAW_MIN_BITS)
    {
        code = 0;
        bits_count = 0;
        te_long = min_pulse * 2;
        tol_l = (te_long * PACKET_PULSE_TIME_TOLERANCE30) / 100;

        for (i = 0; i + 1 < pulsecount && bits_count < BINRAW_MAX_BITS; i += 2)
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
                code |= 1;
            }
            else
            {
                break;
            }
            bits_count++;
        }
    }

    if (bits_count >= BINRAW_MIN_BITS)
    {
        subghz_decenc_ctl.n64_decodedvalue = code;
        subghz_decenc_ctl.ndecodedbitlength = bits_count;
        subghz_decenc_ctl.ndecodeddelay = te_short; /* Store detected te */
        subghz_decenc_ctl.ndecodedprotocol = p;

        M1_LOG_I(M1_LOGDB_TAG, "BinRAW: te=%d bits=%d code=0x%lX%lX\r\n",
                 te_short, bits_count,
                 (uint32_t)(code >> 32), (uint32_t)code);

        return 0;
    }

    return 1;
}
