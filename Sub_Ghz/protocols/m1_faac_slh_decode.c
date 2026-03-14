/* See COPYING.txt for license details. */

/*
*  m1_faac_slh_decode.c
*
*  M1 sub-ghz FAAC SLH 64-bit decoding (decode-only)
*  Manchester encoding: te_short ~255μs (half-bit), te_long ~510μs (full-bit)
*  Gate automation on 433.92/868 MHz (EU market)
*  SLH = SelfLearning Hopping — rolling code, decode-only
*
*  Packet: 64 bits
*    bits 0-31:  encrypted rolling code
*    bits 32-59: serial number (28-bit)
*    bits 60-63: button code
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_FAAC_SLH"

uint8_t subghz_decode_faac_slh(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint8_t bit_count = 0;
    uint8_t max_bits;
    uint8_t last_level = 1;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* Manchester decode: two shorts = same level, one long = transition */
    for (i = 0; i < pulsecount && bit_count < max_bits; i++)
    {
        uint16_t dur = subghz_decenc_ctl.pulse_times[i];

        if (get_diff(dur, te_short) < tol_s)
        {
            /* Short pulse — need another short to complete the bit */
            i++;
            if (i >= pulsecount) break;
            dur = subghz_decenc_ctl.pulse_times[i];
            if (get_diff(dur, te_short) < tol_s)
            {
                /* Two shorts = same level */
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
            /* Long pulse = transition */
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

        /* Extract fields */
        subghz_decenc_ctl.n32_serialnumber = (uint32_t)((code >> 4) & 0x0FFFFFFF);
        subghz_decenc_ctl.n8_buttonid = (uint8_t)(code & 0x0F);
        subghz_decenc_ctl.n32_rollingcode = (uint32_t)(code >> 32);

        M1_LOG_I(M1_LOGDB_TAG, "FAAC SLH: serial=%08lX btn=%X rolling=%08lX\r\n",
                 subghz_decenc_ctl.n32_serialnumber,
                 subghz_decenc_ctl.n8_buttonid,
                 subghz_decenc_ctl.n32_rollingcode);

        return 0;
    }

    return 1;
}
