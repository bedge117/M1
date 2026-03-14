/* See COPYING.txt for license details. */

/*
*  m1_keeloq_decode.c
*
*  M1 sub-ghz KeeLoq decoding (decode-only, no key cracking)
*  Encoding: PWM - short-high + long-low → bit 0; long-high + short-low → bit 1
*  te_short ~400μs, te_long ~800μs
*  Preamble: header pulse, then 66 bits (32-bit encrypted + 28-bit serial + 4-bit button + 2 status)
*  Used by: Chamberlain, LiftMaster, Craftsman, many car alarms
*
*  NOTE: This only captures the raw rolling code packet for display.
*        Decryption requires the manufacturer key which is not included.
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_KEELOQ"

uint8_t subghz_decode_keeloq(uint16_t p, uint16_t pulsecount)
{
    uint64_t code = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint8_t bits_count = 0;
    uint8_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* Skip preamble bits if defined */
    i = subghz_protocols_list[p].preamble_bits;
    if (i >= pulsecount) return 1;

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
            code |= 1;
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

        /* Extract serial (bits 34-61 of the 66-bit packet = upper 28 bits after encrypted) */
        subghz_decenc_ctl.n32_serialnumber = (uint32_t)((code >> 2) & 0x0FFFFFFF);
        subghz_decenc_ctl.n8_buttonid = (uint8_t)((code >> 30) & 0x0F);
        /* The lower 32 bits are the encrypted rolling code */
        subghz_decenc_ctl.n32_rollingcode = (uint32_t)(code & 0xFFFFFFFF);

        return 0;
    }

    return 1;
}
