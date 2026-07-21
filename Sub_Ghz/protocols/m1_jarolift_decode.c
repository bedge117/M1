/* See COPYING.txt for license details. */

/*
 * m1_jarolift_decode.c
 *
 * M1 sub-ghz Jarolift (KeeLoq-based) 72-bit decode-only decoder.
 *
 * Jarolift uses the KeeLoq hopping code algorithm with a Jarolift-specific
 * normal learning scheme.  Decryption requires the manufacturer key which is
 * not publicly available, so this decoder captures the raw packet for
 * display only — matching our KeeLoq decoder approach.
 *
 * OTA format (Momentum reference):
 *   Encoding: OOK PWM (same as standard KeeLoq)
 *     Bit 0: te_short HIGH + te_long  LOW  (400 + 800 µs)
 *     Bit 1: te_long  HIGH + te_short LOW  (800 + 400 µs)
 *   Total bits: 72 (64-bit KeeLoq frame + 8 preamble bits)
 *   Preamble: 10–12 header pulses @400 µs
 *   te_short = 400 µs, te_long = 800 µs, tolerance ±167 µs
 *
 * Packet layout (MSB first after preamble):
 *   [71:40] = encrypted hopping code (32 bits)  → n32_rollingcode
 *   [39:12] = serial number (28 bits)            → n32_serialnumber
 *   [11:8]  = button / function (4 bits)         → n8_buttonid
 *   [7:0]   = status / overflow bits
 *
 * References:
 *   Momentum firmware: lib/subghz/protocols/jarolift.c
 *   KeeLoq specification: Microchip AN1200.02
 *
 * NOTE: Decryption requires the Jarolift manufacturer key.
 *       This decoder is detect-only and does not attempt decryption.
 */

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG "SUBGHZ_JAROLIFT"

uint8_t subghz_decode_jarolift(uint16_t p, uint16_t pulsecount)
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

    /* Skip preamble pulses */
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

        /* Jarolift packet layout (72 bits, MSB first):
         *   [71:40] encrypted hop word (32 bits)
         *   [39:12] serial number (28 bits)
         *   [11:8]  button/function id (4 bits)
         *   [7:0]   status/overflow
         */
        subghz_decenc_ctl.n32_rollingcode  = (uint32_t)((code >> 40) & 0xFFFFFFFF);
        subghz_decenc_ctl.n32_serialnumber = (uint32_t)((code >> 12) & 0x0FFFFFFF);
        subghz_decenc_ctl.n8_buttonid      = (uint8_t)((code >> 8) & 0x0F);

        M1_LOG_I(M1_LOGDB_TAG, "Jarolift serial=0x%07lX btn=%u hop=0x%08lX\r\n",
                 (unsigned long)subghz_decenc_ctl.n32_serialnumber,
                 (unsigned)subghz_decenc_ctl.n8_buttonid,
                 (unsigned long)subghz_decenc_ctl.n32_rollingcode);

        return 0;
    }

    return 1;
}
