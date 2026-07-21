/* See COPYING.txt for license details. */

/*
 * m1_beninca_arc_decode.c
 *
 * M1 sub-ghz Beninca ARC 128-bit decode-only decoder.
 *
 * Beninca ARC is an AES-128 encrypted rolling code protocol used by Beninca
 * gate/barrier operators.  Full decryption requires the manufacturer key from
 * a Beninca-specific keystore file.  This decoder captures the raw 128-bit
 * payload for display, extracting the unencrypted serial and counter fields
 * from the frame header without performing AES decryption.
 *
 * OTA format (Momentum reference: lib/subghz/protocols/beninca_arc.c):
 *   Encoding: OOK PWM, 1:2 ratio
 *     Bit 0: te_short HIGH + te_long  LOW  (300 + 600 µs)
 *     Bit 1: te_long  HIGH + te_short LOW  (600 + 300 µs)
 *   Total bits: 128
 *   te_short = 300 µs, te_long = 600 µs, te_delta = 155 µs
 *
 * Frame layout (Momentum beninca_arc.c, AES-decrypted payload):
 *   The 128-bit OTA frame is AES-128 encrypted (CBC/ECB with manufacturer key).
 *   After decryption:
 *     [127:96] serial number (32 bits)    → n32_serialnumber (approximate, from raw)
 *     [95:64]  rolling counter (32 bits)  → n32_rollingcode  (approximate, from raw)
 *     [63:0]   encrypted/key data
 *
 * This decoder stores the raw 128-bit payload split across n64_decodedvalue
 * (upper 64 bits) and n32_serialnumber / n32_rollingcode (lower fields).
 * The displayed values are the raw (encrypted) bytes — labelled accordingly
 * in the info screen.  Actual serial/counter are only meaningful after AES
 * decryption with the manufacturer key.
 *
 * NOTE: AES decryption requires the Beninca manufacturer key.
 *       This decoder is detect-only; values shown are raw (encrypted).
 */

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG "SUBGHZ_BENINCA"

uint8_t subghz_decode_beninca_arc(uint16_t p, uint16_t pulsecount)
{
    uint64_t code_hi = 0;
    uint64_t code_lo = 0;
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint8_t bits_count = 0;
    uint8_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;   /* 128 */
    te_short = subghz_protocols_list[p].te_short;    /* 300 µs */
    te_long  = subghz_protocols_list[p].te_long;     /* 600 µs */
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* Skip preamble pulses */
    i = subghz_protocols_list[p].preamble_bits;
    if (i >= pulsecount) return 1;

    /* Collect 128 bits into two 64-bit halves */
    for (; i + 1 < pulsecount && bits_count < max_bits; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];
        uint8_t bit;

        if (get_diff(t_hi, te_short) < tol_s && get_diff(t_lo, te_long) < tol_l)
        {
            bit = 0;
        }
        else if (get_diff(t_hi, te_long) < tol_l && get_diff(t_lo, te_short) < tol_s)
        {
            bit = 1;
        }
        else
        {
            break;
        }

        /* First 64 bits → code_hi, next 64 bits → code_lo */
        if (bits_count < 64)
        {
            code_hi = (code_hi << 1) | bit;
        }
        else
        {
            code_lo = (code_lo << 1) | bit;
        }
        bits_count++;
    }

    if (bits_count >= max_bits)
    {
        /* Store raw upper 64 bits as the primary decoded value.
         * Extract approximate serial and counter from the raw frame
         * (these are the raw/encrypted bytes — not the decrypted values). */
        subghz_decenc_ctl.n64_decodedvalue = code_hi;
        subghz_decenc_ctl.ndecodedbitlength = bits_count;
        subghz_decenc_ctl.ndecodeddelay = 0;
        subghz_decenc_ctl.ndecodedprotocol = p;

        /* Raw frame layout interpretation (encrypted — for display only):
         *   code_hi[63:32] → raw serial field position
         *   code_hi[31:0]  → raw counter field position
         * Actual decryption requires AES-128 with manufacturer key.
         */
        subghz_decenc_ctl.n32_serialnumber = (uint32_t)(code_hi >> 32);
        subghz_decenc_ctl.n32_rollingcode  = (uint32_t)(code_hi & 0xFFFFFFFF);
        (void)code_lo;  /* lower 64 bits not stored (no second field in decenc_ctl) */

        M1_LOG_I(M1_LOGDB_TAG, "Beninca ARC raw_hi=0x%016llX raw_lo=0x%016llX\r\n",
                 (unsigned long long)code_hi, (unsigned long long)code_lo);

        return 0;
    }

    return 1;
}
