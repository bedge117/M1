/* See COPYING.txt for license details. */

/*
*  m1_somfy_telis_decode.c
*
*  M1 sub-ghz Somfy Telis (RTS) 56-bit decoding (decode-only)
*  Manchester encoding: te_short ~640μs (half-bit), te_long ~1280μs (full-bit)
*  Motorized blinds/awnings on 433.42 MHz
*
*  Sync: 4 short preamble pulses, then one long hardware sync (~4.5ms)
*  Data: 56 bits Manchester encoded, XOR obfuscated
*
*  Frame structure (after deobfuscation):
*    byte 0:    encryption key (4 bits) + checksum (4 bits)
*    byte 1:    command (up=2, down=4, stop=1, prog=8)
*    bytes 2-3: rolling code (16-bit)
*    bytes 4-6: address (24-bit)
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_SOMFY"

uint8_t subghz_decode_somfy_telis(uint16_t p, uint16_t pulsecount)
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

    /* Skip preamble pulses */
    i = subghz_protocols_list[p].preamble_bits;

    /* Skip hardware sync pulse (long pulse ~4-5ms = ~7*te_short) */
    while (i < pulsecount)
    {
        if (subghz_decenc_ctl.pulse_times[i] > te_long * 2)
        {
            i++;
            break;
        }
        i++;
    }

    /* Manchester decode data bits */
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
        /* Deobfuscate: each byte is XORed with the previous byte */
        uint8_t frame[7];
        uint8_t n;
        for (n = 0; n < 7; n++)
            frame[n] = (uint8_t)((code >> (48 - n * 8)) & 0xFF);

        for (n = 6; n > 0; n--)
            frame[n] ^= frame[n - 1];

        /* Reconstruct deobfuscated code */
        uint64_t deob = 0;
        for (n = 0; n < 7; n++)
            deob = (deob << 8) | frame[n];

        subghz_decenc_ctl.n64_decodedvalue = deob;
        subghz_decenc_ctl.ndecodedbitlength = bit_count;
        subghz_decenc_ctl.ndecodeddelay = 0;
        subghz_decenc_ctl.ndecodedprotocol = p;

        /* Extract fields from deobfuscated frame */
        subghz_decenc_ctl.n8_buttonid = frame[1] & 0x0F;
        subghz_decenc_ctl.n32_rollingcode = (frame[2] << 8) | frame[3];
        subghz_decenc_ctl.n32_serialnumber = ((uint32_t)frame[4] << 16) |
                                              ((uint32_t)frame[5] << 8) |
                                              frame[6];

        M1_LOG_I(M1_LOGDB_TAG, "Somfy: cmd=%X rolling=%04lX addr=%06lX\r\n",
                 subghz_decenc_ctl.n8_buttonid,
                 subghz_decenc_ctl.n32_rollingcode,
                 subghz_decenc_ctl.n32_serialnumber);

        return 0;
    }

    return 1;
}
