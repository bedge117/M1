/* See COPYING.txt for license details. */

/*
*  m1_schrader_decode.c
*
*  M1 sub-ghz Schrader TPMS 40-bit decoding
*  Manchester encoding: te_short ~120μs (half-bit), te_long ~240μs (full-bit)
*  Tire Pressure Monitoring System on 315/433 MHz
*  NOTE: Many TPMS sensors use FSK modulation. This decoder works with
*  OOK/ASK variants or when the radio is configured for FSK demodulation.
*
*  Packet: 40 bits (after 8-bit preamble)
*    bits 0-7:   sensor type/status
*    bits 8-31:  sensor ID (24-bit)
*    bits 32-39: pressure or temperature data
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_SCHRADER"

uint8_t subghz_decode_schrader(uint16_t p, uint16_t pulsecount)
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

    /* Skip preamble */
    i = subghz_protocols_list[p].preamble_bits;

    /* Manchester decode */
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

        /* Extract sensor ID (bits 8-31) */
        subghz_decenc_ctl.n32_serialnumber = (uint32_t)((code >> 8) & 0x00FFFFFF);

        M1_LOG_I(M1_LOGDB_TAG, "Schrader: id=%06lX data=0x%010llX\r\n",
                 subghz_decenc_ctl.n32_serialnumber, code);

        return 0;
    }

    return 1;
}
