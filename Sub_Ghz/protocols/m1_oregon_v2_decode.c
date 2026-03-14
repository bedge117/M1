/* See COPYING.txt for license details. */

/*
*  m1_oregon_v2_decode.c
*
*  M1 sub-ghz Oregon Scientific v2.1 weather station decoding
*  Manchester encoded, preamble of 0xFF (16 1-bits), sync nibble 0xA
*  te_short ~488μs (Manchester half-bit), te_long ~976μs
*  Common sensors: THGR122N, THGN123N (temp+humidity on 433.92 MHz)
*
*  Packet format (nibbles): SSSS CC ID TT TH HH XX
*    SSSS = sensor type, CC = channel, ID = rolling ID
*    TT.T = temperature (BCD, 0.1°C), HH = humidity (BCD)
*    XX = checksum
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_OREGON"

/* Forward declaration of weather data (defined in m1_sub_ghz_decenc.c) */
extern SubGHz_Weather_Data_t weather_data; /* declared static in decenc.c — need to expose */

uint8_t subghz_decode_oregon_v2(uint16_t p, uint16_t pulsecount)
{
    uint16_t te_short, tol_s;
    uint16_t i;
    uint8_t bits[80];
    uint8_t bit_count = 0;
    uint8_t nibbles[16];
    uint8_t n, checksum, calc_sum;
    int16_t temp_raw;
    uint8_t last_level = 1;  /* Start high (after preamble) */

    te_short = subghz_protocols_list[p].te_short;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;

    /* Manchester decode:
     * A short pulse keeps the same level, a long pulse transitions.
     * Two consecutive short pulses = one bit period. */
    for (i = 0; i < pulsecount && bit_count < 72; i++)
    {
        uint16_t dur = subghz_decenc_ctl.pulse_times[i];

        if (get_diff(dur, te_short) < tol_s)
        {
            /* Short pulse — half a bit period. Next short completes the bit. */
            i++;
            if (i >= pulsecount) break;
            dur = subghz_decenc_ctl.pulse_times[i];
            if (get_diff(dur, te_short) < tol_s)
            {
                /* Two shorts = no transition, same level */
                bits[bit_count++] = last_level;
            }
            else
            {
                break; /* unexpected length */
            }
        }
        else if (get_diff(dur, te_short * 2) < tol_s * 2)
        {
            /* Long pulse = transition */
            last_level ^= 1;
            bits[bit_count++] = last_level;
        }
        else
        {
            break;
        }
    }

    /* Need at least 56 bits (after preamble) for a basic temp+humidity packet */
    if (bit_count < 56) return 1;

    /* Skip preamble 1s, find sync nibble 0xA (1010) */
    uint8_t sync_start = 0;
    for (i = 0; i + 3 < bit_count; i++)
    {
        if (bits[i] == 1 && bits[i+1] == 0 && bits[i+2] == 1 && bits[i+3] == 0)
        {
            sync_start = i + 4; /* Data starts after sync */
            break;
        }
    }
    if (sync_start == 0 || sync_start + 40 > bit_count) return 1;

    /* Convert remaining bits to nibbles (LSB first within each nibble) */
    uint8_t data_bits = bit_count - sync_start;
    uint8_t num_nibbles = data_bits / 4;
    if (num_nibbles > 16) num_nibbles = 16;

    for (n = 0; n < num_nibbles; n++)
    {
        uint8_t base = sync_start + n * 4;
        nibbles[n] = (bits[base] << 0) | (bits[base+1] << 1) |
                     (bits[base+2] << 2) | (bits[base+3] << 3);
    }

    if (num_nibbles < 10) return 1;

    /* Extract fields (Oregon v2.1 THGR122N layout):
     * nibbles[0..3] = sensor type
     * nibbles[4]    = channel
     * nibbles[5..6] = rolling ID
     * nibbles[7]    = temperature sign + tens digit flags
     * nibbles[8]    = temperature ones digit
     * nibbles[9]    = temperature tenths
     * nibbles[10..11] = humidity (if present) */

    /* Simple checksum: sum of nibbles[0..num_nibbles-2] vs nibbles[num_nibbles-1] */
    calc_sum = 0;
    for (n = 0; n < num_nibbles - 1; n++)
        calc_sum += nibbles[n];
    checksum = nibbles[num_nibbles - 1];

    /* Temperature: BCD encoded, nibbles in reverse order */
    temp_raw = nibbles[9] * 100 + nibbles[8] * 10 + nibbles[7];
    /* Check sign bit (nibble[7] bit 3) — if sensor type indicates signed */
    if (nibbles[7] & 0x08)
        temp_raw = -temp_raw;

    weather_data.id = (nibbles[5] << 4) | nibbles[6];
    weather_data.channel = nibbles[4] + 1;
    weather_data.temp_raw = temp_raw;
    weather_data.humidity = (num_nibbles > 11) ?
                            (nibbles[10] * 10 + nibbles[11]) : 0;
    weather_data.battery_low = 0;
    weather_data.valid = ((calc_sum & 0x0F) == (checksum & 0x0F)) ? 1 : 0;

    /* Store decoded value for generic display */
    uint64_t code = 0;
    for (n = 0; n < num_nibbles && n < 16; n++)
        code = (code << 4) | nibbles[n];

    subghz_decenc_ctl.n64_decodedvalue = code;
    subghz_decenc_ctl.ndecodedbitlength = num_nibbles * 4;
    subghz_decenc_ctl.ndecodeddelay = 0;
    subghz_decenc_ctl.ndecodedprotocol = p;

    M1_LOG_I(M1_LOGDB_TAG, "Oregon: ch=%d id=%02X temp=%d.%d hum=%d valid=%d\r\n",
             weather_data.channel, weather_data.id,
             weather_data.temp_raw / 10, abs(weather_data.temp_raw) % 10,
             weather_data.humidity, weather_data.valid);

    return 0;
}
