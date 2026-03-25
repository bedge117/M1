/* See COPYING.txt for license details. */

/*
*  m1_lacrosse_tx_decode.c
*
*  M1 sub-ghz LaCrosse TX series weather station decoding
*  OOK PWM: te_short ~550μs, te_long ~1100μs
*  Common sensors: TX-3, TX-6U, TX-7U (temp on 433.92 MHz)
*
*  Packet: 44 bits
*    bits 0-3:   preamble (0000)
*    bits 4-7:   sensor type (0 = temp, 14/0xE = humidity)
*    bits 8-14:  sensor ID (7 bits, randomized at battery change)
*    bit  15:    new battery flag
*    bits 16-27: data (temperature: BCD hundreds + tens + ones, offset 500 for 50.0°C)
*    bits 28-35: data repeated (for error checking)
*    bits 36-43: CRC-8
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_LACROSSE"

extern SubGHz_Weather_Data_t weather_data;

uint8_t subghz_decode_lacrosse_tx(uint16_t p, uint16_t pulsecount)
{
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint64_t code = 0;
    uint8_t bit_count = 0;
    uint8_t max_bits;

    max_bits = subghz_protocols_list[p].data_bits;
    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    /* PWM decode: each bit is a pulse pair */
    for (i = 0; i + 1 < pulsecount && bit_count < max_bits; i += 2)
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
        bit_count++;
    }

    if (bit_count < max_bits) return 1;

    /* Extract fields from 44-bit packet */
    uint8_t sensor_type = (code >> 36) & 0x0F;
    uint8_t sensor_id   = (code >> 29) & 0x7F;
    uint8_t new_batt    = (code >> 28) & 0x01;

    /* Data nibbles (BCD): hundreds, tens, ones */
    uint8_t d_hundreds = (code >> 24) & 0x0F;
    uint8_t d_tens     = (code >> 20) & 0x0F;
    uint8_t d_ones     = (code >> 16) & 0x0F;

    int16_t raw_value = d_hundreds * 100 + d_tens * 10 + d_ones;

    if (sensor_type == 0x00)
    {
        /* Temperature: offset by 500 (50.0°C) — value is in 0.1°C */
        int16_t temp_c_10 = raw_value - 500;
        weather_data.temp_raw = temp_c_10;
        weather_data.humidity = 0;
    }
    else if (sensor_type == 0x0E)
    {
        /* Humidity: raw value is percentage */
        weather_data.temp_raw = 0;
        weather_data.humidity = (uint8_t)(raw_value / 10);
    }
    else
    {
        return 1; /* Unknown sensor type */
    }

    /* LaCrosse TX CRC: nibble sum of nibbles 1-8, masked to 4 bits,
     * compared against nibble 9 (bits 7-4) and nibble 10 (bits 3-0) */
    uint8_t nib_sum = 0;
    for (i = 1; i <= 8; i++) {
        nib_sum += (uint8_t)((code >> (40 - i * 4)) & 0x0F);
    }
    uint8_t crc_nib = (uint8_t)(code & 0xFF);
    uint8_t crc_ok = ((nib_sum & 0xFF) == crc_nib);

    weather_data.id = sensor_id;
    weather_data.channel = 1; /* LaCrosse TX doesn't have channel */
    weather_data.battery_low = new_batt ? 0 : 1; /* new_batt=1 means good */
    weather_data.valid = crc_ok;

    subghz_decenc_ctl.n64_decodedvalue = code;
    subghz_decenc_ctl.ndecodedbitlength = bit_count;
    subghz_decenc_ctl.ndecodeddelay = 0;
    subghz_decenc_ctl.ndecodedprotocol = p;

    M1_LOG_I(M1_LOGDB_TAG, "LaCrosse: type=%d id=%02X temp=%d.%dC hum=%d batt=%d\r\n",
             sensor_type, sensor_id,
             weather_data.temp_raw / 10, abs(weather_data.temp_raw) % 10,
             weather_data.humidity, weather_data.battery_low);

    return 0;
}
