/* See COPYING.txt for license details. */

/*
*  m1_infactory_decode.c
*
*  M1 sub-ghz Infactory weather station 40-bit decoding
*  OOK PWM: te_short ~500μs, te_long ~1500μs (1:3 ratio)
*  Preamble: 4 sync pulses
*  Temperature + humidity sensor on 433.92 MHz
*
*  Packet: 40 bits (5 bytes)
*    byte 0:    sensor ID (8-bit, randomized at battery change)
*    byte 1:    channel (2 bits) + battery (1 bit) + flags (5 bits)
*    bytes 2-3: temperature (12 bits, signed, 0.1°C resolution)
*    byte 4:    humidity (8-bit, 0-100%)
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_INFACTORY"

extern SubGHz_Weather_Data_t weather_data;

uint8_t subghz_decode_infactory(uint16_t p, uint16_t pulsecount)
{
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint8_t data[5];
    uint8_t bit_count = 0;
    uint8_t byte_idx = 0, bit_idx = 7;

    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    memset(data, 0, sizeof(data));

    /* Skip preamble sync pulses */
    i = subghz_protocols_list[p].preamble_bits;

    /* PWM decode: high pulse width determines bit value */
    for (; i + 1 < pulsecount && bit_count < 40; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];

        if (get_diff(t_hi, te_long) < tol_l)
        {
            data[byte_idx] |= (1 << bit_idx); /* bit 1 */
        }
        else if (get_diff(t_hi, te_short) < tol_s)
        {
            /* bit 0 */
        }
        else
        {
            break;
        }

        bit_count++;
        if (bit_idx == 0)
        {
            bit_idx = 7;
            byte_idx++;
        }
        else
        {
            bit_idx--;
        }
    }

    if (bit_count < 40) return 1;

    /* Extract fields */
    uint8_t sensor_id  = data[0];
    uint8_t channel    = (data[1] >> 6) & 0x03;
    uint8_t battery    = (data[1] >> 5) & 0x01;

    /* Temperature: 12-bit signed, 0.1°C */
    int16_t temp_raw = ((data[2] & 0x0F) << 8) | data[3];
    if (temp_raw & 0x0800) /* sign extend 12-bit */
        temp_raw |= 0xF000;

    uint8_t humidity = data[4];

    weather_data.id = sensor_id;
    weather_data.channel = channel + 1;
    weather_data.temp_raw = temp_raw;
    weather_data.humidity = humidity;
    weather_data.battery_low = battery;
    weather_data.valid = 1;

    /* Store raw value */
    uint64_t code = 0;
    for (i = 0; i < 5; i++)
        code = (code << 8) | data[i];

    subghz_decenc_ctl.n64_decodedvalue = code;
    subghz_decenc_ctl.ndecodedbitlength = 40;
    subghz_decenc_ctl.ndecodeddelay = 0;
    subghz_decenc_ctl.ndecodedprotocol = p;

    M1_LOG_I(M1_LOGDB_TAG, "Infactory: id=%02X ch=%d temp=%d.%dC hum=%d%% batt=%d\r\n",
             sensor_id, channel + 1,
             temp_raw / 10, abs(temp_raw) % 10,
             humidity, battery);

    return 0;
}
