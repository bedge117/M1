/* See COPYING.txt for license details. */

/*
*  m1_acurite_decode.c
*
*  M1 sub-ghz Acurite weather station decoding
*  OOK PWM: short pulse (~200μs) = bit 0, long pulse (~600μs) = bit 1
*  Preamble: 4 x sync pulses
*  Common sensor: Acurite 00592TXR (temp+humidity on 433.92 MHz)
*
*  Packet: 7 bytes (56 bits)
*    byte 0-1: sensor ID (14 bits) + battery (1 bit) + channel (1 bit)
*    byte 2: humidity (7 bits)
*    byte 3-4: temperature (12 bits, offset by 1000, in 0.1°F)
*    byte 5-6: CRC or parity check
*/

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

#define M1_LOGDB_TAG	"SUBGHZ_ACURITE"

extern SubGHz_Weather_Data_t weather_data;

uint8_t subghz_decode_acurite(uint16_t p, uint16_t pulsecount)
{
    uint16_t te_short, te_long, tol_s, tol_l;
    uint16_t i;
    uint8_t data[7];
    uint8_t bit_count = 0;
    uint8_t byte_idx = 0, bit_idx = 7;
    uint16_t raw_temp;

    te_short = subghz_protocols_list[p].te_short;
    te_long  = subghz_protocols_list[p].te_long;
    tol_s = (te_short * subghz_protocols_list[p].te_tolerance) / 100;
    tol_l = (te_long  * subghz_protocols_list[p].te_tolerance) / 100;

    memset(data, 0, sizeof(data));

    /* Skip preamble sync pulses */
    i = subghz_protocols_list[p].preamble_bits;

    /* PWM decode: each bit is a high pulse + low pulse.
     * The high pulse width determines the bit value. */
    for (; i + 1 < pulsecount && bit_count < 56; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];

        if (get_diff(t_hi, te_long) < tol_l)
        {
            data[byte_idx] |= (1 << bit_idx);  /* bit 1 */
        }
        else if (get_diff(t_hi, te_short) < tol_s)
        {
            /* bit 0 — already 0 */
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

    if (bit_count < 56) return 1;

    /* Extract fields */
    uint16_t sensor_id = ((data[0] & 0x3F) << 8) | data[1];
    uint8_t  battery   = (data[0] >> 6) & 0x01;
    uint8_t  channel   = (data[0] >> 7) & 0x01;
    uint8_t  humidity  = data[2] & 0x7F;

    /* Temperature: 12 bits, offset 1000, units 0.1°F */
    raw_temp = ((data[3] & 0x0F) << 8) | data[4];

    /* Convert from Acurite format (0.1°F with offset 1000) to 0.1°C */
    int16_t temp_f_10 = (int16_t)raw_temp - 1000;
    int16_t temp_c_10 = ((temp_f_10 - 320) * 5) / 9;

    weather_data.id = sensor_id;
    weather_data.channel = channel + 1;
    weather_data.temp_raw = temp_c_10;
    weather_data.humidity = humidity;
    weather_data.battery_low = battery;
    weather_data.valid = 1;  /* TODO: add CRC check on data[5..6] */

    /* Store raw value */
    uint64_t code = 0;
    for (i = 0; i < 7; i++)
        code = (code << 8) | data[i];

    subghz_decenc_ctl.n64_decodedvalue = code;
    subghz_decenc_ctl.ndecodedbitlength = 56;
    subghz_decenc_ctl.ndecodeddelay = 0;
    subghz_decenc_ctl.ndecodedprotocol = p;

    M1_LOG_I(M1_LOGDB_TAG, "Acurite: id=%04X ch=%d temp=%d.%dC hum=%d%% batt=%d\r\n",
             sensor_id, channel + 1,
             temp_c_10 / 10, abs(temp_c_10) % 10,
             humidity, battery);

    return 0;
}
