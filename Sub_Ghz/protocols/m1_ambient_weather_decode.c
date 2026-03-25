/* See COPYING.txt for license details. */

/*
*  m1_ambient_weather_decode.c
*
*  M1 sub-ghz Ambient Weather 40-bit decoding
*  Manchester encoding, te=500, tol 25%
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_manchester(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_ambient_weather(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_manchester(p, pulsecount);
}
