/* See COPYING.txt for license details. */

/*
*  m1_gangqi_decode.c
*
*  M1 sub-ghz GangQi decoding
*  te=500/1200, 34 bits
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_gangqi(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
