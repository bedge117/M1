/* See COPYING.txt for license details. */

/*
*  m1_nice_flor_s_decode.c
*
*  M1 sub-ghz Nice Flor S 52-bit decoding
*  te=500, ratio 1:2
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_nice_flor_s(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
