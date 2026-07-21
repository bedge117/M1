/* See COPYING.txt for license details. */

/*
*  m1_hay21_decode.c
*
*  M1 sub-ghz Hay21 decoding
*  te=300/700, 21 bits
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_hay21(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
