/* See COPYING.txt for license details. */

/*
*  m1_bett_decode.c
*
*  M1 sub-ghz BETT 18-bit decoding
*  te=340, ratio 1:2
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_bett(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
