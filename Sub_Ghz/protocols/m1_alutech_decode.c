/* See COPYING.txt for license details. */

/*
*  m1_alutech_decode.c
*
*  M1 sub-ghz Alutech AT-4N 72-bit decoding
*  te=400, ratio 1:2
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_alutech(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
