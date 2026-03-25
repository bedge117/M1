/* See COPYING.txt for license details. */

/*
*  m1_intertechno_decode.c
*
*  M1 sub-ghz Intertechno 32-bit decoding
*  te=250, ratio 1:3
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_intertechno(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
