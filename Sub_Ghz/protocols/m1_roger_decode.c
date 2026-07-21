/* See COPYING.txt for license details. */

/*
*  m1_roger_decode.c
*
*  M1 sub-ghz Roger decoding
*  te=500/1000, 28 bits
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_roger(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
