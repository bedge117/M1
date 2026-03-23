/* See COPYING.txt for license details. */

/*
*  m1_came_twee_decode.c
*
*  M1 sub-ghz CAME Twee 54-bit decoding
*  te=260, ratio 1:2
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_came_twee(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
