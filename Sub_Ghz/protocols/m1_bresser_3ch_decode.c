/* See COPYING.txt for license details. */

/*
*  m1_bresser_3ch_decode.c
*
*  M1 sub-ghz Bresser 3-channel 40-bit weather station decoding
*  te=250, ratio 1:2, tol 25%
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_bresser_3ch(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
