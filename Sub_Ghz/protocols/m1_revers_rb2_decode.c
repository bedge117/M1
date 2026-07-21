/* See COPYING.txt for license details. */

/*
*  m1_revers_rb2_decode.c
*
*  M1 sub-ghz Revers_RB2 decoding
*  te=250/500, 64 bits, Manchester
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_manchester(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_revers_rb2(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_manchester(p, pulsecount);
}
