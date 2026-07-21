/* See COPYING.txt for license details. */

/*
*  m1_somfy_keytis_decode.c
*
*  M1 sub-ghz Somfy Keytis decoding
*  te=640/1280, 80 bits, Manchester
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_manchester(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_somfy_keytis(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_manchester(p, pulsecount);
}
