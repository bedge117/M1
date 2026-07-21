/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/vauno_en8822c.c
 * Licensed under GPLv3
 *
 * m1_vauno_en8822c_decode.c
 * Vauno EN8822C weather sensor — 42 bits, PPM
 * te_short=500µs (pulse), te_long=1940µs (bit-0 gap), bit-1 gap≈3880µs
 */
#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_ppm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_vauno_en8822c(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_ppm(p, pulsecount);
}
