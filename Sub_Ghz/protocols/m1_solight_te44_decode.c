/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/solight_te44.c
 * Licensed under GPLv3
 *
 * m1_solight_te44_decode.c
 * Solight TE44 weather sensor — 36 bits, PPM
 * te_short=490µs (pulse), te_long=1960µs (bit-0 gap), bit-1 gap≈3920µs
 */
#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_ppm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_solight_te44(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_ppm(p, pulsecount);
}
