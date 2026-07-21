/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/kedsum_th.c
 * Licensed under GPLv3
 *
 * m1_kedsum_th_decode.c
 * Kedsum temperature/humidity sensor — 42 bits, PPM
 * te_short=500µs (pulse), te_long=2000µs (bit-0 gap), bit-1 gap≈4000µs
 */
#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_ppm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_kedsum_th(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_ppm(p, pulsecount);
}
