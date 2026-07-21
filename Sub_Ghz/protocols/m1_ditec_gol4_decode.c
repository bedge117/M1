/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/ditec_gol4.c
 * Licensed under GPLv3
 *
 * m1_ditec_gol4_decode.c
 * DITEC GOL4 gate remote — 54 bits, PWM
 * te=400/1100µs
 */
#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_ditec_gol4(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
