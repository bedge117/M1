/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/lacrosse_tx141thbv2.c
 * Licensed under GPLv3
 *
 * m1_lacrosse_tx141thbv2_decode.c
 * LaCrosse TX141THBv2 weather sensor — 40 bits, PWM
 * te=208/417µs
 */
#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_lacrosse_tx141thbv2(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
