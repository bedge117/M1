/* See COPYING.txt for license details. */
/*
 * Ported from Momentum Firmware — https://github.com/Next-Flip/Momentum-Firmware
 * Original: lib/subghz/protocols/wendox_w6726.c
 * Licensed under GPLv3
 *
 * m1_wendox_w6726_decode.c
 * Wendox W6726 weather sensor — 29 bits, PWM
 * te=1955/5865µs
 */
#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_wendox_w6726(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
