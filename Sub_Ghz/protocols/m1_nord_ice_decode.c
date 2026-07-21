/* See COPYING.txt for license details. */

/*
 *  m1_nord_ice_decode.c
 *
 *  M1 sub-ghz Nord ICE decoding
 *  OOK PWM, 33 bits, 433.92 MHz
 *
 *  Bit encoding (PWM):
 *    bit 0: 300 µs HIGH + 800 µs LOW
 *    bit 1: 800 µs HIGH + 300 µs LOW
 *  GAP: ~7500 µs (25 × te_short)
 *
 *  Packet layout (33 bits):
 *    bits 32-15: 18-bit serial (upper part)
 *    bits 14- 9:  6-bit button code
 *    bits  8- 0:  9-bit serial (lower part)
 *
 *  References:
 *    Momentum Firmware, lib/subghz/protocols/nord_ice.c
 *    @xMasterX (MMX), 2026-03
 */

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_nord_ice(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
