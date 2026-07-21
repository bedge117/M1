/* See COPYING.txt for license details. */
/*
 *  m1_treadmill37_decode.c — Treadmill37 (QH-433) 37-bit OOK PWM decoder
 *  te_short=300us, te_long=900us, 37 bits, 433 MHz static protocol.
 *  Ported from Momentum Firmware (@xMasterX).
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_treadmill37(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
