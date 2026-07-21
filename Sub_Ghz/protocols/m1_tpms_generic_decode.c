/* See COPYING.txt for license details. */
/*
 *  m1_tpms_generic_decode.c — TPMS Generic Manchester decoder
 *  Catch-all for aftermarket TPMS sensors on 433/315 MHz.
 *  te_short=52us, te_long=104us, min 32 bits, Manchester encoded.
 *  Ported from Momentum Firmware TPMS Generic.
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_tpms_generic(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_manchester(p, pulsecount);
}
