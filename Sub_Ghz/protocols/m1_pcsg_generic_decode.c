/* See COPYING.txt for license details. */
/*
 *  m1_pcsg_generic_decode.c — PCSG Generic pager decoder
 *  Manchester-encoded paging signals on 433 MHz (POCSAG-1200 baud rate).
 *  Complements the full POCSAG decoder for signals without frame sync.
 *  Ported from Momentum Firmware PCSG Generic.
 */
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

uint8_t subghz_decode_pcsg_generic(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_manchester(p, pulsecount);
}
