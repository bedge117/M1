/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See COPYING.txt for license details. */

/*
 * subghz_decode_try_fn.c
 *
 * Canonical SubGhzRawDecodeTryFn bridge between subghz_decode_raw_offline()
 * and the global protocol registry + subghz_decenc_ctl.
 *
 * Extracted from subghz_protocol_registry.c so that unit tests can link
 * this file in isolation against a minimal stub registry and a controllable
 * subghz_decenc_read() stub, without pulling in the entire 100+ protocol
 * production registry.
 *
 * Dependencies (resolved at link time):
 *   subghz_protocol_registry[]      — from subghz_protocol_registry.c (firmware)
 *                                     or a test stub registry
 *   subghz_protocol_registry_count  — same
 *   subghz_decenc_ctl               — from m1_sub_ghz_decenc.c (firmware)
 *                                     or a test stub
 *   subghz_decenc_read()            — same
 */

#include <string.h>
#include "subghz_protocol_registry.h"
#include "m1_sub_ghz_decenc.h"
#include "subghz_raw_decoder.h"

bool subghz_registry_decode_try_fn(const uint16_t *pulse_buf,
                                    uint16_t        pulse_count,
                                    SubGhzRawDecodeResult *out_result,
                                    void           *user_ctx)
{
    (void)user_ctx;

    memcpy(subghz_decenc_ctl.pulse_times, pulse_buf,
           pulse_count * sizeof(uint16_t));
    subghz_decenc_ctl.npulsecount = pulse_count;

    for (uint16_t p = 0; p < subghz_protocol_registry_count; p++)
    {
        const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
        if (proto->decode && proto->decode(p, pulse_count) == 0)
        {
            SubGHz_Dec_Info_t info;
            if (subghz_decenc_read(&info, false))
            {
                out_result->protocol      = info.protocol;
                out_result->key           = info.key;
                out_result->bit_len       = info.bit_len;
                out_result->te            = info.te;
                out_result->serial_number = info.serial_number;
                out_result->rolling_code  = info.rolling_code;
                out_result->button_id     = info.button_id;
                return true;
            }
            /* decode() matched at the pulse level but decenc_read() failed
             * (state desync — shouldn't happen).  Continue trying other
             * decoders rather than giving up on this packet entirely. */
        }
    }
    return false;
}
