/* See COPYING.txt for license details. */
/*
 *  m1_pocsag_decode.c — POCSAG pager protocol decoder
 *
 *  POCSAG (Post Office Code Standardisation Advisory Group) is a paging
 *  protocol using FSK modulation (NRZ at baseband).
 *
 *  Three baud rates auto-detected by pulse width:
 *    POCSAG-512:  te ≈ 1950 µs
 *    POCSAG-1200: te ≈ 833 µs
 *    POCSAG-2400: te ≈ 417 µs
 *
 *  Preamble: ≥576 alternating bits (we gate on 24).
 *  Frame sync: 0x7CD215D8 (32-bit codeword).
 *  After frame sync: batches of 16 × 32-bit codewords.
 *    Address CW (bit 31 = 0): RIC[20:3] = bits[30:12], func = bits[11:10]
 *    Message CW (bit 31 = 1): message data in bits[30:11]
 *
 *  M1 output: first decoded RIC → n32_serialnumber, function → n8_buttonid,
 *             first two codewords → n64_decodedvalue.
 *
 *  Ported from Momentum Firmware (Next-Flip/Momentum-Firmware).
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

#define POCSAG_MIN_SYNC_BITS    24u
#define POCSAG_CW_BITS          32u
#define POCSAG_FRAME_SYNC_CODE  0x7CD215D8UL
#define POCSAG_IDLE_CODE_WORD   0x7A89C197UL

/* Timing tolerances for the three rates */
#define TE_1200   833u
#define TE_512    1950u
#define TE_2400   417u
#define TOL_1200  120u
#define TOL_512   300u
#define TOL_2400  70u

uint8_t subghz_decode_pocsag(uint16_t p, uint16_t pulsecount)
{
    if (pulsecount < POCSAG_MIN_SYNC_BITS + POCSAG_CW_BITS * 2)
        return 1;

    /* Auto-detect baud rate: sample first 8 pulses */
    uint32_t avg = 0;
    uint16_t s = (pulsecount < 8) ? pulsecount : 8;
    for (uint16_t i = 0; i < s; i++)
        avg += subghz_decenc_ctl.pulse_times[i];
    avg /= s;

    uint16_t te, tol;
    if      (DURATION_DIFF(avg, TE_1200) < 200) { te = TE_1200; tol = TOL_1200; }
    else if (DURATION_DIFF(avg, TE_512)  < 400) { te = TE_512;  tol = TOL_512;  }
    else if (DURATION_DIFF(avg, TE_2400) < 100) { te = TE_2400; tol = TOL_2400; }
    else return 1;

    /*
     * Walk the pulse array building a raw bit stream.
     * POCSAG is NRZ-L and is inverted on most hardware: HIGH = 0, LOW = 1.
     * Even index pulses are marks (HIGH), odd are spaces (LOW).
     * Each pulse may span multiple bit periods.
     */
    uint32_t running = 0;          /* sliding 32-bit window for sync search */
    bool     past_frame = false;
    uint64_t cw_data  = 0;
    uint16_t cw_bits  = 0;
    uint16_t alt_run  = 0;         /* alternating-bit run length for preamble */

    for (uint16_t i = 0; i < pulsecount; i++)
    {
        uint16_t dur = subghz_decenc_ctl.pulse_times[i];
        bool     level   = (i & 1) == 0;   /* even = HIGH = mark */
        bool     bit_val = !level;          /* invert: HIGH→0, LOW→1 */

        /* Clamp to 1-4 bit periods */
        uint16_t n_bits = (uint16_t)((dur + te / 2) / te);
        if (n_bits < 1) n_bits = 1;
        if (n_bits > 4) n_bits = 4;
        /* If remaining fraction is significant, add one more */
        uint16_t rem = (uint16_t)(dur - n_bits * te);
        if (rem > tol) n_bits++;

        for (uint16_t b = 0; b < n_bits; b++)
        {
            running = (running << 1) | (uint32_t)(bit_val ? 1u : 0u);

            if (!past_frame) {
                /* Track preamble (alternating 1/0) */
                if (cw_bits > 0 &&
                    ((running & 1) != ((running >> 1) & 1)))
                    alt_run++;
                else
                    alt_run = 1;

                cw_bits++;  /* reuse as total bit counter until frame sync */

                if (alt_run >= POCSAG_MIN_SYNC_BITS &&
                    (running & 0xFFFFFFFFUL) == POCSAG_FRAME_SYNC_CODE)
                {
                    past_frame = true;
                    cw_data   = 0;
                    cw_bits   = 0;
                    continue;
                }
                continue;
            }

            /* After frame sync: accumulate codeword bits */
            cw_data = (cw_data << 1) | (uint64_t)(bit_val ? 1u : 0u);
            cw_bits++;
            if (cw_bits >= 64)
                goto done;
        }
    }

done:
    if (!past_frame || cw_bits < POCSAG_CW_BITS)
        return 1;

    /* Extract first codeword */
    uint32_t cw1 = (cw_bits >= 64)
                   ? (uint32_t)(cw_data >> 32)
                   : (uint32_t)(cw_data & 0xFFFFFFFFUL);

    /* Skip idle codewords */
    if (cw1 == POCSAG_IDLE_CODE_WORD) {
        if (cw_bits < 64) return 1;
        cw1 = (uint32_t)(cw_data & 0xFFFFFFFFUL);
        if (cw1 == POCSAG_IDLE_CODE_WORD) return 1;
    }

    uint32_t ric  = 0;
    uint8_t  func = 0;
    if ((cw1 >> 31) == 0) {
        ric  = (cw1 >> 13) << 3;
        func = (uint8_t)((cw1 >> 11) & 0x03u);
    }

    subghz_decenc_ctl.n64_decodedvalue   = cw_data;
    subghz_decenc_ctl.ndecodedbitlength  = cw_bits;
    subghz_decenc_ctl.ndecodeddelay      = 0;
    subghz_decenc_ctl.ndecodedprotocol   = p;
    subghz_decenc_ctl.n32_serialnumber   = ric;
    subghz_decenc_ctl.n8_buttonid        = func;
    return 0;
}
