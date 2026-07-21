/* See COPYING.txt for license details. */

/*
 * subghz_secplus_v1_encoder.c
 *
 * Security+ 1.0 ternary OOK packet encoder.
 *
 * This is the inverse of m1_chamberlain_decode.c: given a known fixed code
 * and rolling counter, it produces the two-sub-packet ternary OOK signal
 * that a Security+ 1.0 remote would transmit.
 *
 * Algorithm: argilo/secplus (MIT licence)
 *   https://github.com/argilo/secplus
 * Implementation adapted from Flipper Zero firmware (GPLv3):
 *   lib/subghz/protocols/secplus_v1.c
 *
 * Symbol encoding (OOK, each symbol = LOW + HIGH pair):
 *   Symbol 0: LOW 1500 µs + HIGH  500 µs   (3:1)
 *   Symbol 1: LOW 1000 µs + HIGH 1000 µs   (2:2)
 *   Symbol 2: LOW  500 µs + HIGH 1500 µs   (1:3)
 *
 * Sub-packet 1 header: Symbol 0  (PKT1_HEADER)
 * Sub-packet 2 header: Symbol 2  (PKT2_HEADER)
 * Data: 20 symbols per sub-packet.
 *
 * Encoding loop (inverse of decode, from Flipper secplus_v1.c):
 *   Working from LSBs of ternary-encoded rolling + fixed:
 *   Packet 1: pairs at odd indices 1,3,5,…,19 in data_array
 *     data_array[i]   = rolling_digit (mod 3)
 *     data_array[i+1] = (fixed_digit + acc) % 3
 *   Packet 2: pairs at even indices 0,2,4,…,18 in data_array
 *     same pattern
 *
 * M1 Project — Hapax fork
 */

#include <string.h>
#include "subghz_secplus_v1_encoder.h"
#include "../../m1_csrc/bit_util.h"

/* ── Symbol timing helpers ──────────────────────────────────────────────── */

/*
 * Fill one symbol slot.
 * sym: 0 → (1500 LOW, 500 HIGH), 1 → (1000, 1000), 2 → (500, 1500)
 */
static void fill_symbol(SubGhzSecPlusV1Symbol *s, uint8_t sym)
{
    switch (sym)
    {
        case 0:
            s->low_us  = SUBGHZ_SECPLUS_V1_TE_LONG;
            s->high_us = SUBGHZ_SECPLUS_V1_TE_SHORT;
            break;
        case 1:
            s->low_us  = SUBGHZ_SECPLUS_V1_TE_MID;
            s->high_us = SUBGHZ_SECPLUS_V1_TE_MID;
            break;
        case 2:
        default:
            s->low_us  = SUBGHZ_SECPLUS_V1_TE_SHORT;
            s->high_us = SUBGHZ_SECPLUS_V1_TE_LONG;
            break;
    }
}

/* ── Ternary decomposition ──────────────────────────────────────────────── */

/*
 * Decompose a 20-trit ternary number into an array of trits.
 * trits[0] is the MOST significant trit (for MSB-first encoding).
 * total: the numeric value (must be < 3^20)
 * trits: output array of 20 uint8_t values (0, 1, or 2)
 */
static void decompose_trits(uint32_t total, uint8_t trits[20])
{
    for (int8_t i = 19; i >= 0; i--)
    {
        trits[i] = (uint8_t)(total % 3u);
        total /= 3u;
    }
}

/* ── Public encoder ─────────────────────────────────────────────────────── */

bool subghz_secplus_v1_encode(uint32_t fixed, uint32_t rolling,
                               SubGhzSecPlusV1Packet *out)
{
    if (!out)
        return false;

    /* Validate inputs: both fixed and rolling must fit in 20 ternary digits */
    if (fixed > SUBGHZ_SECPLUS_V1_MAX_ROLLING || rolling > SUBGHZ_SECPLUS_V1_MAX_ROLLING)
        return false;

    /*
     * The encoder writes the same trit pairs expected by the decoder:
     *
     *   Packet 1 uses the first 10 trit pairs and packet 2 uses the next 10.
     *   In each pair, the rolling trit is emitted first, then the fixed trit
     *   is emitted in accumulated form:
     *
     *     out_roll = rolling_trit
     *     acc += rolling_trit
     *     out_fixed = (fixed_trit + acc) % 3
     *     acc += fixed_trit
     *
     *   Packet 2 repeats the same process with acc reset to 0.
     *   This is the direct inverse of the decoder's fixed-trit recovery step.
     *
     *   The loops below serialize those symbols into:
     *     [PKT1_HEADER][20 data symbols][PKT2_HEADER][20 data symbols]
     */

    /* Decompose fixed and rolling into 20 ternary digits each, MSB first */
    uint8_t rolling_trits[20];
    uint8_t fixed_trits[20];
    decompose_trits(rolling, rolling_trits);
    decompose_trits(fixed,   fixed_trits);

    /* Build data_array[42] */
    uint8_t da[42];
    memset(da, 0, sizeof(da));
    da[0]  = 0; /* PKT1_HEADER = Symbol 0 */
    da[21] = 2; /* PKT2_HEADER = Symbol 2 */

    uint32_t acc;

    /* Packet 1: pairs at positions [1,2], [3,4], … [19,20] → 10 rolling+fixed pairs */
    acc = 0;
    for (uint8_t k = 0; k < 10; k++)
    {
        uint8_t idx = (uint8_t)(1u + k * 2u);
        da[idx]     = rolling_trits[k];
        acc        += rolling_trits[k];
        da[idx + 1] = (uint8_t)((fixed_trits[k] + acc) % 3u);
        acc        += fixed_trits[k];
    }

    /* Packet 2: pairs at positions [22,23], [24,25], … [40,41] → 10 rolling+fixed pairs */
    acc = 0;
    for (uint8_t k = 0; k < 10; k++)
    {
        uint8_t idx = (uint8_t)(22u + k * 2u);
        da[idx]     = rolling_trits[10u + k];
        acc        += rolling_trits[10u + k];
        da[idx + 1] = (uint8_t)((fixed_trits[10u + k] + acc) % 3u);
        acc        += fixed_trits[10u + k];
    }

    /* Convert data_array symbols to timing pairs */
    for (uint8_t i = 0; i < 42u; i++)
    {
        fill_symbol(&out->symbols[i], da[i]);
    }

    return true;
}
