/* See COPYING.txt for license details. */

/*
 * subghz_secplus_v1_encoder.h
 *
 * Security+ 1.0 ternary OOK packet encoder.
 *
 * Security+ 1.0 uses a ternary (3-symbol) encoding with two sub-packets.
 * This is the inverse of the existing m1_chamberlain_decode.c decoder,
 * implementing the argilo/secplus algorithm for packet construction.
 *
 * References:
 *   argilo/secplus (MIT): https://github.com/argilo/secplus
 *   Flipper Zero secplus_v1.c (GPLv3):
 *     lib/subghz/protocols/secplus_v1.c
 *   M1 decoder: Sub_Ghz/protocols/m1_chamberlain_decode.c
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_SECPLUS_V1_ENCODER_H
#define SUBGHZ_SECPLUS_V1_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Output types                                                                */
/*============================================================================*/

/**
 * A single raw timing pair for Security+ 1.0 transmission.
 * Each pair encodes one OOK symbol: LOW pulse duration + HIGH pulse duration.
 */
typedef struct {
    uint16_t low_us;   /**< LOW  pulse duration (µs) */
    uint16_t high_us;  /**< HIGH pulse duration (µs) */
} SubGhzSecPlusV1Symbol;

/**
 * Complete encoded Security+ 1.0 packet: two sub-packets.
 *
 * Each sub-packet has 1 header symbol + 20 data symbols = 21 symbols.
 * Two sub-packets = 42 symbols total, stored sequentially:
 *   symbols[0..20]  = sub-packet 1 (header sym[0] + data syms[1..20])
 *   symbols[21..41] = sub-packet 2 (header sym[21] + data syms[22..41])
 *
 * The inter-sub-packet gap is not included here; callers insert it using
 * the SUBGHZ_SECPLUS_V1_INTER_PKT_GAP_US constant.
 */
typedef struct {
    SubGhzSecPlusV1Symbol symbols[42];
} SubGhzSecPlusV1Packet;

/*============================================================================*/
/* Timing constants (from argilo/secplus + Flipper secplus_v1.c)              */
/*============================================================================*/

#define SUBGHZ_SECPLUS_V1_TE_SHORT   500u   /**< Short element duration (µs) */
#define SUBGHZ_SECPLUS_V1_TE_MID     1000u  /**< Mid element duration (µs) */
#define SUBGHZ_SECPLUS_V1_TE_LONG    1500u  /**< Long element duration (µs) */

/** Inter-sub-packet gap between sub-packet 1 and sub-packet 2 (µs). */
#define SUBGHZ_SECPLUS_V1_INTER_PKT_GAP_US  58000u

/** Inter-packet gap after sub-packet 2 before next repetition (µs). */
#define SUBGHZ_SECPLUS_V1_END_GAP_US        58000u

/** Number of symbols in each sub-packet (header + 20 data). */
#define SUBGHZ_SECPLUS_V1_SYMS_PER_PKT  21u

/** Total symbols in one complete packet (2 sub-packets). */
#define SUBGHZ_SECPLUS_V1_TOTAL_SYMS    42u

/** Maximum valid rolling counter value (3^20 - 1 = 3486784400). */
#define SUBGHZ_SECPLUS_V1_MAX_ROLLING  3486784400UL

/*============================================================================*/
/* API                                                                         */
/*============================================================================*/

/**
 * Encode a Security+ 1.0 packet from fixed code and rolling counter.
 *
 * Implements the argilo/secplus encoding algorithm (MIT licence) as used in
 * Flipper Zero firmware's secplus_v1.c.
 *
 * Both input values are represented as 20-trit base-3 integers, matching the
 * existing m1_chamberlain_decode.c decoder output.
 *
 * NOTE: The M1 decoder exposes the rolling code after applying reverse32() to
 * the on-air ternary value.  To replay a saved packet whose n32_rollingcode
 * was captured by the decoder, pass reverse32(n32_rollingcode) here.  For a
 * brute-force counter sweep (0 to SUBGHZ_SECPLUS_V1_MAX_ROLLING), pass the
 * counter value directly — it maps to the on-air ternary rolling value.
 *
 * @param fixed    Fixed code as a 20-trit ternary value stored in uint32_t
 *                 (valid range: 0 .. 3^20 - 1 = SUBGHZ_SECPLUS_V1_MAX_ROLLING)
 * @param rolling  Rolling counter as a 20-trit ternary value stored in uint32_t
 *                 (valid range: 0 .. 3^20 - 1 = SUBGHZ_SECPLUS_V1_MAX_ROLLING)
 * @param out      Output packet buffer (caller-allocated)
 * @return true on success, false if any input is invalid
 *         (fixed > 3^20 - 1 or rolling > 3^20 - 1) or out is NULL
 */
bool subghz_secplus_v1_encode(uint32_t fixed, uint32_t rolling,
                               SubGhzSecPlusV1Packet *out);

#endif /* SUBGHZ_SECPLUS_V1_ENCODER_H */
