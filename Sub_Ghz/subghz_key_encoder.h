/* See COPYING.txt for license details. */

/*
 * subghz_key_encoder.h
 *
 * Extract from sub_ghz_replay_flipper_file() — the KEY→RAW OOK PWM
 * encoding logic.  Converts a protocol name + key value + bit count
 * into raw timing samples using the protocol registry for timing lookup.
 *
 * This module is hardware-independent and compiles on both ARM and host.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_KEY_ENCODER_H
#define SUBGHZ_KEY_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Key Encoding Parameters                                                     */
/*============================================================================*/

/** Parsed KEY file fields — input to the encoder */
typedef struct {
    char     protocol[32];    /**< Protocol name (e.g. "Princeton") */
    uint64_t key_value;       /**< Key data, MSB-first */
    uint32_t bit_count;       /**< Number of bits to encode */
    uint32_t te;              /**< Short pulse duration override (0 = use registry default) */
} SubGhzKeyParams;

/** Timing parameters resolved from the registry + fallbacks */
typedef struct {
    uint32_t te_short;        /**< Short symbol duration (μs) */
    uint32_t te_long;         /**< Long symbol duration (μs) */
    uint32_t gap_low;         /**< Sync gap LOW duration (μs) */
} SubGhzKeyTiming;

/** Single raw timing sample: duration in microseconds */
typedef struct {
    uint32_t high_us;         /**< HIGH pulse duration */
    uint32_t low_us;          /**< LOW pulse duration */
} SubGhzRawPair;

/*============================================================================*/
/* Return codes                                                                */
/*============================================================================*/

#define SUBGHZ_KEY_OK             0   /**< Success */
#define SUBGHZ_KEY_ERR_DYNAMIC    6   /**< Not re-encodable: KeeLoq (needs manufacturer key),
                                           Security+ 1.0/2.0 (ternary/FSK encoding),
                                           Hormann BiSecur (Manchester+AES),
                                           Beninca ARC (AES-128), Jarolift (KeeLoq-based),
                                           Weather sensors, TPMS.
                                           Other Dynamic (rolling code) protocols that use
                                           standard OOK PWM encoding are now allowed. */
#define SUBGHZ_KEY_ERR_UNSUPPORTED 7  /**< Unsupported protocol (not in registry, no fallback) */
#define SUBGHZ_KEY_ERR_OVERFLOW   8   /**< Output buffer too small */

/*============================================================================*/
/* Functions                                                                   */
/*============================================================================*/

/**
 * Resolve timing parameters for a KEY file's protocol.
 *
 * Looks up the protocol in the registry, applies strstr() fallbacks for
 * third-party protocols, and validates that the protocol type is encodable.
 *
 * @param params   Input key parameters (protocol name, te override)
 * @param timing   Output timing parameters
 * @return SUBGHZ_KEY_OK on success, SUBGHZ_KEY_ERR_* on failure
 */
uint8_t subghz_key_resolve_timing(const SubGhzKeyParams *params, SubGhzKeyTiming *timing);

/**
 * Encode a key value as OOK PWM raw timing pairs.
 *
 * Generates one repetition of the encoded signal:
 *   - bit_count pairs (one per bit, MSB first)
 *   - 1 sync gap pair at the end
 *   Total = bit_count + 1 pairs per repetition
 *
 * Bit encoding:
 *   - Bit 1: HIGH=te_long, LOW=te_short
 *   - Bit 0: HIGH=te_short, LOW=te_long
 *   - Sync:  HIGH=te_short, LOW=gap_low
 *
 * @param params       Input key parameters
 * @param timing       Resolved timing (from subghz_key_resolve_timing)
 * @param out          Output array of raw pairs
 * @param max_pairs    Size of output array
 * @param repetitions  Number of full signal repetitions (typically 3)
 * @return Number of pairs written, or 0 on error
 */
uint32_t subghz_key_encode(const SubGhzKeyParams *params,
                            const SubGhzKeyTiming *timing,
                            SubGhzRawPair *out,
                            uint32_t max_pairs,
                            uint8_t repetitions);

/**
 * Check if a protocol requires a protocol-specific encoder.
 *
 * Some protocols (e.g. Magellan) use non-standard encoding that cannot
 * be handled by the generic OOK PWM encoder (inverted bit polarity,
 * custom preambles, start/stop bits).
 *
 * @param protocol  Protocol name
 * @return true if a protocol-specific encoder exists
 */
bool subghz_key_has_custom_encoder(const char *protocol);

/**
 * Encode a key value using a protocol-specific encoder.
 *
 * Currently supports: Magellan
 *
 * Call subghz_key_has_custom_encoder() first to check availability.
 *
 * @param params       Input key parameters (protocol, key, bit_count)
 * @param out          Output array of raw pairs
 * @param max_pairs    Size of output array
 * @param repetitions  Number of full signal repetitions
 * @return Number of pairs written, or 0 on error
 */
uint32_t subghz_key_encode_custom(const SubGhzKeyParams *params,
                                   SubGhzRawPair *out,
                                   uint32_t max_pairs,
                                   uint8_t repetitions);

/**
 * Calculate the number of in-file repetitions needed for a low-TE signal.
 *
 * For signals with TE < 250µs, a single repetition is too short for reliable
 * receiver detection.  This helper scales the repetition count so that each
 * DMA burst carries approximately the same total duration as a TE=370 signal
 * with 3 repetitions (~140ms).
 *
 * Formula: ceil(3 * 370 / key_te), clamped to [3, 12].
 * For key_te >= 250 (or key_te == 0), returns 3 (the default).
 *
 * @param key_te  Short pulse duration in µs (from the .sub KEY file)
 * @return        Number of repetitions to embed in the encoded raw file
 */
uint8_t subghz_low_te_calc_reps(uint32_t key_te);

/**
 * Return the number of raw pairs needed by the custom encoder for @p params
 * and @p repetitions, without writing any data.
 *
 * This decouples the caller from protocol-specific size constants: the
 * caller can allocate exactly the right buffer regardless of which custom
 * encoder is selected.
 *
 * Returns 0 if @p params has no custom encoder or has an invalid bit_count.
 */
uint32_t subghz_key_custom_required_pairs(const SubGhzKeyParams *params,
                                           uint8_t repetitions);

/** Number of pairs per Magellan repetition:
 *  1 (header burst) + 12 (header toggles) + 1 (header end) +
 *  1 (start bit) + 32 (data bits) + 1 (stop bit) = 48 */
#define SUBGHZ_MAGELLAN_PAIRS_PER_REP  48

#endif /* SUBGHZ_KEY_ENCODER_H */
