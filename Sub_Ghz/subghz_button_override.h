/* See COPYING.txt for license details. */

/**
 * @file   subghz_button_override.h
 * @brief  Sub-GHz button-override key mutation — pure logic.
 *
 * Given a (protocol, key_value, button_index) tuple, replaces the
 * protocol's button-field bits inside the 64-bit key value with the
 * requested button index, returning the new key.  This is the
 * "actually mutate the transmitted key" backend behind the Transmitter
 * scene's LEFT/RIGHT button-cycling.
 *
 * Coverage rule:  Only protocols where the button is encoded as a
 * known, plain-bit field within the Flipper SubGhz Key value are
 * supported.  Protocols whose button is encoded via an opaque
 * manufacturer lookup table (Nice FloR-S, FAAC SLH) or as an encrypted
 * payload byte (CAME Atomo, Alutech AT-4N, KingGates Stylo4k, DITEC
 * GOL4, Toyota, Scher-Khan) are intentionally NOT supported here —
 * naïve bit-replacement would produce a non-decodable signal.  Those
 * will require per-protocol re-encoders in follow-up phases.
 *
 * Supported protocols (KeeLoq family, 4-bit button field):
 *   - KeeLoq     — button at bits [63:60] (upper nibble of FIX half)
 *   - Jarolift   — button at bits [63:60] (same layout as KeeLoq)
 *   - Star Line  — button at bits [ 3: 0] (lower nibble)
 *
 * For supported protocols, the KeeLoq counter-mode encoder will
 * re-encrypt the hop word with the new button when the converter
 * passes the mutated key into keeloq_encode_replay().  For Star Line,
 * the button bits are part of the FIX portion which is transmitted
 * verbatim — naïve bit-replacement is correct.
 *
 * This module is hardware-independent and host-testable.  It does NOT
 * touch hardware, FatFS, RTOS, or display state.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Replace the button-field bits in a captured key value with
 *         the requested button index.
 *
 * @param  protocol     Canonical protocol name string (e.g. "KeeLoq",
 *                      "Star Line", "Jarolift").  Case-insensitive.
 *                      May contain a trailing space; NULL is treated
 *                      as unsupported.
 * @param  key_in       Original 64-bit key value as parsed from the
 *                      Flipper .sub `Key:` field.
 * @param  button_index Requested button index, 0..15.  Higher 4 bits
 *                      are ignored (mask 0x0F).
 * @param  key_out      Out-parameter for the mutated key.  Must not
 *                      be NULL.  On failure, *key_out is set to
 *                      key_in (the caller can use it verbatim).
 *
 * @retval true   The protocol is supported and *key_out has been
 *                mutated to encode the requested button.
 * @retval false  The protocol is unsupported (or @p key_out is NULL).
 *                *key_out is set to key_in if @p key_out is non-NULL.
 *
 * @note  A return of @c false is the canonical signal for the caller
 *        to skip key mutation and either transmit the original key
 *        (legacy Phase 4b behaviour) or display a "cycling disabled"
 *        indicator.  Returning @c false is NOT an error — it is the
 *        expected behaviour for protocols outside the supported set.
 */
bool subghz_button_override_apply(const char *protocol,
                                  uint64_t    key_in,
                                  uint8_t     button_index,
                                  uint64_t   *key_out);

/**
 * @brief  Predicate: does this protocol have a plain-bit button field
 *         that this module can mutate?
 *
 * Useful for the Transmitter scene to decide whether LEFT/RIGHT
 * cycling should actually mutate the key (and require a TX
 * re-prepare) or be a no-op UI indicator.
 *
 * @param  protocol  Protocol name string.  Case-insensitive.  NULL → false.
 * @retval true      Protocol is in the supported set
 *                   ({KeeLoq, Jarolift, Star Line}).
 * @retval false     Protocol is not supported.
 */
bool subghz_button_override_supports(const char *protocol);

#ifdef __cplusplus
}
#endif
