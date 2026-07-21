/* See COPYING.txt for license details. */

/**
 * @file   subghz_nice_flor_s.h
 * @brief  Nice FloR-S cipher — hardware-independent pure-logic module.
 *
 * Implements the Nice FloR-S (and Nice One) rolling-code cipher used by
 * Nice garage-door and gate remotes.  The cipher operates on a 52-bit
 * over-the-air packet whose encrypted payload (44 bits) carries the
 * 28-bit serial and 16-bit rolling counter.
 *
 * The cipher requires a 32-byte "rainbow table" that maps 5-bit indices
 * to substitution bytes.  This table is device-family-specific and must
 * be loaded at runtime (analogous to KeeLoq manufacturer keys).  The
 * algorithm itself is ported from the Flipper Zero firmware
 * (lib/subghz/protocols/nice_flor_s.c — public source).
 *
 * This file has no hardware dependencies and is testable on the host.
 *
 * Packet wire format (52-bit Nice FloR-S):
 *   [51:48] P0 — 4-bit button positional code (plaintext)
 *   [47:44] P1 — 4-bit repetition counter    (plaintext)
 *   [43: 0] encrypted payload (44 bits = 6 bytes, high nibble masked)
 *
 * Decrypted payload layout:
 *   [43:16] 28-bit serial number
 *   [15: 0] 16-bit rolling counter
 *
 * M1 Project — Hapax fork
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* Rainbow table size                                                          */
/*============================================================================*/

/** Size of the Nice FloR-S rainbow/permutation table in bytes. */
#define NICE_FLOR_S_TABLE_SIZE  32

/*============================================================================*/
/* Cipher primitives                                                           */
/*============================================================================*/

/**
 * @brief  Encrypt a 44-bit plaintext payload (serial << 16 | counter) using
 *         the Nice FloR-S cipher and the provided rainbow table.
 *
 * The input @p data uses the lower 44 bits of the uint64_t:
 *   data = ((uint64_t)serial << 16) | counter
 *
 * The upper 20 bits of @p data are ignored and zeroed in the output.
 *
 * @param  data   44-bit plaintext (serial << 16 | counter)
 * @param  table  32-byte rainbow table (must not be NULL)
 * @return 44-bit ciphertext (upper 20 bits zero)
 */
uint64_t nice_flor_s_encrypt(uint64_t data, const uint8_t table[NICE_FLOR_S_TABLE_SIZE]);

/**
 * @brief  Decrypt a 44-bit ciphertext payload using the Nice FloR-S cipher
 *         and the provided rainbow table.
 *
 * The inverse of @ref nice_flor_s_encrypt.  Recovers the plaintext
 * serial and counter from the encrypted payload stored in the .sub file.
 *
 * @param  data   44-bit ciphertext (from bits [43:0] of the 52-bit key)
 * @param  table  32-byte rainbow table (must not be NULL)
 * @return 44-bit plaintext: serial in [43:16], counter in [15:0]
 */
uint64_t nice_flor_s_decrypt(uint64_t data, const uint8_t table[NICE_FLOR_S_TABLE_SIZE]);

#ifdef __cplusplus
}
#endif
