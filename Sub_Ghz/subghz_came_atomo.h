/* See COPYING.txt for license details. */

/**
 * @file   subghz_came_atomo.h
 * @brief  CAME Atomo cipher — hardware-independent pure-logic module.
 *
 * Implements the CAME Atomo LFSR-based stream cipher used by CAME Atomo
 * garage-door and gate remotes.  The cipher operates on an 8-byte (64-bit)
 * buffer and requires no external key material — the LFSR seed is derived
 * entirely from the first byte of the buffer.
 *
 * The algorithm is ported from the DarkFlippers/unleashed-firmware
 * (lib/subghz/protocols/came_atomo.c — public source).
 *
 * This file has no hardware dependencies and is testable on the host.
 *
 * Over-the-air format (62 Manchester bits → 60 significant bits):
 *   Transmitted data = (~plaintext64 >> 4)  (bitwise invert, right-shift 4)
 *
 * Plaintext 64-bit block layout:
 *   [63:57] cnt_2   — 7-bit hold-cycle counter (0x00–0x7F)
 *   [56]    always 0
 *   [55:40] cnt     — 16-bit rolling counter
 *   [39: 8] serial  — 32-bit device serial
 *   [ 7: 4] btn     — 4-bit button code (upper nibble of last byte)
 *   [ 3: 0] always 0
 *
 * M1 Project — Hapax fork
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* Cipher primitives                                                           */
/*============================================================================*/

/**
 * @brief  Encrypt an 8-byte CAME Atomo plaintext block in place.
 *
 * The LFSR seed is derived from buff[0].  Bits 8–58 of the 64-bit
 * block are XOR'd with the LFSR keystream.  buff[0] is additionally
 * XOR'd with 0x05 and masked to 7 bits.
 *
 * @param  buff  8-byte buffer (modified in place).
 */
void came_atomo_encrypt(uint8_t buff[8]);

/**
 * @brief  Decrypt an 8-byte CAME Atomo ciphertext block in place.
 *
 * Exact inverse of @ref came_atomo_encrypt.  Recovers the plaintext
 * serial, counter, and button code from the encrypted payload.
 *
 * @param  buff  8-byte buffer (modified in place).
 */
void came_atomo_decrypt(uint8_t buff[8]);

#ifdef __cplusplus
}
#endif
