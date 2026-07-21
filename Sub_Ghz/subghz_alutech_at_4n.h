/* See COPYING.txt for license details. */

/**
 * @file   subghz_alutech_at_4n.h
 * @brief  Alutech AT-4N cipher — hardware-independent pure-logic module.
 *
 * Implements the Alutech AT-4N TEA-variant block cipher used by Alutech
 * garage-door and gate remotes.  The cipher operates on a 64-bit data
 * block and requires eight 32-bit values from a "rainbow table"
 * (32 bytes total; decrypt uses entries [0..5], encrypt uses [0..2, 4..7]).
 *
 * The algorithm is ported from the Flipper Zero firmware
 * (lib/subghz/protocols/alutech_at_4n.c — public source).
 *
 * This file has no hardware dependencies and is testable on the host.
 *
 * Rainbow table layout (32 bytes = 8 × uint32_t, big-endian):
 *   [0] initial sum / final sum for decrypt loop
 *   [1] K0
 *   [2] K1
 *   [3] delta (negative, subtracted each decrypt round)
 *   [4] K2
 *   [5] K3
 *   [6] encrypt step delta (added each encrypt round)
 *   [7] encrypt terminal sum (loop stopper)
 *
 * Plaintext 64-bit block layout (after decrypt, MSB-first):
 *   Byte[0]     CRC check byte = crc8_maxim(cnt & 0xFF)
 *   Byte[1:4]   32-bit serial (big-endian)
 *   Byte[5:6]   16-bit rolling counter (big-endian)
 *   Byte[7]     button code (0xFF=#1, 0x11=#2, 0x22=#3, 0x33=#4, 0x44=#5)
 *
 * On-air format (72 bits, LSB-first):
 *   [63:0]  encrypted 64-bit data (bit-reversed)
 *   [71:64] 8-bit CRC (bit-reversed, poly 0x31 init 0xFF)
 *
 * M1 Project — Hapax fork
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* Rainbow table size                                                          */
/*============================================================================*/

/** Size of the Alutech AT-4N rainbow table in bytes (8 × uint32_t). */
#define ALUTECH_AT_4N_TABLE_SIZE  32

/*============================================================================*/
/* Cipher primitives                                                           */
/*============================================================================*/

/**
 * @brief  Decrypt a 64-bit Alutech AT-4N ciphertext block using the
 *         TEA-variant cipher and the provided rainbow table.
 *
 * @param  data   64-bit ciphertext (MSB-first byte order as stored).
 * @param  table  32-byte rainbow table (must not be NULL).
 *                Uses entries [0..5] (24 bytes / 6 × uint32_t).
 * @return 64-bit plaintext.
 */
uint64_t alutech_at_4n_decrypt(uint64_t data,
                               const uint8_t table[ALUTECH_AT_4N_TABLE_SIZE]);

/**
 * @brief  Encrypt a 64-bit Alutech AT-4N plaintext block using the
 *         TEA-variant cipher and the provided rainbow table.
 *
 * Inverse of @ref alutech_at_4n_decrypt.
 *
 * @param  data   64-bit plaintext (MSB-first byte order).
 * @param  table  32-byte rainbow table (must not be NULL).
 *                Uses entries [0..2, 4..7] (28 bytes / 7 × uint32_t).
 * @return 64-bit ciphertext.
 */
uint64_t alutech_at_4n_encrypt(uint64_t data,
                               const uint8_t table[ALUTECH_AT_4N_TABLE_SIZE]);

/*============================================================================*/
/* CRC helpers                                                                 */
/*============================================================================*/

/**
 * @brief  Compute the CRC-8/MAXIM frame checksum over a 64-bit data block.
 *
 * Polynomial 0x31, initial value 0xFF, no reflection, no final XOR.
 * Used to validate the 8-bit CRC that follows the 64-bit payload on-air.
 *
 * @param  data  64-bit data block (bytes processed MSB-first).
 * @return 8-bit CRC.
 */
uint8_t alutech_at_4n_crc(uint64_t data);

/**
 * @brief  Compute the integrity check byte for the low byte of the counter.
 *
 * CRC-8/MAXIM of a single byte, result bitwise-inverted.
 * The decrypted byte[0] must equal this value for the packet to be valid.
 *
 * @param  cnt_low  Low byte of the 16-bit rolling counter.
 * @return 8-bit integrity marker.
 */
uint8_t alutech_at_4n_decrypt_data_crc(uint8_t cnt_low);

#ifdef __cplusplus
}
#endif
