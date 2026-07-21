/* See COPYING.txt for license details. */

/**
 * @file   subghz_keeloq.h
 * @brief  KeeLoq block cipher — hardware-independent pure-logic module.
 *
 * Implements the KeeLoq NLFSR cipher (Microchip AN66903 / AN1200.02) plus
 * the three most common device-key derivation (learning) modes used by
 * garage-door and automotive rolling-code remotes.
 *
 * This file has no hardware dependencies and is testable on the host.
 *
 * References:
 *   Microchip AN1200.02 — KeeLoq Code Hopping Encoder & Decoder
 *   Microchip AN66903 — KeeLoq Algorithm Description
 *   DarkFlippers/unleashed-firmware — lib/subghz/protocols/keeloq_common.c
 *   Nohl, Starbug, Plötz — "Dismantling KeeLoq" (EUROCRYPT 2008)
 */

#pragma once

#include <stdint.h>

/*============================================================================*/
/* KeeLoq cipher primitives                                                   */
/*============================================================================*/

/**
 * @brief  Encrypt a 32-bit plaintext block with a 64-bit key.
 *         Runs 528 forward NLFSR rounds.
 * @param  data  32-bit plaintext
 * @param  key   64-bit round key (used cyclically)
 * @return 32-bit ciphertext
 */
uint32_t keeloq_encrypt(uint32_t data, uint64_t key);

/**
 * @brief  Decrypt a 32-bit ciphertext block with a 64-bit key.
 *         Runs 528 inverse NLFSR rounds.
 * @param  data  32-bit ciphertext
 * @param  key   64-bit round key (used cyclically, reversed)
 * @return 32-bit plaintext
 */
uint32_t keeloq_decrypt(uint32_t data, uint64_t key);

/*============================================================================*/
/* Device-key derivation (learning modes)                                     */
/*============================================================================*/

/**
 * @brief  Normal Learning — derive a 64-bit device key from the serial number
 *         and the manufacturer master key.
 *
 * Used by most garage-door openers (Chamberlain, BFT, Novoferm, Marantec,
 * Ditec, AN-Motors, Cardin, Stilmatic, Sommer, EcoStar, etc.).
 *
 * Algorithm (Flipper-compatible):
 *   k1 = keeloq_decrypt(serial | 0x20000000, mfr_key)
 *   k2 = keeloq_decrypt(serial | 0x60000000, mfr_key)
 *   device_key = (k2 << 32) | k1
 *
 * @param  serial   28-bit remote serial number (bits 27:0 used)
 * @param  mfr_key  64-bit manufacturer master key
 * @return 64-bit device key
 */
uint64_t keeloq_learn_normal(uint32_t serial, uint64_t mfr_key);

/**
 * @brief  Simple Learning — derive a 64-bit device key by XOR of the serial
 *         and manufacturer master key.
 *
 * Used by some older/cheaper devices.
 *
 * Algorithm:
 *   device_key = mfr_key ^ (serial | (serial << 16))  [see source]
 *
 * @param  serial   28-bit remote serial number
 * @param  mfr_key  64-bit manufacturer master key
 * @return 64-bit device key
 */
uint64_t keeloq_learn_simple(uint32_t serial, uint64_t mfr_key);

/**
 * @brief  Secure Learning — derive a 64-bit device key using the serial and a
 *         seed value.  Used by some high-end remotes.
 *
 * Algorithm:
 *   k1 = keeloq_decrypt(serial & 0x0FFFFFFF, mfr_key)
 *   k2 = keeloq_decrypt(seed, mfr_key)
 *   device_key = (k1 << 32) | k2
 *
 * @param  serial   28-bit remote serial number
 * @param  seed     32-bit seed (from the receiver's stored seed)
 * @param  mfr_key  64-bit manufacturer master key
 * @return 64-bit device key
 */
uint64_t keeloq_learn_secure(uint32_t serial, uint32_t seed, uint64_t mfr_key);

/**
 * @brief  Magic XOR Type 1 Learning — derive a 64-bit device key by XOR of
 *         the replicated serial and the manufacturer key.
 *
 * Used by Beninca and some other manufacturers.
 *
 * Algorithm (Flipper-compatible):
 *   device_key = ((serial << 32) | serial) ^ mfr_key
 *
 * @param  serial   28-bit remote serial number (bits 27:0 used)
 * @param  mfr_key  64-bit manufacturer master key
 * @return 64-bit device key
 */
uint64_t keeloq_learn_magic_xor_type1(uint32_t serial, uint64_t mfr_key);

/**
 * @brief  FAAC SLH Learning — derive a 64-bit device key from a seed and the
 *         manufacturer key using KeeLoq encryption.
 *
 * Used by FAAC SLH (Self-Learning Hopping) remotes.
 *
 * Algorithm (Flipper-compatible):
 *   hs  = seed >> 16
 *   lsb = (hs << 16) | 0x544D
 *   device_key = (keeloq_encrypt(seed, mfr_key) << 32) |
 *                 keeloq_encrypt(lsb, mfr_key)
 *
 * @param  seed     32-bit seed value (from captured data)
 * @param  mfr_key  64-bit manufacturer master key
 * @return 64-bit device key
 */
uint64_t keeloq_learn_faac_slh(uint32_t seed, uint64_t mfr_key);

/*============================================================================*/
/* Counter-mode rolling code increment                                        */
/*============================================================================*/

/**
 * @brief  KeeLoq counter-mode rolling code increment.
 *
 * Decrypts the current encrypted hop word using the device key, increments
 * the 16-bit counter field, and re-encrypts to produce the next valid hop
 * word.  This is the core operation for KeeLoq counter-mode replay.
 *
 * Plaintext HOP word layout (Microchip HCS301, 32-bit):
 *   [31:16] 16-bit rolling counter
 *   [15:12]  4-bit button code (discriminant)
 *   [11:10]  2-bit VLOW / battery flag
 *   [ 9: 4]  6-bit discriminant (low 6 bits of serial)
 *   [ 3: 0]  4-bit overflow counter
 *
 * @param  hop_enc     32-bit encrypted hop word (from captured packet)
 * @param  device_key  64-bit device key (from keeloq_learn_*)
 * @return             32-bit new encrypted hop word (counter + 1)
 */
uint32_t keeloq_increment_hop(uint32_t hop_enc, uint64_t device_key);
