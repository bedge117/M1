/* See COPYING.txt for license details. */

/*
 * mfc_crypto1.h - Software MIFARE Classic Crypto-1 cipher
 *
 * Implements the Crypto-1 stream cipher used by MIFARE Classic cards.
 * The ST25R3916 has no hardware Crypto-1 support, so all
 * authentication and encrypted communication is done in software.
 *
 * Reference: "A Practical Attack on the MIFARE Classic" (Garcia et al.)
 */

#ifndef MFC_CRYPTO1_H_
#define MFC_CRYPTO1_H_

#include <stdint.h>
#include <stdbool.h>

/* Crypto-1 LFSR state (48-bit) */
typedef struct {
    uint32_t odd;   /* Odd-numbered LFSR bits  (bits 1,3,5,...,47) = 24 bits */
    uint32_t even;  /* Even-numbered LFSR bits (bits 0,2,4,...,46) = 24 bits */
} crypto1_state_t;

/* Initialize LFSR from 48-bit key */
void crypto1_init(crypto1_state_t *s, uint64_t key);

/* Reset state to zero */
void crypto1_reset(crypto1_state_t *s);

/* Clock the LFSR once with input bit, return filter output */
uint8_t crypto1_bit(crypto1_state_t *s, uint8_t in, uint8_t is_encrypted);

/* Process one byte through the cipher */
uint8_t crypto1_byte(crypto1_state_t *s, uint8_t in, uint8_t is_encrypted);

/* Process 32 bits through the cipher */
uint32_t crypto1_word(crypto1_state_t *s, uint32_t in, uint8_t is_encrypted);

/* Generate parity bit for one byte using cipher stream */
uint8_t crypto1_parity_bit(crypto1_state_t *s);

/* PRNG successor: step the MIFARE Classic PRNG n times */
uint32_t mfc_prng_successor(uint32_t x, uint32_t n);

/* --- High-level MIFARE Classic operations --- */

#define MFC_KEY_LEN         6
#define MFC_BLOCK_SIZE      16
#define MFC_AUTH_CMD_A      0x60
#define MFC_AUTH_CMD_B      0x61

/* Perform MIFARE Classic authentication
 * Returns true on success, false on failure
 * uid:       4-byte UID (or first 4 bytes of 7-byte UID)
 * blockNo:   block number to authenticate
 * keyType:   MFC_AUTH_CMD_A or MFC_AUTH_CMD_B
 * key:       6-byte key
 * state:     output: authenticated Crypto-1 state for subsequent reads
 */
bool mfc_auth(crypto1_state_t *state,
              const uint8_t uid[4],
              uint8_t blockNo,
              uint8_t keyType,
              const uint8_t key[MFC_KEY_LEN]);

/* Read one block using an authenticated Crypto-1 state
 * Returns true on success
 * state:   authenticated Crypto-1 state (modified in place)
 * blockNo: block to read
 * out:     16-byte output buffer
 */
bool mfc_read_block_crypto(crypto1_state_t *state,
                           uint8_t blockNo,
                           uint8_t out[MFC_BLOCK_SIZE]);

#endif /* MFC_CRYPTO1_H_ */
