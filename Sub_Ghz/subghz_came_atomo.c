/* See COPYING.txt for license details. */

/**
 * @file   subghz_came_atomo.c
 * @brief  CAME Atomo cipher — encrypt / decrypt implementation.
 *
 * Ported from DarkFlippers/unleashed-firmware
 * (lib/subghz/protocols/came_atomo.c).
 *
 * The algorithm uses a 7-bit LFSR (Linear Feedback Shift Register) whose
 * seed is derived from the first byte of the 8-byte block.  51 keystream
 * bits (positions 8–58) are XOR'd into the buffer.  The first byte is
 * additionally XOR'd with 0x05 and masked to 7 bits.
 *
 * No external key material is required — the cipher is entirely
 * self-contained.
 *
 * M1 Project — Hapax fork
 */

#include "subghz_came_atomo.h"

/*============================================================================*/
/* LFSR core — shared by encrypt and decrypt                                   */
/*============================================================================*/

/**
 * Run the 7-bit LFSR over bits 8..58 of @p buff, XOR-ing each generated
 * keystream bit into the corresponding buffer position.
 *
 * The LFSR feedback taps check bits 3 and 4 (mask 0x18):
 *   if both are set AND the quotient condition ((tmpB/8)&3) != 3 holds,
 *   the new LSB is 1; otherwise 0.
 * The MSB (bit 7) of the LFSR state is the keystream bit.
 */
static void lfsr_xor(uint8_t *buff, uint8_t seed)
{
    uint8_t tmpB   = seed;
    uint8_t bitCnt = 8;

    while (bitCnt < 59)
    {
        if ((tmpB & 0x18U) && (((tmpB / 8U) & 3U) != 3U))
            tmpB = (uint8_t)(((tmpB << 1) & 0xFFU) | 1U);
        else
            tmpB = (uint8_t)((tmpB << 1) & 0xFFU);

        if (tmpB & 0x80U)
            buff[bitCnt / 8U] ^= (uint8_t)(0x80U >> (bitCnt & 7U));

        bitCnt++;
    }
}

/*============================================================================*/
/* Encrypt                                                                     */
/*============================================================================*/

void came_atomo_encrypt(uint8_t buff[8])
{
    /* Derive the LFSR seed from buff[0]: two's complement, masked to 7 bits.
     * (~buff[0] + 1) & 0x7F  ≡  (-buff[0]) & 0x7F                        */
    const uint8_t seed = (uint8_t)((~buff[0] + 1U) & 0x7FU);

    lfsr_xor(buff, seed);

    /* Finalize buff[0]: XOR with 0x05, mask to 7 bits. */
    buff[0] = (uint8_t)((buff[0] ^ 0x05U) & 0x7FU);
}

/*============================================================================*/
/* Decrypt                                                                     */
/*============================================================================*/

void came_atomo_decrypt(uint8_t buff[8])
{
    /* Undo the buff[0] finalization first. */
    buff[0] = (uint8_t)((buff[0] ^ 0x05U) & 0x7FU);

    /* Re-derive the seed.  For decrypt the upstream uses (-buff[0]) & 0x7F
     * which is numerically identical to the encrypt seed derivation
     * (~buff[0] + 1) & 0x7F — both are two's complement negation. */
    const uint8_t seed = (uint8_t)((-buff[0]) & 0x7FU);

    /* XOR is self-inverse: applying the same keystream undoes the cipher. */
    lfsr_xor(buff, seed);
}
