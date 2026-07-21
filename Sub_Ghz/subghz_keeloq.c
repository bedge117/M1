/* See COPYING.txt for license details. */

/**
 * @file  subghz_keeloq.c
 * @brief KeeLoq block cipher — hardware-independent pure-logic implementation.
 *
 * Implements the 528-round NLFSR cipher from Microchip AN66903.
 * Compatible with the DarkFlippers/unleashed-firmware keeloq_common.c
 * reference implementation.
 */

#include "subghz_keeloq.h"

/*============================================================================*/
/* KeeLoq NLF lookup constant and helpers                                     */
/*============================================================================*/

/**
 * KeeLoq Non-Linear Function (NLF) constant.
 * The bit at position g5(x) in this 32-bit word is the NLF output.
 * g5 selects 5 bits from the NLFSR state to form a 5-bit index.
 */
#define KEELOQ_NLF  0x3A5C742EUL

/** Extract bit @p n from value @p x. */
#define BIT(x, n)   (((x) >> (n)) & 1U)

/**
 * Build the 5-bit NLF index from five specified bit positions of @p x.
 */
static inline uint32_t g5(uint32_t x, int a, int b, int c, int d, int e)
{
    return BIT(x, a) | (BIT(x, b) << 1) | (BIT(x, c) << 2) |
           (BIT(x, d) << 3) | (BIT(x, e) << 4);
}

/*============================================================================*/
/* Cipher primitives                                                           */
/*============================================================================*/

uint32_t keeloq_encrypt(uint32_t data, uint64_t key)
{
    uint32_t x = data;
    uint32_t r;

    for (r = 0; r < 528; r++)
    {
        uint32_t bit = BIT(x, 0) ^ BIT(x, 16) ^
                       (uint32_t)(key >> (r & 63) & 1) ^
                       BIT(KEELOQ_NLF, g5(x, 1, 9, 20, 26, 31));
        x = (x >> 1) ^ (bit << 31);
    }

    return x;
}

uint32_t keeloq_decrypt(uint32_t data, uint64_t key)
{
    uint32_t x = data;
    uint32_t r;

    for (r = 0; r < 528; r++)
    {
        uint32_t bit = BIT(x, 31) ^ BIT(x, 15) ^
                       (uint32_t)(key >> ((15 - r) & 63) & 1) ^
                       BIT(KEELOQ_NLF, g5(x, 0, 8, 19, 25, 30));
        x = (x << 1) ^ bit;
    }

    return x;
}

/*============================================================================*/
/* Device-key derivation (learning modes)                                     */
/*============================================================================*/

uint64_t keeloq_learn_normal(uint32_t serial, uint64_t mfr_key)
{
    uint32_t s = serial & 0x0FFFFFFFU;
    uint32_t k1 = keeloq_decrypt(s | 0x20000000U, mfr_key);
    uint32_t k2 = keeloq_decrypt(s | 0x60000000U, mfr_key);
    return ((uint64_t)k2 << 32) | k1;
}

uint64_t keeloq_learn_simple(uint32_t serial, uint64_t mfr_key)
{
    /*
     * Simple learning: XOR the 64-bit MK with a serial-derived 64-bit value.
     * The serial is replicated into a 64-bit pattern used by many low-cost
     * remotes (Cardin S449, some CAME variants, etc.).
     */
    uint64_t s = (uint64_t)(serial & 0x0FFFFFFFU);
    uint64_t sv = s | (s << 32);
    return mfr_key ^ sv;
}

uint64_t keeloq_learn_secure(uint32_t serial, uint32_t seed, uint64_t mfr_key)
{
    uint32_t k1 = keeloq_decrypt(serial & 0x0FFFFFFFU, mfr_key);
    uint32_t k2 = keeloq_decrypt(seed, mfr_key);
    return ((uint64_t)k1 << 32) | k2;
}

uint64_t keeloq_learn_magic_xor_type1(uint32_t serial, uint64_t mfr_key)
{
    uint64_t s = (uint64_t)(serial & 0x0FFFFFFFU);
    return ((s << 32) | s) ^ mfr_key;
}

uint64_t keeloq_learn_faac_slh(uint32_t seed, uint64_t mfr_key)
{
    uint16_t hs = (uint16_t)(seed >> 16);
    uint32_t lsb = ((uint32_t)hs << 16) | 0x544DU;
    uint64_t hi = (uint64_t)keeloq_encrypt(seed, mfr_key);
    uint64_t lo = (uint64_t)keeloq_encrypt(lsb, mfr_key);
    return (hi << 32) | lo;
}

/*============================================================================*/
/* Counter-mode increment                                                     */
/*============================================================================*/

uint32_t keeloq_increment_hop(uint32_t hop_enc, uint64_t device_key)
{
    /* Decrypt to get plaintext hop word */
    uint32_t plain = keeloq_decrypt(hop_enc, device_key);

    /* Extract and increment the 16-bit counter (bits [31:16]) */
    uint16_t counter = (uint16_t)(plain >> 16);
    counter++;

    /* Reconstruct plaintext with incremented counter, preserve lower 16 bits */
    uint32_t new_plain = ((uint32_t)counter << 16) | (plain & 0x0000FFFFU);

    /* Re-encrypt with the same device key */
    return keeloq_encrypt(new_plain, device_key);
}
