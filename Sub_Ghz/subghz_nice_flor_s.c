/* See COPYING.txt for license details. */

/**
 * @file   subghz_nice_flor_s.c
 * @brief  Nice FloR-S cipher — encrypt / decrypt implementation.
 *
 * Ported from Flipper Zero firmware (lib/subghz/protocols/nice_flor_s.c).
 * The algorithm uses a 32-byte rainbow/permutation table to perform two
 * rounds of substitution-XOR operations followed by a byte-level
 * permutation with bitwise complement.
 *
 * M1 Project — Hapax fork
 */

#include "subghz_nice_flor_s.h"

/*============================================================================*/
/* Internal helpers                                                            */
/*============================================================================*/

/**
 * XOR bytes p[1]..p[5] (the encrypted body) with a round key @p k.
 * p[0] is the index byte and is handled separately by the caller.
 */
static inline void magic_xor(uint8_t *p, uint8_t k)
{
    for (int i = 1; i < 6; ++i)
        p[i] ^= k;
}

/*============================================================================*/
/* Encrypt                                                                     */
/*============================================================================*/

uint64_t nice_flor_s_encrypt(uint64_t data,
                             const uint8_t table[NICE_FLOR_S_TABLE_SIZE])
{
    uint8_t *p = (uint8_t *)&data;   /* little-endian byte view */
    uint8_t  k;

    /* Two forward rounds. */
    for (int y = 0; y < 2; ++y)
    {
        /* Round step A: table lookup at (p[0] & 0x1F), XOR body. */
        k = table[p[0] & 0x1FU];
        magic_xor(p, k);
        p[5] &= 0x0FU;
        p[0] ^= k & 0xE0U;

        /* Round step B: table lookup at (p[0] >> 3), XOR body (+0x25). */
        k = table[p[0] >> 3] + 0x25U;
        magic_xor(p, k);
        p[5] &= 0x0FU;
        p[0] ^= k & 0x07U;

        /* After round 0 only: swap p[0] and p[1]. */
        if (y == 0)
        {
            k = p[0]; p[0] = p[1]; p[1] = k;
        }
    }

    /* Final bitwise complement + byte permutation. */
    p[5] = ~p[5] & 0x0FU;
    k     = ~p[4];
    p[4]  = ~p[0];
    p[0]  = ~p[2];
    p[2]  =  k;       /* p[2] = ~old_p[4] */
    k     = ~p[3];
    p[3]  = ~p[1];
    p[1]  =  k;       /* p[1] = ~old_p[3] */

    return data;
}

/*============================================================================*/
/* Decrypt                                                                     */
/*============================================================================*/

uint64_t nice_flor_s_decrypt(uint64_t data,
                             const uint8_t table[NICE_FLOR_S_TABLE_SIZE])
{
    uint8_t *p = (uint8_t *)&data;   /* little-endian byte view */
    uint8_t  k;

    /* Undo final permutation (exact inverse of encrypt's last step). */
    k     = ~p[4];
    p[5]  = ~p[5] & 0x0FU;
    p[4]  = ~p[2];
    p[2]  = ~p[0];
    p[0]  =  k;       /* p[0] = ~old_p[4] */
    k     = ~p[3];
    p[3]  = ~p[1];
    p[1]  =  k;       /* p[1] = ~old_p[3] */

    /* Two reverse rounds (steps B then A, matching encrypt order because
     * the XOR operations are self-inverse when the same key is used). */
    for (int y = 0; y < 2; ++y)
    {
        /* Undo round step B. */
        k = table[p[0] >> 3] + 0x25U;
        magic_xor(p, k);
        p[5] &= 0x0FU;
        p[0] ^= k & 0x07U;

        /* Undo round step A. */
        k = table[p[0] & 0x1FU];
        magic_xor(p, k);
        p[5] &= 0x0FU;
        p[0] ^= k & 0xE0U;

        /* After round 0 only: swap p[0] and p[1]. */
        if (y == 0)
        {
            k = p[0]; p[0] = p[1]; p[1] = k;
        }
    }

    return data;
}
