/* See COPYING.txt for license details. */

/**
 * @file   subghz_alutech_at_4n.c
 * @brief  Alutech AT-4N cipher — encrypt / decrypt implementation.
 *
 * Ported from Flipper Zero firmware
 * (lib/subghz/protocols/alutech_at_4n.c) and DarkFlippers/unleashed-
 * firmware (same path, with encrypt enabled).
 *
 * The cipher is a TEA (Tiny Encryption Algorithm) variant that uses
 * six 32-bit round constants loaded from a "rainbow table" file.  The
 * table is device-family-specific and must be loaded at runtime
 * (analogous to Nice FloR-S rainbow table / KeeLoq manufacturer keys).
 *
 * M1 Project — Hapax fork
 */

#include "subghz_alutech_at_4n.h"

/*============================================================================*/
/* Internal helpers — table read                                               */
/*============================================================================*/

/**
 * Read a big-endian uint32_t at byte offset @p idx*4 from the 32-byte
 * rainbow table.
 */
static inline uint32_t table_u32(const uint8_t *table, unsigned idx)
{
    const unsigned off = idx * 4U;
    return ((uint32_t)table[off + 0] << 24) |
           ((uint32_t)table[off + 1] << 16) |
           ((uint32_t)table[off + 2] <<  8) |
           ((uint32_t)table[off + 3]);
}

/*============================================================================*/
/* Decrypt                                                                     */
/*============================================================================*/

uint64_t alutech_at_4n_decrypt(uint64_t data,
                               const uint8_t table[ALUTECH_AT_4N_TABLE_SIZE])
{
    /* Split 64-bit block into two 32-bit halves (MSB-first byte order). */
    uint8_t *p = (uint8_t *)&data;
    uint32_t data1 = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] <<  8) | ((uint32_t)p[3]);
    uint32_t data2 = ((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) |
                     ((uint32_t)p[6] <<  8) | ((uint32_t)p[7]);

    /* Load the six decrypt round constants from the rainbow table:
     *   magic[0] = initial sum (loop variable start value)
     *   magic[1] = K0
     *   magic[2] = K1
     *   magic[3] = delta (negative — subtracted each round)
     *   magic[4] = K2
     *   magic[5] = K3                                              */
    const uint32_t m0 = table_u32(table, 0);
    const uint32_t m1 = table_u32(table, 1);
    const uint32_t m2 = table_u32(table, 2);
    const uint32_t m3 = table_u32(table, 3);
    const uint32_t m4 = table_u32(table, 4);
    const uint32_t m5 = table_u32(table, 5);

    uint32_t i = m0;
    for (unsigned rounds = 0U; rounds < 32U; ++rounds) {
        data2 = data2 - ((m1 + (data1 << 4)) ^
                         ((m2 + (data1 >> 5)) ^ (data1 + i)));
        const uint32_t data3 = data2 + i;
        i += m3;   /* subtract delta (m3 is negative delta) */
        data1 = data1 - ((m4 + (data2 << 4)) ^
                         ((m5 + (data2 >> 5)) ^ data3));
        if (i == 0U) break;
    }

    /* Write back to the byte buffer. */
    p[0] = (uint8_t)(data1 >> 24);
    p[1] = (uint8_t)(data1 >> 16);
    p[2] = (uint8_t)(data1 >>  8);
    p[3] = (uint8_t)(data1);
    p[4] = (uint8_t)(data2 >> 24);
    p[5] = (uint8_t)(data2 >> 16);
    p[6] = (uint8_t)(data2 >>  8);
    p[7] = (uint8_t)(data2);

    return data;
}

/*============================================================================*/
/* Encrypt                                                                     */
/*============================================================================*/

uint64_t alutech_at_4n_encrypt(uint64_t data,
                               const uint8_t table[ALUTECH_AT_4N_TABLE_SIZE])
{
    /* Split 64-bit block into two 32-bit halves (MSB-first byte order). */
    uint8_t *p = (uint8_t *)&data;
    uint32_t data2 = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] <<  8) | ((uint32_t)p[3]);
    uint32_t data3 = ((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) |
                     ((uint32_t)p[6] <<  8) | ((uint32_t)p[7]);

    /* Load encrypt round constants from the rainbow table:
     *   [6] = encrypt step delta (added each round)
     *   [4] = K2
     *   [5] = K3
     *   [1] = K0
     *   [2] = K1
     *   [0] = terminal sum (loop stop condition)                    */
    const uint32_t step_delta    = table_u32(table, 6);
    const uint32_t k2            = table_u32(table, 4);
    const uint32_t k3            = table_u32(table, 5);
    const uint32_t k0            = table_u32(table, 1);
    const uint32_t k1            = table_u32(table, 2);
    const uint32_t terminal_sum  = table_u32(table, 0);

    uint32_t data1 = 0;
    for (unsigned rounds = 0U; rounds < 32U; ++rounds) {
        data1 = data1 + step_delta;
        data2 = data2 + ((k2 + (data3 << 4)) ^
                         ((k3 + (data3 >> 5)) ^ (data1 + data3)));
        data3 = data3 + ((k0 + (data2 << 4)) ^
                         ((k1 + (data2 >> 5)) ^ (data1 + data2)));
        if (data1 == terminal_sum) break;
    }

    /* Write back in same byte order as decrypt. */
    p[0] = (uint8_t)(data2 >> 24);
    p[1] = (uint8_t)(data2 >> 16);
    p[2] = (uint8_t)(data2 >>  8);
    p[3] = (uint8_t)(data2);
    p[4] = (uint8_t)(data3 >> 24);
    p[5] = (uint8_t)(data3 >> 16);
    p[6] = (uint8_t)(data3 >>  8);
    p[7] = (uint8_t)(data3);

    return data;
}

/*============================================================================*/
/* CRC helpers                                                                 */
/*============================================================================*/

uint8_t alutech_at_4n_crc(uint64_t data)
{
    uint8_t *p = (uint8_t *)&data;
    uint8_t crc = 0xFFU;

    for (uint8_t y = 0; y < 8; y++)
    {
        crc ^= p[y];
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x80U)
            {
                crc <<= 1;
                crc ^= 0x31U;   /* CRC-8/MAXIM polynomial */
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t alutech_at_4n_decrypt_data_crc(uint8_t cnt_low)
{
    uint8_t crc = cnt_low;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 0x80U)
        {
            crc <<= 1;
            crc ^= 0x31U;
        }
        else
        {
            crc <<= 1;
        }
    }
    return (uint8_t)(~crc);
}
