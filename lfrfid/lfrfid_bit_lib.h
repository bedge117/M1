/* See COPYING.txt for license details. */

/*
 * Bit manipulation library for LF RFID protocol decoders.
 * Ported from Flipper Zero firmware (lib/bit_lib).
 *
 * Original: Copyright (C) Flipper Devices Inc., GPLv3.
 * Modifications: Copyright (C) 2026 Monstatek.
 */

#ifndef LFRFID_BIT_LIB_H_
#define LFRFID_BIT_LIB_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*============================================================================*/
/* Push a single bit into MSB shift register                                  */
/*============================================================================*/
static inline void bl_push_bit(uint8_t *data, size_t data_size, bool bit)
{
    for (size_t i = 0; i < data_size - 1; i++)
        data[i] = (uint8_t)((data[i] << 1) | (data[i + 1] >> 7));
    data[data_size - 1] = (uint8_t)((data[data_size - 1] << 1) | (bit ? 1 : 0));
}

/*============================================================================*/
/* Get a single bit                                                           */
/*============================================================================*/
static inline bool bl_get_bit(const uint8_t *data, size_t pos)
{
    return (data[pos / 8] >> (7 - (pos % 8))) & 1;
}

/*============================================================================*/
/* Set a single bit                                                           */
/*============================================================================*/
static inline void bl_set_bit(uint8_t *data, size_t pos, bool val)
{
    if (val)
        data[pos / 8] |= (uint8_t)(1 << (7 - (pos % 8)));
    else
        data[pos / 8] &= (uint8_t)~(1 << (7 - (pos % 8)));
}

/*============================================================================*/
/* Get up to 8 bits from a bit stream                                         */
/*============================================================================*/
static inline uint8_t bl_get_bits(const uint8_t *data, size_t pos, uint8_t len)
{
    uint8_t result = 0;
    for (uint8_t i = 0; i < len; i++)
        result = (uint8_t)((result << 1) | bl_get_bit(data, pos + i));
    return result;
}

/*============================================================================*/
/* Get up to 16 bits from a bit stream                                        */
/*============================================================================*/
static inline uint16_t bl_get_bits_16(const uint8_t *data, size_t pos, uint8_t len)
{
    uint16_t result = 0;
    for (uint8_t i = 0; i < len; i++)
        result = (uint16_t)((result << 1) | bl_get_bit(data, pos + i));
    return result;
}

/*============================================================================*/
/* Get up to 32 bits from a bit stream                                        */
/*============================================================================*/
static inline uint32_t bl_get_bits_32(const uint8_t *data, size_t pos, uint8_t len)
{
    uint32_t result = 0;
    for (uint8_t i = 0; i < len; i++)
        result = (result << 1) | bl_get_bit(data, pos + i);
    return result;
}

/*============================================================================*/
/* Get up to 64 bits from a bit stream                                        */
/*============================================================================*/
static inline uint64_t bl_get_bits_64(const uint8_t *data, size_t pos, uint8_t len)
{
    uint64_t result = 0;
    for (uint8_t i = 0; i < len; i++)
        result = (result << 1) | bl_get_bit(data, pos + i);
    return result;
}

/*============================================================================*/
/* Set multiple bits at a position                                            */
/*============================================================================*/
static inline void bl_set_bits(uint8_t *data, size_t pos, uint8_t val, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
        bl_set_bit(data, pos + i, (val >> (len - 1 - i)) & 1);
}

/*============================================================================*/
/* Copy bits between buffers                                                  */
/*============================================================================*/
static inline void bl_copy_bits(uint8_t *dst, size_t dst_pos,
                                size_t len,
                                const uint8_t *src, size_t src_pos)
{
    for (size_t i = 0; i < len; i++)
        bl_set_bit(dst, dst_pos + i, bl_get_bit(src, src_pos + i));
}

/*============================================================================*/
/* Remove every nth bit (for Gallagher etc.)                                  */
/*============================================================================*/
static inline void bl_remove_bit_every_nth(uint8_t *data, size_t start,
                                           size_t len, size_t n)
{
    size_t dst = start;
    for (size_t i = start; i < start + len; i++)
    {
        if (((i - start + 1) % n) != 0)
        {
            bl_set_bit(data, dst, bl_get_bit(data, i));
            dst++;
        }
    }
    /* Zero remaining bits */
    for (size_t i = dst; i < start + len; i++)
        bl_set_bit(data, i, false);
}

/*============================================================================*/
/* Circular index increment                                                   */
/*============================================================================*/
#define bl_increment_index(index, max_val) \
    do { (index)++; if ((index) >= (max_val)) (index) = 0; } while(0)

/*============================================================================*/
/* CRC-8 (Gallagher, Paradox, etc.)                                           */
/*============================================================================*/
static inline uint8_t bl_crc8(
    const uint8_t *data, size_t len,
    uint8_t poly, uint8_t init,
    bool ref_in, bool ref_out, uint8_t xor_out)
{
    uint8_t crc = init;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];
        if (ref_in)
        {
            uint8_t rev = 0;
            for (int b = 0; b < 8; b++)
                rev |= (uint8_t)(((byte >> b) & 1) << (7 - b));
            byte = rev;
        }
        crc ^= byte;
        for (int b = 0; b < 8; b++)
        {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ poly);
            else
                crc <<= 1;
        }
    }
    if (ref_out)
    {
        uint8_t rev = 0;
        for (int b = 0; b < 8; b++)
            rev |= (uint8_t)(((crc >> b) & 1) << (7 - b));
        crc = rev;
    }
    return crc ^ xor_out;
}

/*============================================================================*/
/* Test parity of a bit range                                                 */
/*============================================================================*/
static inline bool bl_test_parity_32(uint32_t val, uint8_t len)
{
    uint32_t parity = 0;
    for (uint8_t i = 0; i < len; i++)
        parity ^= (val >> i) & 1;
    return parity & 1;
}

#endif /* LFRFID_BIT_LIB_H_ */
