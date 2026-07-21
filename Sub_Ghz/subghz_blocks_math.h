/* See COPYING.txt for license details. */

/*
 * subghz_blocks_math.h
 *
 * Flipper-compatible math, CRC, parity, and bit-manipulation helpers.
 *
 * Mirrors Flipper's lib/subghz/blocks/math.h — provides identically-named
 * wrappers around M1's existing bit_util.h functions so ported Flipper decode
 * logic compiles with zero renaming.
 *
 * Ported from Flipper Zero firmware:
 *   https://github.com/flipperdevices/flipperzero-firmware
 *   Original file: lib/subghz/blocks/math.h
 *   Copyright (C) Flipper Devices Inc. — Licensed under GPLv3
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_BLOCKS_MATH_H
#define SUBGHZ_BLOCKS_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "bit_util.h"

/*============================================================================*/
/* Bit manipulation macros — match Flipper's lib/subghz/blocks/math.h         */
/*============================================================================*/

#ifndef bit_read
#define bit_read(value, bit)  (((value) >> (bit)) & 0x01)
#endif

#ifndef bit_set
#define bit_set(value, bit)   ((value) |= ((typeof(value))1 << (bit)))
#endif

#ifndef bit_clear
#define bit_clear(value, bit) ((value) &= ~((typeof(value))1 << (bit)))
#endif

#ifndef bit_write
#define bit_write(value, bit, bitvalue) \
    ((bitvalue) ? bit_set(value, bit) : bit_clear(value, bit))
#endif

#ifndef DURATION_DIFF
#define DURATION_DIFF(x, y)  (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))
#endif

/*============================================================================*/
/* Key reversal — Flipper-named wrapper                                       */
/*============================================================================*/

/**
 * Flip the data bitwise (LSB↔MSB).
 *
 * Matches Flipper's subghz_protocol_blocks_reverse_key().
 */
static inline uint64_t subghz_protocol_blocks_reverse_key(uint64_t key,
                                                           uint8_t bit_count)
{
    uint64_t result = 0;
    for (uint8_t i = 0; i < bit_count; i++) {
        result = (result << 1) | (key & 1u);
        key >>= 1;
    }
    return result;
}

/*============================================================================*/
/* Parity helpers — Flipper-named wrappers                                    */
/*============================================================================*/

/**
 * Get parity of a 64-bit key (count of set bits mod 2).
 *
 * Matches Flipper's subghz_protocol_blocks_get_parity().
 */
static inline uint8_t subghz_protocol_blocks_get_parity(uint64_t key,
                                                         uint8_t bit_count)
{
    uint8_t parity = 0;
    for (uint8_t i = 0; i < bit_count; i++) {
        parity ^= (key >> i) & 1u;
    }
    return parity;
}

/**
 * Compute bit parity of a single byte.
 * Wrapper around bit_util.h parity8().
 */
static inline uint8_t subghz_protocol_blocks_parity8(uint8_t byte)
{
    return (uint8_t)parity8(byte);
}

/**
 * Compute bit parity of a number of bytes.
 * Wrapper around bit_util.h parity_bytes().
 */
static inline uint8_t subghz_protocol_blocks_parity_bytes(const uint8_t message[],
                                                           size_t size)
{
    return (uint8_t)parity_bytes(message, (unsigned)size);
}

/**
 * XOR (byte-wide parity) of a number of bytes.
 * Wrapper around bit_util.h xor_bytes().
 */
static inline uint8_t subghz_protocol_blocks_xor_bytes(const uint8_t message[],
                                                        size_t size)
{
    return xor_bytes(message, (unsigned)size);
}

/**
 * Addition of a number of bytes.
 * Wrapper around bit_util.h add_bytes().
 */
static inline uint8_t subghz_protocol_blocks_add_bytes(const uint8_t message[],
                                                        size_t size)
{
    return (uint8_t)add_bytes(message, (unsigned)size);
}

/*============================================================================*/
/* CRC helpers — Flipper-named wrappers around bit_util.h                     */
/*============================================================================*/

static inline uint8_t subghz_protocol_blocks_crc4(const uint8_t message[],
                                                    size_t size,
                                                    uint8_t polynomial,
                                                    uint8_t init)
{
    return crc4(message, (unsigned)size, polynomial, init);
}

static inline uint8_t subghz_protocol_blocks_crc7(const uint8_t message[],
                                                    size_t size,
                                                    uint8_t polynomial,
                                                    uint8_t init)
{
    return crc7(message, (unsigned)size, polynomial, init);
}

static inline uint8_t subghz_protocol_blocks_crc8(const uint8_t message[],
                                                    size_t size,
                                                    uint8_t polynomial,
                                                    uint8_t init)
{
    return crc8(message, (unsigned)size, polynomial, init);
}

static inline uint8_t subghz_protocol_blocks_crc8le(const uint8_t message[],
                                                      size_t size,
                                                      uint8_t polynomial,
                                                      uint8_t init)
{
    return crc8le(message, (unsigned)size, polynomial, init);
}

static inline uint16_t subghz_protocol_blocks_crc16lsb(const uint8_t message[],
                                                         size_t size,
                                                         uint16_t polynomial,
                                                         uint16_t init)
{
    return crc16lsb(message, (unsigned)size, polynomial, init);
}

static inline uint16_t subghz_protocol_blocks_crc16(const uint8_t message[],
                                                      size_t size,
                                                      uint16_t polynomial,
                                                      uint16_t init)
{
    return crc16(message, (unsigned)size, polynomial, init);
}

/*============================================================================*/
/* LFSR digest helpers — Flipper-named wrappers                               */
/*============================================================================*/

static inline uint8_t subghz_protocol_blocks_lfsr_digest8(const uint8_t message[],
                                                           size_t size,
                                                           uint8_t gen,
                                                           uint8_t key)
{
    return lfsr_digest8(message, (unsigned)size, gen, key);
}

static inline uint8_t subghz_protocol_blocks_lfsr_digest8_reflect(
    const uint8_t message[],
    size_t size,
    uint8_t gen,
    uint8_t key)
{
    return lfsr_digest8_reflect(message, (int)size, gen, key);
}

static inline uint16_t subghz_protocol_blocks_lfsr_digest16(const uint8_t message[],
                                                              size_t size,
                                                              uint16_t gen,
                                                              uint16_t key)
{
    return lfsr_digest16(message, (unsigned)size, gen, key);
}

#endif /* SUBGHZ_BLOCKS_MATH_H */
