/* See COPYING.txt for license details. */

/*
 * subghz_block_decoder.h
 *
 * Flipper-compatible block decoder state machine.
 *
 * Mirrors Flipper's lib/subghz/blocks/decoder.h — provides a reusable struct
 * for tracking decoder state (parser step, last pulse duration, accumulated
 * data bits) and helpers for bit accumulation.
 *
 * When porting a Flipper protocol decoder:
 *   - Replace Flipper's SubGhzBlockDecoder with this identical struct.
 *   - Use subghz_protocol_blocks_add_bit() to accumulate bits.
 *   - Use parser_step to track state machine transitions.
 *   - Use te_last to remember the previous pulse duration.
 *
 * Ported from Flipper Zero firmware:
 *   https://github.com/flipperdevices/flipperzero-firmware
 *   Original file: lib/subghz/blocks/decoder.h
 *   Copyright (C) Flipper Devices Inc. — Licensed under GPLv3
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_BLOCK_DECODER_H
#define SUBGHZ_BLOCK_DECODER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Block decoder state — mirrors Flipper's SubGhzBlockDecoder exactly.
 *
 * Field names and types match Flipper so ported code compiles unchanged.
 *
 * Usage pattern (from Flipper decoders):
 *
 *     SubGhzBlockDecoder decoder = {0};
 *
 *     // In the feed() / decode loop:
 *     switch (decoder.parser_step) {
 *         case MyProtoDecoderStepReset:
 *             if (DURATION_DIFF(duration, te_short) < te_delta) {
 *                 decoder.te_last = duration;
 *                 decoder.parser_step = MyProtoDecoderStepSaveDuration;
 *             }
 *             break;
 *         case MyProtoDecoderStepSaveDuration:
 *             subghz_protocol_blocks_add_bit(&decoder, bit_value);
 *             if (decoder.decode_count_bit >= MIN_BITS) {
 *                 // success — copy decoder.decode_data to output
 *             }
 *             break;
 *     }
 */
typedef struct SubGhzBlockDecoder {
    uint32_t parser_step;       /**< State machine step (protocol-defined enum) */
    uint32_t te_last;           /**< Duration of the previous pulse (μs) */
    uint64_t decode_data;       /**< Bit accumulator (up to 64 bits) */
    uint8_t  decode_count_bit;  /**< Number of bits accumulated so far */
} SubGhzBlockDecoder;

/**
 * Reset the decoder to its initial state.
 */
static inline void subghz_block_decoder_reset(SubGhzBlockDecoder *decoder)
{
    decoder->parser_step     = 0;
    decoder->te_last         = 0;
    decoder->decode_data     = 0;
    decoder->decode_count_bit = 0;
}

/**
 * Add one data bit during decoding.
 *
 * This is the canonical Flipper helper — shifts decode_data left by 1 and
 * ORs in the new bit.  Increments decode_count_bit.
 *
 * Matches Flipper's subghz_protocol_blocks_add_bit() signature exactly.
 *
 * @param decoder  Pointer to a SubGhzBlockDecoder instance.
 * @param bit      Data bit (0 or 1).
 */
static inline void subghz_protocol_blocks_add_bit(SubGhzBlockDecoder *decoder, bool bit)
{
    decoder->decode_data = (decoder->decode_data << 1) | (bit & 1u);
    decoder->decode_count_bit++;
}

/**
 * Add one data bit to a 128-bit accumulator (two 64-bit halves).
 *
 * The lower 64 bits live in decoder->decode_data.  When a new bit is shifted
 * in, the MSB of decode_data overflows into the LSB of *head_64_bit.
 *
 * Matches Flipper's subghz_protocol_blocks_add_to_128_bit().
 *
 * @param decoder     Pointer to a SubGhzBlockDecoder instance (lower 64 bits).
 * @param bit         Data bit (0 or 1).
 * @param head_64_bit Pointer to the upper 64 bits.
 */
static inline void subghz_protocol_blocks_add_to_128_bit(
    SubGhzBlockDecoder *decoder,
    uint8_t bit,
    uint64_t *head_64_bit)
{
    *head_64_bit = (*head_64_bit << 1) | ((decoder->decode_data >> 63) & 1u);
    decoder->decode_data = (decoder->decode_data << 1) | (bit & 1u);
    decoder->decode_count_bit++;
}

/**
 * Compute a simple hash of the accumulated decode data.
 *
 * Useful for dedup — two receptions of the same signal will produce the same
 * hash.  Matches Flipper's subghz_protocol_blocks_get_hash_data().
 *
 * @param decoder  Pointer to a SubGhzBlockDecoder instance.
 * @param len      Number of bytes to hash (typically sizeof(decode_data)).
 * @return         8-bit hash.
 */
static inline uint8_t subghz_protocol_blocks_get_hash_data(
    SubGhzBlockDecoder *decoder,
    size_t len)
{
    const uint8_t *p = (const uint8_t *)&decoder->decode_data;
    uint8_t hash = 0;
    for (size_t i = 0; i < len && i < sizeof(decoder->decode_data); i++) {
        hash ^= p[i];
    }
    return hash;
}

#endif /* SUBGHZ_BLOCK_DECODER_H */
