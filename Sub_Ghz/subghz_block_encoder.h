/* See COPYING.txt for license details. */

/*
 * subghz_block_encoder.h
 *
 * Flipper-compatible block encoder for Sub-GHz transmit waveforms.
 *
 * Mirrors Flipper's lib/subghz/blocks/encoder.{h,c} — provides:
 *
 *   1. SubGhzProtocolBlockEncoder struct: tracks upload buffer, repeat count,
 *      and playback position for the TX ISR.
 *
 *   2. Bit-array helpers: set/get individual bits in a uint8_t[] array
 *      (MSB-first layout matching Flipper).
 *
 *   3. Upload generator: converts a bit array + duration into a
 *      LevelDuration[] upload suitable for the SI4463 TX path.
 *
 * All functions are inline — no .c file needed.
 *
 * Depends on:
 *   - subghz_level_duration.h (LevelDuration type)
 *   - subghz_blocks_math.h    (bit_read, bit_write macros)
 *
 * Ported from Flipper Zero firmware:
 *   https://github.com/flipperdevices/flipperzero-firmware
 *   Original file: lib/subghz/blocks/encoder.{h,c}
 *   Copyright (C) Flipper Devices Inc. — Licensed under GPLv3
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_BLOCK_ENCODER_H
#define SUBGHZ_BLOCK_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "subghz_level_duration.h"
#include "subghz_blocks_math.h"

/*============================================================================*/
/* Block encoder state — mirrors Flipper's SubGhzProtocolBlockEncoder         */
/*============================================================================*/

typedef struct {
    bool            is_running;     /**< true while TX is active */
    size_t          repeat;         /**< number of TX repeats remaining */
    size_t          front;          /**< current playback index into upload[] */
    size_t          size_upload;    /**< number of entries in upload[] */
    LevelDuration  *upload;         /**< pointer to the upload waveform buffer */
} SubGhzProtocolBlockEncoder;

/*============================================================================*/
/* Alignment enum                                                             */
/*============================================================================*/

typedef enum {
    SubGhzProtocolBlockAlignBitLeft,
    SubGhzProtocolBlockAlignBitRight,
} SubGhzProtocolBlockAlignBit;

/*============================================================================*/
/* Bit-array helpers — MSB-first layout (same as Flipper)                     */
/*============================================================================*/

/**
 * Set a single bit in a byte array.
 *
 * @param bit_value     The bit value (true = 1, false = 0).
 * @param data_array    The byte array.
 * @param set_index_bit Bit index (0 = MSB of data_array[0]).
 * @param max_size_array Array size in bytes (bounds check).
 */
static inline void subghz_protocol_blocks_set_bit_array(
    bool     bit_value,
    uint8_t  data_array[],
    size_t   set_index_bit,
    size_t   max_size_array)
{
    if (set_index_bit >= max_size_array * 8) return;
    bit_write(data_array[set_index_bit >> 3],
              7 - (set_index_bit & 0x7),
              bit_value);
}

/**
 * Get a single bit from a byte array.
 *
 * @param data_array     The byte array.
 * @param read_index_bit Bit index (0 = MSB of data_array[0]).
 * @return The bit value.
 */
static inline bool subghz_protocol_blocks_get_bit_array(
    const uint8_t data_array[],
    size_t        read_index_bit)
{
    return bit_read(data_array[read_index_bit >> 3],
                    7 - (read_index_bit & 0x7));
}

/*============================================================================*/
/* Upload generator — converts bit array → LevelDuration waveform            */
/*============================================================================*/

/**
 * Generate a LevelDuration upload from a bit array.
 *
 * Consecutive same-level bits are merged into a single long pulse.
 * This is the standard NRZ-style encoding used by most static protocols.
 *
 * @param data_array           The encoded bit array.
 * @param count_bit_data_array Number of bits to process.
 * @param upload               Output LevelDuration buffer.
 * @param max_size_upload      Capacity of upload[] in entries.
 * @param duration_bit         Duration of one bit period (μs).
 * @param align_bit            Bit alignment (left = MSB-first, right = LSB-first).
 * @return Number of LevelDuration entries written.
 */
static inline size_t subghz_protocol_blocks_get_upload_from_bit_array(
    const uint8_t                data_array[],
    size_t                       count_bit_data_array,
    LevelDuration               *upload,
    size_t                       max_size_upload,
    uint32_t                     duration_bit,
    SubGhzProtocolBlockAlignBit  align_bit)
{
    size_t   bias_bit    = 0;
    size_t   size_upload = 0;
    uint32_t duration    = duration_bit;

    if (align_bit == SubGhzProtocolBlockAlignBitRight) {
        if (count_bit_data_array & 0x7) {
            bias_bit = 8 - (count_bit_data_array & 0x7);
        }
    }

    size_t index_bit = bias_bit;
    bool last_bit = subghz_protocol_blocks_get_bit_array(data_array, index_bit++);

    for (size_t i = 1 + bias_bit; i < count_bit_data_array + bias_bit; i++) {
        if (last_bit == subghz_protocol_blocks_get_bit_array(data_array, index_bit)) {
            duration += duration_bit;
        } else {
            if (size_upload >= max_size_upload) {
                /* Buffer overflow — stop generating */
                return size_upload;
            }
            upload[size_upload++] = level_duration_make(
                subghz_protocol_blocks_get_bit_array(data_array, index_bit - 1),
                duration);
            last_bit = !last_bit;
            duration = duration_bit;
        }
        index_bit++;
    }

    if (size_upload < max_size_upload) {
        upload[size_upload++] = level_duration_make(
            subghz_protocol_blocks_get_bit_array(data_array, index_bit - 1),
            duration);
    }

    return size_upload;
}

#endif /* SUBGHZ_BLOCK_ENCODER_H */
