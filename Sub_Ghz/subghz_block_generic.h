/* See COPYING.txt for license details. */

/*
 * subghz_block_generic.h
 *
 * Flipper-compatible generic protocol data container.
 *
 * Mirrors Flipper's lib/subghz/blocks/generic.h — provides a standard struct
 * for decoded protocol output (data payload, serial, button, counter, bit
 * count).  Ported Flipper decoders populate this struct instead of writing
 * directly to M1's global subghz_decenc_ctl.
 *
 * For backward compatibility, a helper copies SubGhzBlockGeneric fields into
 * the global M1 state after a successful decode.
 *
 * Ported from Flipper Zero firmware:
 *   https://github.com/flipperdevices/flipperzero-firmware
 *   Original file: lib/subghz/blocks/generic.h
 *   Copyright (C) Flipper Devices Inc. — Licensed under GPLv3
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_BLOCK_GENERIC_H
#define SUBGHZ_BLOCK_GENERIC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * Standard decoded protocol output — mirrors Flipper's SubGhzBlockGeneric.
 *
 * When porting a Flipper decoder, replace references to the Flipper
 * SubGhzBlockGeneric with this struct.  The field names and types are
 * intentionally identical so ported code compiles with zero renaming.
 */
typedef struct SubGhzBlockGeneric {
    const char *protocol_name;  /**< Protocol name (points into registry) */
    uint64_t    data;           /**< Decoded payload (up to 64 bits) */
    uint32_t    serial;         /**< Serial number / device ID */
    uint16_t    data_count_bit; /**< Number of valid bits in data */
    uint8_t     btn;            /**< Button / command field */
    uint32_t    cnt;            /**< Counter / rolling code */
} SubGhzBlockGeneric;

/**
 * Reset all fields to zero.
 */
static inline void subghz_block_generic_reset(SubGhzBlockGeneric *instance)
{
    memset(instance, 0, sizeof(*instance));
}

/**
 * Copy decoded results from SubGhzBlockGeneric into the M1 global state.
 *
 * Call this at the end of a successful decode to bridge Flipper-style output
 * into M1's subghz_decenc_ctl.  This avoids touching the global during the
 * decode loop — only commit on success.
 *
 * @param instance   Pointer to the decoded SubGhzBlockGeneric.
 * @param proto_idx  Protocol index (enum value from m1_sub_ghz_decenc.h).
 *
 * Note: This is declared here but implemented in subghz_block_generic.c
 * because it depends on subghz_decenc_ctl (m1_sub_ghz_decenc.h).
 */
void subghz_block_generic_commit_to_m1(const SubGhzBlockGeneric *instance,
                                        uint16_t proto_idx);

#endif /* SUBGHZ_BLOCK_GENERIC_H */
