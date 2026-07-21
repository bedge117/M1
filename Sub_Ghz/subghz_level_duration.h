/* See COPYING.txt for license details. */

/*
 * subghz_level_duration.h
 *
 * Flipper-compatible level-duration pair for encoder uploads.
 *
 * Mirrors Flipper's lib/toolbox/level_duration.h — a compact struct
 * representing a single pulse edge: HIGH or LOW for N microseconds.
 * Used by SubGhzProtocolBlockEncoder and Manchester encoder to build
 * transmit waveforms.
 *
 * Ported from Flipper Zero firmware:
 *   https://github.com/flipperdevices/flipperzero-firmware
 *   Original file: lib/toolbox/level_duration.h
 *   Copyright (C) Flipper Devices Inc. — Licensed under GPLv3
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_LEVEL_DURATION_H
#define SUBGHZ_LEVEL_DURATION_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* LevelDuration — one (level, duration_μs) pair                              */
/*                                                                            */
/* Uses the "BIG" layout from Flipper: 30-bit duration + 2-bit level tag.     */
/* Max representable duration: 2^30 − 1 ≈ 1.07 s (more than enough).         */
/*============================================================================*/

#define LEVEL_DURATION_RESET      0U
#define LEVEL_DURATION_LEVEL_LOW  1U
#define LEVEL_DURATION_LEVEL_HIGH 2U
#define LEVEL_DURATION_WAIT       3U
#define LEVEL_DURATION_RESERVED   0x800000U

typedef struct {
    uint32_t duration : 30;
    uint8_t  level    : 2;
} LevelDuration;

/** Create a LevelDuration from a logic level and microsecond duration. */
static inline LevelDuration level_duration_make(bool level, uint32_t duration)
{
    LevelDuration ld;
    ld.level    = level ? LEVEL_DURATION_LEVEL_HIGH : LEVEL_DURATION_LEVEL_LOW;
    ld.duration = duration;
    return ld;
}

/** Create a reset sentinel (level == 0, duration == 0). */
static inline LevelDuration level_duration_reset(void)
{
    LevelDuration ld;
    ld.level    = LEVEL_DURATION_RESET;
    ld.duration = 0;
    return ld;
}

/** Create a wait sentinel. */
static inline LevelDuration level_duration_wait(void)
{
    LevelDuration ld;
    ld.level    = LEVEL_DURATION_WAIT;
    ld.duration = 0;
    return ld;
}

/** Test for reset sentinel. */
static inline bool level_duration_is_reset(LevelDuration ld)
{
    return ld.level == LEVEL_DURATION_RESET;
}

/** Test for wait sentinel. */
static inline bool level_duration_is_wait(LevelDuration ld)
{
    return ld.level == LEVEL_DURATION_WAIT;
}

/** Get the logic level (true = HIGH, false = LOW). */
static inline bool level_duration_get_level(LevelDuration ld)
{
    return ld.level == LEVEL_DURATION_LEVEL_HIGH;
}

/** Get the duration in microseconds. */
static inline uint32_t level_duration_get_duration(LevelDuration ld)
{
    return ld.duration;
}

#endif /* SUBGHZ_LEVEL_DURATION_H */
