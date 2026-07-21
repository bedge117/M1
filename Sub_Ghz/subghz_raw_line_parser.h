/* See COPYING.txt for license details. */

/*
 * subghz_raw_line_parser.h
 *
 * RAW_Data line parser — extracted from sub_ghz_replay_flipper_file().
 *
 * Handles Flipper's RAW_Data format:
 *   - Signed values (negative → absolute value)
 *   - Zero values skipped
 *   - Cross-buffer leftover digit recombination when f_gets truncates
 *     a long line mid-number
 *
 * This module is hardware-independent and compiles on both ARM and host.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_RAW_LINE_PARSER_H
#define SUBGHZ_RAW_LINE_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*============================================================================*/
/* Parser State                                                                */
/*============================================================================*/

/** Persistent state across multiple calls (for split-line handling) */
typedef struct {
    char leftover[16];     /**< Partial number from previous buffer boundary */
} SubGhzRawLineState;

/*============================================================================*/
/* Functions                                                                   */
/*============================================================================*/

/**
 * Initialize parser state.
 */
void subghz_raw_line_state_init(SubGhzRawLineState *state);

/**
 * Parse a RAW_Data line (or continuation) into absolute-value samples.
 *
 * Handles Flipper's signed raw data format:
 *   - Negative values are converted to absolute value
 *   - Zero values are skipped
 *   - If line_complete is false, the last number may be incomplete
 *     (split across f_gets buffer boundaries) — it is saved in state->leftover
 *     and will be recombined with the next call
 *
 * @param text           Raw text to parse (after "RAW_Data:" prefix has been stripped)
 * @param line_complete  true if this text ends with a newline (full line),
 *                       false if the line was truncated by f_gets buffer size
 * @param state          Parser state (for leftover digit handling)
 * @param samples_out    Output array for absolute-value samples
 * @param max_samples    Size of output array
 * @return Number of samples written to samples_out
 */
uint16_t subghz_parse_raw_data_line(const char *text,
                                     bool line_complete,
                                     SubGhzRawLineState *state,
                                     uint32_t *samples_out,
                                     uint16_t max_samples);

#endif /* SUBGHZ_RAW_LINE_PARSER_H */
