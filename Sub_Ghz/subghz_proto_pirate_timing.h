/* See COPYING.txt for license details. */

/**
 * @file   subghz_proto_pirate_timing.h
 * @brief  Proto Pirate — pure-logic timing analysis for automotive keyfob signals.
 *
 * Hardware-independent module: takes an array of raw pulse durations (µs)
 * and a protocol timing reference, returns statistical analysis suitable
 * for the Timing Tuner scene.
 *
 * All functions are pure (no global side-effects) and host-testable.
 */

#ifndef SUBGHZ_PROTO_PIRATE_TIMING_H
#define SUBGHZ_PROTO_PIRATE_TIMING_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* Protocol timing reference table                                            */
/*============================================================================*/

/** Per-protocol timing reference (matches ProtoPirate's ProtoPirateProtocolTiming) */
typedef struct {
    const char *name;       /**< Protocol display name */
    uint16_t    te_short;   /**< Expected short pulse duration (µs) */
    uint16_t    te_long;    /**< Expected long pulse duration (µs) */
    uint16_t    te_delta;   /**< Acceptable deviation (µs) */
} pptime_proto_ref_t;

/** Automotive protocol reference table (defined in .c) */
extern const pptime_proto_ref_t pptime_proto_table[];

/** Number of entries in pptime_proto_table */
extern const uint8_t pptime_proto_table_count;

/*============================================================================*/
/* Statistics                                                                  */
/*============================================================================*/

/** Pulse timing statistics produced by pptime_analyze() */
typedef struct {
    /* Short-pulse class */
    int32_t avg_short;    /**< Average short pulse duration (µs), 0 if none */
    int32_t min_short;    /**< Minimum short pulse duration (µs), 0 if none */
    int32_t max_short;    /**< Maximum short pulse duration (µs), 0 if none */
    size_t  n_short;      /**< Number of pulses classified as short */

    /* Long-pulse class */
    int32_t avg_long;     /**< Average long pulse duration (µs), 0 if none */
    int32_t min_long;     /**< Minimum long pulse duration (µs), 0 if none */
    int32_t max_long;     /**< Maximum long pulse duration (µs), 0 if none */
    size_t  n_long;       /**< Number of pulses classified as long */

    size_t  n_total;      /**< Total pulses examined (after range filtering) */
} pptime_stats_t;

/** Match verdict returned by pptime_match() */
typedef enum {
    PPTIME_MATCH_NO_DATA = 0, /**< Insufficient samples to conclude */
    PPTIME_MATCH_OK,          /**< Both short and long within tolerance */
    PPTIME_MATCH_SHORT_HI,    /**< Short pulses too long */
    PPTIME_MATCH_SHORT_LO,    /**< Short pulses too short */
    PPTIME_MATCH_LONG_HI,     /**< Long pulses too long */
    PPTIME_MATCH_LONG_LO,     /**< Long pulses too short */
    PPTIME_MATCH_MISMATCH,    /**< Both classes out of tolerance */
} pptime_match_result_t;

/*============================================================================*/
/* API                                                                        */
/*============================================================================*/

/**
 * @brief Analyse an array of raw pulse durations into short/long statistics.
 *
 * Pulses are classified by a threshold midway between te_short and te_long.
 * Values outside [min_valid, max_valid] (derived from 2×te_delta) are ignored
 * as noise.  The stats struct is zeroed before writing.
 *
 * @param durations  Array of unsigned pulse durations in µs (from M1's pulse_times[] buffer).
 * @param count      Number of elements in durations[].
 * @param ref        Protocol reference used to set threshold and valid range.
 *                   May be NULL; in that case reasonable defaults are used.
 * @param out        Output statistics (written on return).
 */
void pptime_analyze(const uint16_t *durations, size_t count,
                    const pptime_proto_ref_t *ref,
                    pptime_stats_t *out);

/**
 * @brief Compare pptime_stats_t against a protocol reference and return verdict.
 *
 * @param stats  Statistics produced by pptime_analyze().
 * @param ref    Protocol reference to compare against.
 * @return       PPTIME_MATCH_* verdict.
 */
pptime_match_result_t pptime_match(const pptime_stats_t *stats,
                                   const pptime_proto_ref_t *ref);

/**
 * @brief Short human-readable string for a match verdict.
 *
 * @param result  One of the PPTIME_MATCH_* values.
 * @return        Static string (never NULL).
 */
const char *pptime_match_str(pptime_match_result_t result);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_PROTO_PIRATE_TIMING_H */
