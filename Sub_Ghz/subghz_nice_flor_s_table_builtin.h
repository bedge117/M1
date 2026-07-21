/* See COPYING.txt for license details. */

/**
 * @file  subghz_nice_flor_s_table_builtin.h
 * @brief Nice FloR-S rainbow table — build-time injection declarations.
 *
 * The 32-byte rainbow/permutation table can be baked into the firmware
 * binary at build time using scripts/gen_nice_flor_s_table_builtin.py
 * and the NICE_FLOR_S_RAINBOW_TABLE GitHub Actions secret (or env var).
 *
 * When the secret is absent, the committed stub
 * (subghz_nice_flor_s_table_builtin.c) provides a NULL pointer and
 * zero length — callers fall back to an SD-card-based table or show
 * "table?" placeholders.
 *
 * M1 Project — Hapax fork
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Pointer to the builtin 32-byte rainbow table, or NULL when the stub
 * is in effect (no table embedded at build time).
 */
extern const uint8_t * const nice_flor_s_table_builtin;

/**
 * Length of the builtin rainbow table in bytes (32 when populated, 0 for stub).
 */
extern const uint32_t nice_flor_s_table_builtin_len;

/**
 * @brief  Convenience: returns true when a builtin rainbow table is available.
 *
 * Equivalent to `nice_flor_s_table_builtin != NULL`.
 */
static inline bool nice_flor_s_table_builtin_available(void)
{
    return nice_flor_s_table_builtin != NULL;
}

#ifdef __cplusplus
}
#endif
