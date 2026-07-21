/* See COPYING.txt for license details. */

/**
 * @file  subghz_alutech_at_4n_table_builtin.c
 * @brief Alutech AT-4N rainbow table embedded at build time.
 *
 * This stub version contains no table data and is the version committed
 * to the public repository.  At build time,
 * scripts/gen_alutech_at_4n_table_builtin.py regenerates this file from
 * the ALUTECH_AT_4N_RAINBOW_TABLE environment variable (or GitHub Actions
 * secret).
 *
 * When this file contains a real table:
 *   - The scene handler and signal-fields code use it directly from
 *     firmware flash to decrypt/re-encrypt Alutech AT-4N payloads.
 *   - No SD card file is consulted (same approach as KeeLoq builtin
 *     keys and Nice FloR-S rainbow table).
 *
 * When this file contains the stub (len == 0):
 *   - Counter and serial fields show "table?" placeholders.
 *   - This is the behaviour for public/CI builds without the secret.
 *
 * DO NOT commit a version of this file that contains a real rainbow
 * table.  The generator script overwrites this file in-place during
 * private builds; only the NULL stub version ever appears in the
 * public repository.
 */

#include <stddef.h>
#include "subghz_alutech_at_4n_table_builtin.h"

/* No table embedded — callers will show "table?" placeholders. */
const uint8_t * const alutech_at_4n_table_builtin     = NULL;
const uint32_t        alutech_at_4n_table_builtin_len  = 0;
