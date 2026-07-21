/* See COPYING.txt for license details. */

/**
 * @file  subghz_nice_flor_s_table_builtin.c
 * @brief Nice FloR-S rainbow table embedded at build time.
 *
 * This stub version contains no table data and is the version committed
 * to the public repository.  At build time,
 * scripts/gen_nice_flor_s_table_builtin.py regenerates this file from
 * the NICE_FLOR_S_RAINBOW_TABLE environment variable (or GitHub Actions
 * secret).
 *
 * When this file contains a real table:
 *   - The scene handler and signal-fields code use it directly from
 *     firmware flash to decrypt/re-encrypt Nice FloR-S payloads.
 *   - No SD card file is consulted (same approach as KeeLoq builtin
 *     keys).
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
#include "subghz_nice_flor_s_table_builtin.h"

/* No table embedded — callers will show "table?" placeholders. */
const uint8_t * const nice_flor_s_table_builtin     = NULL;
const uint32_t        nice_flor_s_table_builtin_len  = 0;
