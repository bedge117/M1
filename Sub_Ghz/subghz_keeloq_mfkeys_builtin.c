/* See COPYING.txt for license details. */

/**
 * @file  subghz_keeloq_mfkeys_builtin.c
 * @brief KeeLoq manufacturer keys embedded at build time.
 *
 * This stub version contains no keys and is the version committed to the
 * public repository.  At build time, scripts/gen_keeloq_mfkeys_builtin.py
 * regenerates this file from a private key vault when the
 * KEELOQ_KEY_VAULT environment variable (or GitHub Actions secret) is set.
 *
 * When this file contains real keys:
 *   - keeloq_mfkeys_load() uses them directly from firmware flash.
 *   - No SD card file is consulted (Flipper-compatible behaviour).
 *   - The keys are never visible as a file on the SD card.
 *
 * When this file contains the stub (len == 0):
 *   - keeloq_mfkeys_load() falls back to the SD card keystore.
 *   - This is the behaviour for public/CI builds without a vault.
 *
 * DO NOT commit a version of this file that contains real manufacturer keys.
 * The generator script overwrites this file in-place during private builds;
 * only the NULL stub version ever appears in the public repository.
 */

#include <stddef.h>
#include "subghz_keeloq_mfkeys.h"

/* No keys embedded — keeloq_mfkeys_load() will fall back to the SD card. */
const char * const keeloq_mfkeys_builtin_text = NULL;
const uint32_t     keeloq_mfkeys_builtin_len  = 0;
