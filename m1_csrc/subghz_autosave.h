/* See COPYING.txt for license details. */

/*
 * subghz_autosave.h
 *
 * Pure-logic Sub-GHz autosave helpers — filename generation and duplicate
 * detection for the autosave feature.
 *
 * Extracted for host-side testability.
 * Hardware-independent: no FatFS, no RTOS, no display.
 *
 * M1 Project
 */

#ifndef SUBGHZ_AUTOSAVE_H_
#define SUBGHZ_AUTOSAVE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SUBGHZ_AUTOSAVE_DIR_NAME  "autosave"
#define SUBGHZ_AUTOSAVE_DIR_PATH  "/SUBGHZ/" SUBGHZ_AUTOSAVE_DIR_NAME

/**
 * @brief  Generate an autosave filename.
 *
 * Produces a path like:  /SUBGHZ/autosave/<protocol>_<key_hex>_<tick>.<ext>
 *
 * @param out         output buffer for the full path
 * @param out_size    size of the output buffer
 * @param protocol    protocol name string (e.g. "Princeton")
 * @param key         decoded key value
 * @param tick        monotonic timestamp for uniqueness
 * @param use_sgh     true = .sgh extension, false = .sub extension
 * @return            number of characters written (excl NUL), or 0 on error
 */
int subghz_autosave_make_path(char *out, size_t out_size,
                              const char *protocol, uint64_t key,
                              uint32_t tick, bool use_sgh);

/**
 * @brief  Check if a protocol+key combination was already saved this session.
 *
 * Uses a small ring of recently saved {protocol_hash, key} pairs.
 * Returns true if the signal is a duplicate of one already saved.
 * Does NOT record the entry — call subghz_autosave_record() after a
 * successful save to register it.
 *
 * @param protocol  protocol name string (NULL returns true)
 * @param key       decoded key value
 * @return          true if duplicate (already saved), false if new
 */
bool subghz_autosave_is_duplicate(const char *protocol, uint64_t key);

/**
 * @brief  Record a protocol+key pair as "saved" for duplicate detection.
 *
 * Call this AFTER a successful save so that future calls to
 * subghz_autosave_is_duplicate() with the same pair return true.
 *
 * @param protocol  protocol name string (NULL is a no-op)
 * @param key       decoded key value
 */
void subghz_autosave_record(const char *protocol, uint64_t key);

/**
 * @brief  Reset the duplicate detection state (e.g. on scene entry).
 */
void subghz_autosave_dup_reset(void);

#endif /* SUBGHZ_AUTOSAVE_H_ */
