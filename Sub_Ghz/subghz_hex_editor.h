/* See COPYING.txt for license details. */

/**
 * @file   subghz_hex_editor.h
 * @brief  Pure-logic hex-digit editor — backing model for the Sub-GHz
 *         Create-from-scratch SetKey scene and the upcoming Phase 8c
 *         SetSerial / SetButton / SetCounter editor scenes.
 *
 * Models a fixed-width hex-digit string with a movable cursor.  Each
 * digit is stored as its own nibble (0..15) so UP/DOWN cycling on an
 * individual digit does not require unpacking a 64-bit value on every
 * keypress.  The whole editor state lives in a single struct that
 * scenes can keep as a file-scope static (the digit array is sized for
 * the worst case: a 64-bit Alutech AT-4N key = 16 hex digits).
 *
 * The module has zero hardware dependencies — no SI4463, no HAL, no
 * FreeRTOS, no FAT FS — and is fully testable on the host.  Cursor and
 * digit mutation use wrap-around semantics matching the legacy
 * `sub_ghz_add_manually()` UX: UP from F → 0, DOWN from 0 → F, LEFT at
 * position 0 stays at 0, RIGHT at the last position stays at the last
 * position (no wrap on cursor — matches Add Manually).
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_HEX_EDITOR_H
#define SUBGHZ_HEX_EDITOR_H

#include <stdbool.h>
#include <stdint.h>

/*============================================================================*/
/* Constants                                                                  */
/*============================================================================*/

/** Maximum number of hex digits the editor can hold.  64 bits = 16 nibbles. */
#define SUBGHZ_HEX_EDITOR_MAX_DIGITS  16U

/*============================================================================*/
/* Editor state                                                               */
/*============================================================================*/

/**
 * Editor state — all fields are byte-sized.  The digits array is in
 * big-endian display order: index 0 is the most-significant nibble that
 * appears leftmost on screen.
 */
typedef struct {
    uint8_t digits[SUBGHZ_HEX_EDITOR_MAX_DIGITS]; /**< Each entry is 0..15 */
    uint8_t cursor;        /**< Cursor position, 0 <= cursor < digit_count */
    uint8_t digit_count;   /**< Number of significant hex digits in use */
    uint8_t bit_count;     /**< Significant bit width (clamped to ≤ 64) */
} subghz_hex_editor_t;

/*============================================================================*/
/* API                                                                        */
/*============================================================================*/

/**
 * Initialise the editor for a given bit width.
 *
 * The digit count is computed as `(bit_count + 3) / 4`, clamped to
 * [1, @ref SUBGHZ_HEX_EDITOR_MAX_DIGITS].  All digits are cleared to 0
 * and the cursor is placed at position 0 (the most-significant nibble).
 *
 * @param ed         Editor state (must be non-NULL).
 * @param bit_count  Number of significant bits the protocol uses.
 *                   Clamped to [1, 64].  bit_count == 0 is treated as 1.
 */
void subghz_hex_editor_init(subghz_hex_editor_t *ed, uint8_t bit_count);

/**
 * Load an existing 64-bit value into the editor's digit array.
 *
 * Bits above the editor's significant bit width are silently masked off
 * (matches @ref subghz_create_proto_key_in_range overflow handling).
 * The cursor is preserved.
 *
 * @param ed     Editor state (must be non-NULL and previously init'd).
 * @param value  Value to load.  Only the low `bit_count` bits are used.
 */
void subghz_hex_editor_set_value(subghz_hex_editor_t *ed, uint64_t value);

/**
 * Assemble the digit array into a single 64-bit value.
 *
 * @param ed  Editor state (must be non-NULL).
 * @return    Big-endian-assembled value, masked to `bit_count` bits.
 *            Returns 0 if @p ed is NULL.
 */
uint64_t subghz_hex_editor_value(const subghz_hex_editor_t *ed);

/**
 * Cycle the digit under the cursor up by one (wrap F → 0).
 * No-op if @p ed is NULL or the cursor is past digit_count.
 */
void subghz_hex_editor_up(subghz_hex_editor_t *ed);

/**
 * Cycle the digit under the cursor down by one (wrap 0 → F).
 * No-op if @p ed is NULL or the cursor is past digit_count.
 */
void subghz_hex_editor_down(subghz_hex_editor_t *ed);

/**
 * Move the cursor one position to the left.  Saturates at 0 — no wrap.
 * No-op if @p ed is NULL.
 */
void subghz_hex_editor_left(subghz_hex_editor_t *ed);

/**
 * Move the cursor one position to the right.  Saturates at
 * `digit_count - 1` — no wrap.
 * No-op if @p ed is NULL.
 */
void subghz_hex_editor_right(subghz_hex_editor_t *ed);

#endif /* SUBGHZ_HEX_EDITOR_H */
