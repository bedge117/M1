/* See COPYING.txt for license details. */

/**
 * @file   subghz_submenu_model.h
 * @brief  Reusable scrollable-list model — pure logic, host-testable.
 *
 * Models the scroll / selection / wrap-around behaviour shared by every
 * scrollable list scene in the Sub-GHz module (Menu, SavedMenu, MoreRAW,
 * Config, …).  Hardware-independent: no u8g2, no FreeRTOS, no SubGhzApp.
 *
 * The rendering layer is a thin firmware-side shim (Phase 7b) that reads
 * `selected` / `scroll_offset` / `visible_count` from this model and draws
 * the highlight box + scrollbar + item text using the Hapax font-aware
 * helpers.  The semantic contract intentionally mirrors Flipper / Momentum's
 * `Submenu` widget so menu code is portable between the two.
 *
 * Key invariants enforced by every mutator:
 *   - `selected   < item_count` (when `item_count > 0`)
 *   - `scroll_offset <= selected`
 *   - `selected   < scroll_offset + visible_count`
 *   - `scroll_offset + visible_count <= max(item_count, visible_count)`
 *
 * Wrap-around: pressing Up at index 0 jumps to `item_count - 1`; pressing
 * Down at the last index jumps to 0.  Scroll window snaps to the new
 * selection in both directions.
 *
 * Empty list (`item_count == 0`) is supported: navigation and selection
 * are no-ops, `selected = 0`, `scroll_offset = 0`.
 */

#ifndef SUBGHZ_SUBMENU_MODEL_H_
#define SUBGHZ_SUBMENU_MODEL_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Submenu model — fits in 4 bytes; cheap to pack into a per-scene
 *         state slot (`subghz_scene_state_t`).
 *
 * `visible_count == 0` is rejected by `init()` and clamped to 1 by every
 * mutator that touches it — division-by-zero in scroll-window math is
 * therefore impossible.
 */
typedef struct {
    uint8_t item_count;     /**< Total number of items in the list.        */
    uint8_t selected;       /**< Currently selected index (0 ≤ < count).   */
    uint8_t scroll_offset;  /**< Index of the first visible row.           */
    uint8_t visible_count;  /**< How many rows fit on screen (≥ 1).        */
} subghz_submenu_model_t;

/**
 * @brief  Initialise a model.  Safe to call with NULL (no-op).
 *
 * @param  m              Model to initialise.
 * @param  item_count     Number of items in the list.
 * @param  visible_count  Visible row count (clamped to ≥ 1).
 *
 * After init: `selected = 0`, `scroll_offset = 0`.
 */
void subghz_submenu_model_init(subghz_submenu_model_t *m,
                               uint8_t item_count,
                               uint8_t visible_count);

/**
 * @brief  Replace the total item count.
 *
 * If the previous `selected` would be out-of-range under the new count,
 * the selection clamps down (to `item_count - 1`, or 0 when empty).  The
 * scroll window is re-anchored so the new selection remains visible.
 */
void subghz_submenu_model_set_item_count(subghz_submenu_model_t *m,
                                         uint8_t item_count);

/**
 * @brief  Replace the visible row count (e.g. on user text-size change).
 *
 * The scroll window is recomputed so `selected` remains visible.  A
 * `visible_count == 0` request is clamped to 1.
 */
void subghz_submenu_model_set_visible_count(subghz_submenu_model_t *m,
                                            uint8_t visible_count);

/**
 * @brief  Set the selected index to an absolute value.
 *
 * Out-of-range indices clamp to `item_count - 1` (or 0 if empty).  The
 * scroll window snaps so the new selection is visible.
 */
void subghz_submenu_model_set_selected(subghz_submenu_model_t *m,
                                       uint8_t index);

/**
 * @brief  Move the selection up one row.  Wraps from 0 → `item_count - 1`.
 *
 * No-op on an empty list.
 */
void subghz_submenu_model_up(subghz_submenu_model_t *m);

/**
 * @brief  Move the selection down one row.  Wraps from last → 0.
 *
 * No-op on an empty list.
 */
void subghz_submenu_model_down(subghz_submenu_model_t *m);

/**
 * @brief  Query: would index @p idx be drawn under the current scroll window?
 */
bool subghz_submenu_model_is_visible(const subghz_submenu_model_t *m,
                                     uint8_t idx);

/**
 * @brief  Query: row position of the selected item within the visible
 *         window (0 = top row).  Returns 0 on an empty list.
 */
uint8_t subghz_submenu_model_visible_row(const subghz_submenu_model_t *m);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_SUBMENU_MODEL_H_ */
