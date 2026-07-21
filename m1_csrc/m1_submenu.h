/* See COPYING.txt for license details. */

/**
 * @file   m1_submenu.h
 * @brief  Firmware-side rendering shim for `subghz_submenu_model_t`.
 *
 * Phase 7b of the Sub-GHz Momentum-parity migration.  Pairs the pure-logic
 * scroll/selection model in `Sub_Ghz/subghz_submenu_model.{c,h}` (Phase 7a)
 * with the Hapax font-aware menu drawing helpers (`m1_menu_item_h`,
 * `m1_menu_font`, `M1_MENU_TEXT_W`, `M1_MENU_SCROLLBAR_X`) so any scene can
 * render a Flipper-style scrollable list with a single call.
 *
 * Usage from a scene's `draw` callback:
 *
 *     m1_submenu_draw(m, "My Title", labels);
 *
 * Usage from a scene's `on_event` callback:
 *
 *     return m1_submenu_event(app, event, m, targets);
 *
 * `m1_submenu_event` and `m1_submenu_draw` both automatically re-sync
 * `model->visible_count` via `M1_MENU_VIS()` so the widget adapts when the
 * user changes the text-size preference without needing explicit calls to
 * `subghz_submenu_model_set_visible_count()` in the scene.
 *
 * The shim is intentionally a thin wrapper around `m1_scene_draw_menu()` —
 * all geometry constants and the highlight/scrollbar drawing live in one
 * place, and changing the user text size (`m1_menu_style`) is picked up
 * automatically via the font-aware helpers.
 *
 * Phase 7c+ migrates individual scenes (Menu, SavedMenu, MoreRAW, Config,
 * Saved file browser, Add Manually picker, Bind Wizard protocol picker)
 * onto this widget; Phase E extends this to WiFi/BT/NFC/Settings menus.
 * Existing hand-rendered lists continue to work unchanged until migrated.
 */

#ifndef M1_SUBMENU_H_
#define M1_SUBMENU_H_

#include <stdbool.h>
#include <stdint.h>
#include "subghz_submenu_model.h"
#include "m1_scene.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Draw a scrollable submenu from a pure-logic model.
 *
 * Renders the same Flipper-style menu used elsewhere in the firmware:
 * centred title, separator line, rounded highlight on the selected row,
 * proportional scrollbar.  Item rows use the user-selected text-size font
 * (`m1_menu_font()`), so the rendering automatically follows
 * `Settings → LCD & Notifications → Text Size`.
 *
 * Automatically re-syncs `model->visible_count` via `M1_MENU_VIS()` before
 * drawing, so the scrollbar remains correct after a text-size preference
 * change.
 *
 * Safe to call with NULL @p model, @p title, or @p labels — the call
 * becomes a no-op in those cases.  When `model->item_count == 0`, the
 * title and an empty list area are drawn (no items, no scrollbar).
 *
 * @param model   Pure-logic submenu model (Phase 7a).  May be NULL.
 * @param title   Title string drawn centred at top.  May be NULL.
 * @param labels  Array of `model->item_count` label strings.  May be NULL
 *                when `model->item_count == 0`.
 */
void m1_submenu_draw(subghz_submenu_model_t *model,
                     const char *title,
                     const char *const *labels);

/**
 * @brief  Handle a scene event for a model-backed scrollable submenu.
 *
 * Phase E convenience wrapper.  Replaces the legacy `m1_scene_menu_event()`
 * call for scenes that own a `subghz_submenu_model_t` instead of a raw
 * `(sel, scroll)` pair.
 *
 * Automatically re-syncs `model->visible_count` via `M1_MENU_VIS()` before
 * processing navigation so that wrap-around math stays correct when the
 * user changes the text-size preference between events.
 *
 * Handled events:
 *   - `M1SceneEventBack`  → `m1_scene_pop(app)`
 *   - `M1SceneEventUp`    → `subghz_submenu_model_up(model)` + redraw
 *   - `M1SceneEventDown`  → `subghz_submenu_model_down(model)` + redraw
 *   - `M1SceneEventOk`    → `m1_scene_push(app, targets[model->selected])`
 *   - all others          → returns `false`
 *
 * @param app      Scene application context.  Must not be NULL.
 * @param event    Incoming scene event.
 * @param model    Submenu model.  If NULL the call returns false.
 * @param targets  Array of scene IDs, one per item.  If NULL the OK
 *                 event is silently consumed without pushing a scene.
 * @return `true` when the event was consumed, `false` otherwise.
 */
bool m1_submenu_event(M1SceneApp *app,
                      M1SceneEvent event,
                      subghz_submenu_model_t *model,
                      const uint8_t *targets);

#ifdef __cplusplus
}
#endif

#endif /* M1_SUBMENU_H_ */
