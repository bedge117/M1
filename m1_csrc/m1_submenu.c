/* See COPYING.txt for license details. */

/**
 * @file   m1_submenu.c
 * @brief  Firmware-side rendering shim for `subghz_submenu_model_t`.
 *
 * Phase 7b: thin adapter that bridges the Phase 7a pure-logic model to
 * the existing `m1_scene_draw_menu()` renderer.  All actual u8g2 drawing
 * lives in `m1_scene.c`; this module only translates the model's fields
 * into the renderer's positional parameters and adds null-pointer guards.
 *
 * Phase E adds `m1_submenu_event()` — a drop-in replacement for the legacy
 * `m1_scene_menu_event()` helper that works with `subghz_submenu_model_t`
 * instead of a raw (sel, scroll) pair.  Both functions also auto-sync the
 * model's `visible_count` from `M1_MENU_VIS()` so callers never need to
 * call `subghz_submenu_model_set_visible_count()` manually.
 */

#include <stddef.h>
#include "m1_submenu.h"
#include "m1_scene.h"

void m1_submenu_draw(subghz_submenu_model_t *model,
                     const char *title,
                     const char *const *labels)
{
    if (model == NULL)
        return;

    /* Auto-sync visible_count so the scrollbar reflects the current text-size
     * preference even when the user changed it while a child scene was active. */
    subghz_submenu_model_set_visible_count(model,
        M1_MENU_VIS(model->item_count));

    /* labels may be NULL only when item_count == 0 — the renderer never
     * dereferences labels[] in that case because its row loop is bounded
     * by both visible_count and item_count.  Reject the inconsistent
     * "non-empty model with NULL labels" combination defensively. */
    if (model->item_count > 0 && labels == NULL)
        return;

    m1_scene_draw_menu(title != NULL ? title : "",
                       labels,
                       model->item_count,
                       model->selected,
                       model->scroll_offset,
                       model->visible_count);
}

bool m1_submenu_event(M1SceneApp *app,
                      M1SceneEvent event,
                      subghz_submenu_model_t *model,
                      const uint8_t *targets)
{
    if (model == NULL)
        return false;

    /* Re-sync visible_count in case text-size preference changed while a
     * child scene was on top. */
    subghz_submenu_model_set_visible_count(model,
        M1_MENU_VIS(model->item_count));

    switch (event)
    {
        case M1SceneEventBack:
            m1_scene_pop(app);
            return true;

        case M1SceneEventUp:
            subghz_submenu_model_up(model);
            app->need_redraw = true;
            return true;

        case M1SceneEventDown:
            subghz_submenu_model_down(model);
            app->need_redraw = true;
            return true;

        case M1SceneEventOk:
            if (targets != NULL && model->item_count > 0 &&
                targets[model->selected] < app->scene_count)
                m1_scene_push(app, targets[model->selected]);
            return true;

        default:
            break;
    }
    return false;
}
