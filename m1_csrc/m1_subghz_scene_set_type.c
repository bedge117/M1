/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_set_type.c
 * @brief  Sub-GHz Create-from-scratch SetType picker scene (Phase 8b-2).
 *
 * Scrollable picker scene over the Phase 8a/8b-1 `subghz_create_proto_*`
 * catalog.  Lets the user pick one of the 17 supported "create from
 * scratch" protocols (5 rolling-code remotes + 12 static-OOK families).
 *
 * On OK the picked @ref SubGhzCreateProtoId is stored in
 * @ref SubGhzApp::create_proto_id and the @ref SubGhzSceneSetKey
 * hex-entry scene is pushed (Phase 8b-3) to let the user build the
 * key.  Selection persists across pushes/pops via the Phase 2
 * per-scene 32-bit state slot keyed by @ref SubGhzSceneSetType so
 * the user is returned to the same protocol if they re-enter the
 * picker.
 *
 * Hardware-independent picker logic — the only firmware-side dependency
 * is the `m1_submenu_draw` widget for rendering.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "m1_display.h"
#include "m1_scene.h"
#include "m1_submenu.h"
#include "m1_subghz_scene.h"
#include "subghz_create_proto.h"
#include "subghz_submenu_model.h"

/*============================================================================*/
/* Scene state                                                                */
/*============================================================================*/

/* The labels array is rebuilt on every scene_on_enter from the catalog so
 * that any future change to the catalog (Phase 8c will add the KeeLoq
 * family) is picked up without touching this scene. */
static subghz_submenu_model_t s_model;
static const char *s_labels[SUBGHZ_CREATE_PROTO_COUNT];

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    const uint32_t count = subghz_create_proto_count();

    for (uint32_t i = 0; i < count; ++i)
    {
        const SubGhzCreateProtoSpec *spec =
            subghz_create_proto_spec((SubGhzCreateProtoId)i);
        s_labels[i] = (spec && spec->label) ? spec->label : "";
    }

    subghz_submenu_model_init(&s_model,
                              (uint8_t)count,
                              M1_MENU_VIS(count));

    /* Restore the previously-picked entry as the cursor position so the
     * user lands on the same row they last picked. */
    uint32_t saved_sel = subghz_scene_get_state(app, SubGhzSceneSetType);
    if (saved_sel < count)
    {
        /* Walk the model up/down to land on the saved index — keeps the
         * scroll-window math centralised in the submenu_model API
         * rather than poking model fields directly. */
        while (s_model.selected < saved_sel)
            subghz_submenu_model_down(&s_model);
        while (s_model.selected > saved_sel)
            subghz_submenu_model_up(&s_model);
    }

    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    const uint32_t count = subghz_create_proto_count();

    /* Re-sync visible_count in case the user changed the text-size
     * preference while a child scene was on top. */
    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(count));

    switch (event)
    {
        case SubGhzEventBack:
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            subghz_submenu_model_up(&s_model);
            app->need_redraw = true;
            return true;

        case SubGhzEventDown:
            subghz_submenu_model_down(&s_model);
            app->need_redraw = true;
            return true;

        case SubGhzEventOk:
            if (s_model.selected < count)
            {
                app->create_proto_id = s_model.selected;
                subghz_scene_set_state(app, SubGhzSceneSetType,
                                       s_model.selected);
                /* Phase 8c-3 — KeeLoq-family protocols use the
                 * Serial / Button / Counter / MfKey editor chain
                 * instead of the single opaque hex key editor.  The
                 * selection is made via the field_flags bitmask so the
                 * dispatch follows the catalog rather than hard-coded
                 * protocol IDs.  Pure SetKey is still used for the
                 * static-OOK families and the Phase 8a rolling-code
                 * remotes that expose a single opaque hex key. */
                if (subghz_create_proto_has_field(
                        (SubGhzCreateProtoId)s_model.selected,
                        SUBGHZ_CREATE_FIELD_SERIAL))
                {
                    subghz_scene_push(app, SubGhzSceneSetSerial);
                }
                else
                {
                    /* Phase 8b-3 — hand off to the hex-key editor scene.
                     * SetKey reads `create_proto_id` to look up the
                     * protocol's bit width and frequency, lets the user
                     * build a key, writes a temp .sub, and pushes the
                     * Transmitter scene to fire it. */
                    subghz_scene_push(app, SubGhzSceneSetKey);
                }
            }
            return true;

        default:
            break;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    (void)app;
}

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

static void draw(SubGhzApp *app)
{
    (void)app;
    /* Re-sync visible_count for the current text-size setting before
     * drawing — matches the Phase 7c-1 Menu scene pattern. */
    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(s_model.item_count));
    m1_submenu_draw(&s_model, "Set Type", s_labels);
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_set_type_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
