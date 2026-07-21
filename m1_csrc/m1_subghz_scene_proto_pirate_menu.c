/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_proto_pirate_menu.c
 * @brief  Sub-GHz Proto Pirate sub-menu scene.
 *
 * Entry point for the ProtoPirate rolling-code analysis toolkit as
 * integrated into the M1 Sub-GHz scene stack.
 *
 * Menu items:
 *   1. Receiver    — Live automotive-protocol capture; pushes SubGhzSceneRead
 *                    (full async protocol-aware receive, same radio stack).
 *   2. Sub Decode  — Offline decode of a saved .sub file; pushes
 *                    SubGhzSceneSaved so the user can browse and select a
 *                    file, then the existing Decode-Raw flow handles analysis.
 *   3. Timing Tuner — Async timing analysis tool; pushes
 *                    SubGhzSceneProtoPirateTuner.
 *
 * All scene transitions are non-blocking; the scene itself only handles
 * navigation events (UP / DOWN / OK / BACK).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_subghz_scene.h"
#include "m1_submenu.h"
#include "subghz_submenu_model.h"

/*============================================================================*/
/* Menu items                                                                 */
/*============================================================================*/

#define PP_MENU_ITEM_COUNT  3

static const char *pp_menu_labels[PP_MENU_ITEM_COUNT] = {
    "Receiver",
    "Sub Decode",
    "Timing Tuner",
};

static const SubGhzSceneId pp_menu_targets[PP_MENU_ITEM_COUNT] = {
    SubGhzSceneRead,                   /* Receiver: reuse existing async read scene */
    SubGhzSceneSaved,                  /* Sub Decode: browse saved files → Decode Raw flow */
    SubGhzSceneProtoPirateTuner,       /* Timing Tuner: new async scene */
};

/*============================================================================*/
/* Selection state                                                            */
/*============================================================================*/

static subghz_submenu_model_t s_model;

static inline uint8_t pp_menu_get_saved_sel(const SubGhzApp *app)
{
    uint32_t s = subghz_scene_get_state(app, SubGhzSceneProtoPirateMenu);
    uint8_t sel = (uint8_t)(s & 0xFFu);
    return (sel < PP_MENU_ITEM_COUNT) ? sel : 0;
}

static inline void pp_menu_save_sel(SubGhzApp *app, uint8_t sel)
{
    subghz_scene_set_state(app, SubGhzSceneProtoPirateMenu, (uint32_t)sel);
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    subghz_submenu_model_init(&s_model,
                              PP_MENU_ITEM_COUNT,
                              M1_MENU_VIS(PP_MENU_ITEM_COUNT));
    subghz_submenu_model_set_selected(&s_model, pp_menu_get_saved_sel(app));
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(PP_MENU_ITEM_COUNT));
    switch (event)
    {
        case SubGhzEventBack:
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            subghz_submenu_model_up(&s_model);
            pp_menu_save_sel(app, s_model.selected);
            app->need_redraw = true;
            return true;

        case SubGhzEventDown:
            subghz_submenu_model_down(&s_model);
            pp_menu_save_sel(app, s_model.selected);
            app->need_redraw = true;
            return true;

        case SubGhzEventOk:
        {
            SubGhzSceneId target = pp_menu_targets[s_model.selected];
            if (target < SubGhzSceneCount)
                subghz_scene_push(app, target);
            return true;
        }

        default:
            break;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    (void)app;
}

static void draw(SubGhzApp *app)
{
    (void)app;
    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(PP_MENU_ITEM_COUNT));
    m1_submenu_draw(&s_model, "Proto Pirate", pp_menu_labels);
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_proto_pirate_menu_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
