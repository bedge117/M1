/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_menu.c
 * @brief  Sub-GHz Menu Scene — entry point with Flipper-style item list.
 *
 * Phase 7c-1 migration: rendering and scroll/selection math come from the
 * reusable `subghz_submenu_model` + `m1_submenu_draw` widget.  This scene
 * only owns the labels, the OK-dispatch target table, and a single
 * persisted byte (the selection index) in the per-scene state slot.
 *
 *   1. Read (protocol decode) — most common, at top
 *   2. Read Raw (raw capture)
 *   3. Saved (file browser)
 *   4. Playlist
 *   5. Frequency Analyzer
 *   6. Spectrum Analyzer
 *   7. RSSI Meter
 *   8. Freq Scanner
 *   9. Weather Station
 *  10. Brute Force
 *  11. Add Manually
 *  12. Remote
 *  13. Bind Remote
 *  14. Proto Pirate (rolling-code analysis — PR #579)
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_submenu.h"
#include "m1_system.h"
#include "m1_subghz_scene.h"
#include "subghz_submenu_model.h"

/*============================================================================*/
/* Menu items                                                                 */
/*============================================================================*/

#define MENU_ITEM_COUNT   14

static const char *menu_labels[MENU_ITEM_COUNT] = {
    "Read",
    "Read Raw",
    "Saved",
    "Playlist",
    "Frequency Analyzer",
    "Spectrum Analyzer",
    "RSSI Meter",
    "Freq Scanner",
    "Weather Station",
    "Brute Force",
    "Add Manually",
    "Remote",
    "Bind Remote",
    "Proto Pirate",
};

static const SubGhzSceneId menu_targets[MENU_ITEM_COUNT] = {
    SubGhzSceneRead,
    SubGhzSceneReadRaw,
    SubGhzSceneSaved,
    SubGhzScenePlaylist,
    SubGhzSceneFreqAnalyzer,
    SubGhzSceneSpectrumAnalyzer,
    SubGhzSceneRssiMeter,
    SubGhzSceneFreqScanner,
    SubGhzSceneWeatherStation,
    SubGhzSceneBruteForce,
    SubGhzSceneSetType,   /* "Add Manually" — Phase 8b-4 retired the blocking delegate */
    SubGhzSceneRemote,
    SubGhzSceneBindWizard,
    SubGhzSceneProtoPirateMenu,
};

/*============================================================================*/
/* Selection state — persisted across child-scene pushes via Phase 2          */
/* per-scene state slot.  Only `selected` needs to persist; the scroll        */
/* window is rederived by the model from the current selection + visible     */
/* count on every entry.                                                      */
/*============================================================================*/

static subghz_submenu_model_t s_model;

static inline uint8_t menu_get_saved_sel(const SubGhzApp *app)
{
    uint32_t s = subghz_scene_get_state(app, SubGhzSceneMenu);
    uint8_t sel = (uint8_t)(s & 0xFFu);
    return (sel < MENU_ITEM_COUNT) ? sel : 0;
}

static inline void menu_save_sel(SubGhzApp *app, uint8_t sel)
{
    subghz_scene_set_state(app, SubGhzSceneMenu, (uint32_t)sel);
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    subghz_submenu_model_init(&s_model,
                              MENU_ITEM_COUNT,
                              M1_MENU_VIS(MENU_ITEM_COUNT));
    subghz_submenu_model_set_selected(&s_model, menu_get_saved_sel(app));
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* Re-sync visible_count in case the user changed the text-size
     * preference while a child scene was on top. */
    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(MENU_ITEM_COUNT));

    switch (event)
    {
        case SubGhzEventBack:
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            subghz_submenu_model_up(&s_model);
            menu_save_sel(app, s_model.selected);
            app->need_redraw = true;
            return true;

        case SubGhzEventDown:
            subghz_submenu_model_down(&s_model);
            menu_save_sel(app, s_model.selected);
            app->need_redraw = true;
            return true;

        case SubGhzEventOk:
        {
            SubGhzSceneId target = menu_targets[s_model.selected];
            if (target < SubGhzSceneCount)
            {
                subghz_scene_push(app, target);
            }
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
                                           M1_MENU_VIS(MENU_ITEM_COUNT));
    m1_submenu_draw(&s_model, "Sub-GHz", menu_labels);
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_menu_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
