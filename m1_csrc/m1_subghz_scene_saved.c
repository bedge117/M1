/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_saved.c
 * @brief  Sub-GHz Saved Scene — pure file browser over 0:/SUBGHZ/.
 *
 * Phase 5 split: the action menu (Decode / Emulate / Info / Rename / Delete),
 * the informational "Signal Info" screen, the offline decode-results screen,
 * and the delete-confirmation dialog have been extracted into dedicated
 * scenes:
 *
 *   - @ref SubGhzSceneSavedMenu  — action menu + info + decode results
 *   - @ref SubGhzSceneDelete     — delete-confirmation dialog
 *
 * This scene is now responsible only for opening the file browser and, on
 * selection, copying the chosen file's path/name into the shared
 * @ref SubGhzApp.saved_filepath / @ref SubGhzApp.saved_filename fields
 * before pushing @ref SubGhzSceneSavedMenu.
 *
 * Navigation:
 *   - User cancels the browser → pops back to the Sub-GHz menu.
 *   - User selects a file      → pushes SavedMenu (which on return brings
 *                                us back here and re-opens the browser).
 *   - SavedMenu / Delete pops back → on_enter fires, the browser re-opens
 *                                automatically; this is the same UX as
 *                                Momentum / Flipper "return to list after
 *                                an action".
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_storage.h"
#include "m1_scene.h"
#include "m1_subghz_scene.h"

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

/**
 * @brief  Open the 0:/SUBGHZ/ file browser and populate the shared
 *         @ref SubGhzApp.saved_filepath / @ref SubGhzApp.saved_filename
 *         fields on selection.
 *
 * @retval true   A file was selected (caller should push SavedMenu).
 * @retval false  User cancelled the browser (caller should pop this scene).
 */
static bool open_saved_browser(SubGhzApp *app)
{
    S_M1_file_info *f_info = storage_browse("0:/SUBGHZ");
    if (!f_info || !f_info->file_is_selected)
        return false;

    snprintf(app->saved_filepath, sizeof(app->saved_filepath), "/SUBGHZ/%s",
             f_info->file_name);
    strncpy(app->saved_filename, f_info->file_name, sizeof(app->saved_filename) - 1);
    app->saved_filename[sizeof(app->saved_filename) - 1] = '\0';
    return true;
}

static void scene_on_enter(SubGhzApp *app)
{
    if (!open_saved_browser(app))
    {
        /* User cancelled the file browser — pop back to Sub-GHz menu. */
        app->saved_filepath[0] = '\0';
        app->saved_filename[0] = '\0';
        subghz_scene_pop(app);
        return;
    }
    /* File chosen — hand off to the action-menu scene. */
    subghz_scene_push(app, SubGhzSceneSavedMenu);
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* The file browser blocks inside open_saved_browser(); by the time we
     * receive an event the scene has already either pushed SavedMenu or
     * popped itself.  This handler exists to satisfy the scene-manager
     * contract — any stray event simply re-runs the entry logic. */
    (void)event;
    if (!open_saved_browser(app))
    {
        app->saved_filepath[0] = '\0';
        app->saved_filename[0] = '\0';
        subghz_scene_pop(app);
        return true;
    }
    subghz_scene_push(app, SubGhzSceneSavedMenu);
    return true;
}

static void scene_on_exit(SubGhzApp *app)
{
    (void)app;
}

static void draw(SubGhzApp *app)
{
    /* The file browser owns the display while it is active; once it
     * returns we have already transitioned to another scene.  This draw
     * is only reached if a redraw is requested between the browser
     * closing and the push/pop completing, which would be transient. */
    (void)app;
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 2, 32, "Loading...");
    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_saved_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
