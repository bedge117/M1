/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_delete.c
 * @brief  Sub-GHz Saved-file delete confirmation scene (Phase 5).
 *
 * Pushed by the SavedMenu scene when the user picks "Delete".  Reads
 * `app->saved_filepath` / `app->saved_filename` to know which file the
 * confirmation applies to.
 *
 *   Layout:
 *     Title:   "Delete file?"
 *     Body:    <filename>  (truncated to fit)
 *     Buttons: [Cancel]  [Delete]    (LEFT=Cancel, RIGHT=Delete)
 *
 *   Inputs:
 *     LEFT / UP    → move cursor to Cancel
 *     RIGHT / DOWN → move cursor to Delete
 *     OK           → execute the selected button
 *     BACK         → equivalent to Cancel (pop back to SavedMenu)
 *
 * On confirm, `f_unlink()` is invoked and the scene pops back through
 * SavedMenu to the Saved file browser (via @ref subghz_scene_search_and_pop_to)
 * — the action menu cannot remain on screen pointing at a now-deleted file.
 *
 * On cancel, the scene pops once, returning the user to the action menu.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "ff.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_subghz_scene.h"

/*============================================================================*/
/* Scene state                                                                */
/*============================================================================*/

enum { DELETE_BTN_CANCEL = 0, DELETE_BTN_CONFIRM = 1 };

/** Currently-highlighted button.  Default = Cancel for safety. */
static uint8_t delete_cursor;

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Default focus on Cancel so an accidental OK does not delete. */
    delete_cursor = DELETE_BTN_CANCEL;
    app->need_redraw = true;
}

static void do_confirm(SubGhzApp *app)
{
    if (app->saved_filepath[0] != '\0')
    {
        char del_path[80];
        snprintf(del_path, sizeof(del_path), "0:%s", app->saved_filepath);
        f_unlink(del_path);
    }

    /* Pop back past SavedMenu to the Saved file browser.  The file the
     * action menu pointed at no longer exists; the browser is the safe
     * place to land.  search_and_pop_to falls back to a single pop if
     * Saved is not on the stack for any reason. */
    if (!subghz_scene_search_and_pop_to(app, SubGhzSceneSaved))
        subghz_scene_pop(app);
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        case SubGhzEventBack:
            /* Cancel — back to SavedMenu. */
            subghz_scene_pop(app);
            return true;

        case SubGhzEventLeft:
        case SubGhzEventUp:
            if (delete_cursor != DELETE_BTN_CANCEL)
            {
                delete_cursor = DELETE_BTN_CANCEL;
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventRight:
        case SubGhzEventDown:
            if (delete_cursor != DELETE_BTN_CONFIRM)
            {
                delete_cursor = DELETE_BTN_CONFIRM;
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventOk:
            if (delete_cursor == DELETE_BTN_CONFIRM)
                do_confirm(app);
            else
                subghz_scene_pop(app);
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

static void draw_button(int x, int y, int w, int h, const char *label, bool selected)
{
    if (selected)
    {
        u8g2_DrawRBox(&m1_u8g2, x, y, w, h, 2);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
    }
    else
    {
        u8g2_DrawRFrame(&m1_u8g2, x, y, w, h, 2);
    }
    uint8_t label_w = u8g2_GetStrWidth(&m1_u8g2, label);
    int text_x = x + (w - (int)label_w) / 2;
    int text_y = y + h - 3;
    u8g2_DrawStr(&m1_u8g2, text_x, text_y, label);
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
}

static void draw(SubGhzApp *app)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Title header — non-inverted, with separator (Hapax visual standard). */
    u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
    u8g2_DrawStr(&m1_u8g2, 2, 10, "Delete file?");
    u8g2_DrawHLine(&m1_u8g2, 0, 12, M1_LCD_DISPLAY_WIDTH);

    /* Body: filename (truncated for display). */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    char dname[26];
    strncpy(dname, app->saved_filename, sizeof(dname) - 1);
    dname[sizeof(dname) - 1] = '\0';
    u8g2_DrawStr(&m1_u8g2, 2, 28, dname);

    u8g2_DrawStr(&m1_u8g2, 2, 40, "This cannot be undone.");

    /* Two-button bar at the bottom — slim-font, individual rounded
     * buttons matching the Hapax visual standard.  Layout: 60px buttons
     * with 4px gap, centered on the 128px display. */
    const int btn_w = 60;
    const int btn_h = 12;
    const int btn_y = 50;
    const int gap   = 4;
    const int total = btn_w * 2 + gap;
    const int x0    = (M1_LCD_DISPLAY_WIDTH - total) / 2;

    draw_button(x0,              btn_y, btn_w, btn_h, "Cancel",
                delete_cursor == DELETE_BTN_CANCEL);
    draw_button(x0 + btn_w + gap, btn_y, btn_w, btn_h, "Delete",
                delete_cursor == DELETE_BTN_CONFIRM);

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_delete_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
