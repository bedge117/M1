/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_more_raw.c
 * @brief  Sub-GHz Read-Raw "More" submenu scene (Phase 6).
 *
 * Pushed by the Read Raw scene from the RIGHT button while in the Loaded
 * state.  Provides the Momentum LoadKeyIDLE → MoreRAW submenu actions for
 * a pre-loaded RAW file:
 *
 *   - Decode → push @ref SubGhzSceneDecodeRaw to run offline protocol
 *              decoding against the currently-loaded file.
 *   - Rename → blocking virtual-keyboard rename; updates
 *              @ref SubGhzApp.raw_filepath in place.
 *   - Delete → blocking confirmation dialog; on confirm unlinks the file,
 *              clears @ref SubGhzApp.raw_filepath, and pops back to the
 *              Read Raw scene where the resume-from-child path detects the
 *              empty filepath and resets to the Start state.
 *
 * The Read Raw scene owns the file path buffer (@ref SubGhzApp.raw_filepath);
 * this scene reads it on entry and may mutate it on Rename / Delete.  All
 * three actions are blocking (VKB / message-box / decode-load), so the
 * scene's event handler runs at most once per user interaction.
 *
 * Phase 6 split: this submenu was previously inlined inside
 * `m1_subghz_scene_read_raw.c` behind `rr_in_more_menu` / `rr_more_sel`
 * file-scope statics.  Extracting it removes ~120 lines of overlay state
 * from Read Raw and aligns the architecture with Momentum's dedicated
 * MoreRAW scene.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "ff.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_submenu.h"
#include "m1_subghz_scene.h"
#include "m1_virtual_kb.h"
#include "subghz_submenu_model.h"

/*============================================================================*/
/* Scene state                                                                */
/*============================================================================*/

#define MORE_RAW_ITEM_COUNT 3

enum {
    MORE_RAW_DECODE = 0,
    MORE_RAW_RENAME,
    MORE_RAW_DELETE,
};

static const char *const more_raw_labels[MORE_RAW_ITEM_COUNT] = {
    "Decode", "Rename", "Delete",
};

/* Phase 7c-2: migrated to the reusable submenu model.  The scene only
 * owns the model itself — scroll/selection math comes from the pure-logic
 * widget and rendering from `m1_submenu_draw`.  Selection is reset each
 * time the scene is entered (matches the prior behaviour where
 * `more_raw_sel = 0` was set unconditionally in `scene_on_enter`). */
static subghz_submenu_model_t s_model;

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

/** Extract bare filename (with extension) from a full path.  Handles both
 *  '/' and '\\' separators. */
static const char *extract_filename(const char *fullpath)
{
    const char *fwd = strrchr(fullpath, '/');
    const char *bck = strrchr(fullpath, '\\');
    const char *p = (fwd && bck) ? (fwd > bck ? fwd : bck)
                  : (fwd ? fwd : bck);
    return p ? p + 1 : fullpath;
}

/*============================================================================*/
/* Action handlers                                                            */
/*============================================================================*/

static void do_rename(SubGhzApp *app)
{
    if (app->raw_filepath[0] == '\0')
        return;

    const char *fname = extract_filename(app->raw_filepath);

    /* Strip extension for the VKB prompt */
    char base[32];
    strncpy(base, fname, sizeof(base) - 1);
    base[sizeof(base) - 1] = '\0';
    char *dot = strrchr(base, '.');
    if (dot) *dot = '\0';

    char new_name[32];
    if (!m1_vkb_get_filename("Rename to:", base, new_name))
        return;

    const char *ext = strrchr(fname, '.');
    if (!ext) ext = ".sub";

    char old_path[80];
    char new_path[80];
    snprintf(old_path, sizeof(old_path), "0:%s", app->raw_filepath);
    snprintf(new_path, sizeof(new_path), "0:/SUBGHZ/%s%s", new_name, ext);

    FRESULT res = f_rename(old_path, new_path);
    if (res == FR_OK)
    {
        snprintf(app->raw_filepath, sizeof(app->raw_filepath),
                 "/SUBGHZ/%s%s", new_name, ext);
    }
    else
    {
        m1_message_box(&m1_u8g2, "Rename failed",
                       "Could not rename file", "", "BACK");
    }
}

static void do_delete(SubGhzApp *app)
{
    if (app->raw_filepath[0] == '\0')
        return;

    char short_name[32];
    strncpy(short_name, extract_filename(app->raw_filepath),
            sizeof(short_name) - 1);
    short_name[sizeof(short_name) - 1] = '\0';

    uint8_t confirm = m1_message_box_choice(
        &m1_u8g2, "Delete file?", short_name, "", "OK  /  Cancel");
    if (confirm != 1)
        return;

    char del_path[80];
    snprintf(del_path, sizeof(del_path), "0:%s", app->raw_filepath);
    FRESULT res = f_unlink(del_path);
    if ((res == FR_OK) || (res == FR_NO_FILE))
    {
        /* Clear the active filepath so the Read Raw scene's
         * resume-from-child path resets to the Start state. */
        app->raw_filepath[0] = '\0';
        app->resume_from_child = true;
        subghz_scene_pop(app);
    }
    else
    {
        m1_message_box(&m1_u8g2, "Delete failed",
                       "Could not delete file", "", "BACK");
    }
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Consume the resume_from_child flag that our parent (ReadRaw) set
     * before pushing us.  Without this, the flag leaks into our own
     * child scenes — notably DecodeRaw, which would skip load_and_decode()
     * on first entry, producing a "No protocols decoded" false negative. */
    app->resume_from_child = false;

    /* If there is no file to act on, immediately pop back. */
    if (app->raw_filepath[0] == '\0')
    {
        subghz_scene_pop(app);
        return;
    }
    subghz_submenu_model_init(&s_model,
                              MORE_RAW_ITEM_COUNT,
                              M1_MENU_VIS(MORE_RAW_ITEM_COUNT));
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* Re-sync visible_count in case the user changed the text-size
     * preference while a child scene was on top. */
    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(MORE_RAW_ITEM_COUNT));

    switch (event)
    {
        case SubGhzEventBack:
            app->resume_from_child = true;
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
            switch (s_model.selected)
            {
                case MORE_RAW_DECODE:
                    /* Decode pushes the DecodeRaw scene which loads the file
                     * and renders results.  On pop-back we stay on this menu. */
                    subghz_scene_push(app, SubGhzSceneDecodeRaw);
                    break;
                case MORE_RAW_RENAME:
                    do_rename(app);
                    app->need_redraw = true;
                    break;
                case MORE_RAW_DELETE:
                    do_delete(app);
                    /* do_delete() pops this scene on success; on failure /
                     * cancel we stay here.  Redraw is harmless in both cases. */
                    app->need_redraw = true;
                    break;
                default:
                    break;
            }
            return true;

        default:
            break;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    /* Do not modify resume_from_child here.
     *
     * subghz_scene_push() calls the current scene's on_exit() before
     * entering the pushed scene, so setting resume_from_child here would
     * leak into child scenes such as DecodeRaw and make them think they
     * are being resumed from a child return.
     *
     * Any resume_from_child handoff needed for actual pops back to
     * ReadRaw must be set explicitly by those pop/delete paths, not
     * unconditionally on every exit from this scene. */
    (void)app;
}

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

static void draw(SubGhzApp *app)
{
    /* Title: truncated filename (or "RAW" fallback) */
    char title[22];
    const char *fname = (app->raw_filepath[0] != '\0')
                        ? extract_filename(app->raw_filepath) : "RAW";
    strncpy(title, fname, sizeof(title) - 1);
    title[sizeof(title) - 1] = '\0';

    /* Re-sync visible_count for the current text-size setting before
     * drawing — matches the Phase 7c-1 Menu scene pattern. */
    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(MORE_RAW_ITEM_COUNT));
    m1_submenu_draw(&s_model, title, more_raw_labels);
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_more_raw_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
