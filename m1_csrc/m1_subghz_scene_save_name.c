/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_save_name.c
 * @brief  Sub-GHz Save Name Scene — virtual keyboard for filename entry.
 *
 * Auto-generates a default name from protocol + key, allows user to edit
 * or accept. Saves in the format selected by the user in Config (Flipper
 * .sub or M1 native .sgh).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_virtual_kb.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"
#include "m1_sub_ghz_decenc.h"
#include "flipper_subghz.h"

extern const char *subghz_freq_labels[];
extern uint8_t subghz_get_save_fmt_ext(void);

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Build default filename from protocol + key */
    const SubGHz_History_Entry_t *e = subghz_history_get(&app->history, app->history_sel);
    if (e)
    {
        snprintf(app->file_name, sizeof(app->file_name), "%.12s_%lX",
                 protocol_text[e->info.protocol],
                 (unsigned long)(uint32_t)e->info.key);
    }
    else
    {
        snprintf(app->file_name, sizeof(app->file_name), "signal_%lu",
                 (unsigned long)HAL_GetTick());
    }

    /* Run the blocking VKB + save flow immediately on enter */
    if (!e)
    {
        subghz_scene_pop(app);
        return;
    }

    char new_name[32];
    if (!m1_vkb_get_filename("Save signal as:", app->file_name, new_name))
    {
        /* User cancelled */
        subghz_scene_pop(app);
        return;
    }

    /* Choose file extension based on user's save format preference */
    uint8_t fmt = subghz_get_save_fmt_ext();
    const char *ext = (fmt == 1) ? ".sgh" : ".sub";
    snprintf(app->file_path, sizeof(app->file_path), "/SUBGHZ/%s%s", new_name, ext);

    /* Ensure the SUBGHZ directory exists before attempting to save.
     * FR_EXIST is only acceptable if the existing path is actually a directory. */
    FRESULT mkdir_res = f_mkdir("/SUBGHZ");
    if (mkdir_res == FR_EXIST)
    {
        FILINFO dir_info;
        FRESULT stat_res = f_stat("/SUBGHZ", &dir_info);
        if ((stat_res != FR_OK) || !(dir_info.fattrib & AM_DIR))
        {
            subghz_scene_pop(app);
            return;
        }
    }
    else if (mkdir_res != FR_OK)
    {
        subghz_scene_pop(app);
        return;
    }

    bool saved;
    if (fmt == 1)
        saved = flipper_subghz_save_m1native_key(app->file_path,
                    e->frequency, "FuriHalSubGhzPresetOok650Async",
                    protocol_text[e->info.protocol],
                    e->info.bit_len, e->info.key, e->info.te);
    else
        saved = flipper_subghz_save_key(app->file_path,
                    e->frequency, "FuriHalSubGhzPresetOok650Async",
                    protocol_text[e->info.protocol],
                    e->info.bit_len, e->info.key, e->info.te);

    if (saved)
    {
        /* Replace this scene with success screen */
        subghz_scene_replace(app, SubGhzSceneSaveSuccess);
    }
    else
    {
        /* Save failed — pop back */
        subghz_scene_pop(app);
    }
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* VKB is blocking so no events expected here */
    if (event == SubGhzEventBack)
    {
        subghz_scene_pop(app);
        return true;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    (void)app;
}

static void draw(SubGhzApp *app)
{
    /* VKB handles its own drawing in scene_on_enter; this is a no-op placeholder */
    (void)app;
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_save_name_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
