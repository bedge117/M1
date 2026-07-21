/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_need_saving.c
 * @brief  Sub-GHz Need Saving Scene — "Unsaved signals. Save?" dialog.
 *
 * Shown when exiting Read scene with unsaved history entries.
 */

#include <stdint.h>
#include <stdbool.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"

static uint8_t choice = 0;  /* 0 = Save, 1 = Discard */

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    (void)app;
    choice = 0;
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        case SubGhzEventBack:
            /* Go back to the scene that pushed us (don't discard yet) */
            subghz_scene_pop(app);
            return true;

        case SubGhzEventLeft:
        case SubGhzEventRight:
            choice = choice ? 0 : 1;
            app->need_redraw = true;
            return true;

        case SubGhzEventOk:
            if (choice == 0)
            {
                /* Save */
                subghz_scene_replace(app, SubGhzSceneSaveName);
            }
            else
            {
                /* Discard — pop this dialog; the calling scene should check
                 * app context to know if signals were intentionally discarded */
                subghz_history_reset(&app->history);
                app->has_decoded = false;
                subghz_scene_pop(app);
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

static void draw(SubGhzApp *app)
{
    (void)app;

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Title */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 12, 124, "Unsaved Signals!", TEXT_ALIGN_CENTER);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 24, 124, "Do you want to save", TEXT_ALIGN_CENTER);
    m1_draw_text(&m1_u8g2, 2, 33, 124, "before exiting?", TEXT_ALIGN_CENTER);

    /* Choice buttons — centered above the bottom bar (y=52) */
    u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);

    /* Save button */
    if (choice == 0)
    {
        u8g2_DrawBox(&m1_u8g2, 10, 38, 40, 10);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
    }
    u8g2_DrawStr(&m1_u8g2, 18, 46, "Save");
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Discard button */
    if (choice == 1)
    {
        u8g2_DrawBox(&m1_u8g2, 68, 38, 50, 10);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
    }
    u8g2_DrawStr(&m1_u8g2, 72, 46, "Discard");
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Bottom bar */
    subghz_button_bar_draw(
        arrowleft_8x8, "Cancel",
        NULL, "LR:Choose",
        NULL, "OK");

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_need_saving_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
