/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_save_success.c
 * @brief  Sub-GHz Save Success Scene — brief confirmation with auto-dismiss.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_lp5814.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"
#include "uiView.h"

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Green LED flash for success */
    m1_led_fast_blink(LED_BLINK_ON_GREEN, LED_FASTBLINK_PWM_H, LED_FASTBLINK_ONTIME_H);
    app->need_redraw = true;

    /* Auto-dismiss after 1.5 seconds via screen timeout */
    uiScreen_timeout_start(1500, NULL);
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        case SubGhzEventBack:
        case SubGhzEventOk:
        case SubGhzEventTimeout:
            /* Dismiss */
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
    uiScreen_timeout_cancel();
    /* Restore LED */
    m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
}

static void draw(SubGhzApp *app)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Success icon (checkmark) */
    u8g2_DrawXBMP(&m1_u8g2, 54, 8, 10, 10, check_10x10);

    /* Title */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 30, 124, "Saved!", TEXT_ALIGN_CENTER);

    /* Filename */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    /* Show just the filename portion of the path */
    const char *fname = app->file_path;
    const char *slash = strrchr(fname, '/');
    if (slash) fname = slash + 1;
    m1_draw_text(&m1_u8g2, 2, 42, 124, fname, TEXT_ALIGN_CENTER);

    /* Bottom bar */
    subghz_button_bar_draw(
        arrowleft_8x8, "OK",
        NULL, NULL,
        NULL, NULL);

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_save_success_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
