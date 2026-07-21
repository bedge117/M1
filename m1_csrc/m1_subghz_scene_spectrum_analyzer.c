/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_spectrum_analyzer.c
 * @brief  Sub-GHz Spectrum Analyzer Scene — bar-graph spectrum display.
 *
 * Delegates to existing sub_ghz_spectrum_analyzer() which handles its own
 * event loop.  This scene wrapper provides proper integration with the
 * scene manager.
 */

#include <stdint.h>
#include <stdbool.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_subghz_scene.h"
#include "m1_sub_ghz.h"

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    (void)app;
    sub_ghz_spectrum_analyzer();

    /* After the blocking call returns, pop this scene */
    app->running = true;
    subghz_scene_pop(app);
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    (void)app;
    (void)event;
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    (void)app;
}

static void draw(SubGhzApp *app)
{
    (void)app;
    /* The spectrum analyzer handles its own drawing */
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_spectrum_analyzer_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
