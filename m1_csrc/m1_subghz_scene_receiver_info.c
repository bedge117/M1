/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_receiver_info.c
 * @brief  Sub-GHz Receiver Info Scene — signal detail with save/send options.
 *
 * Shows full decoded signal details. Navigation:
 *   OK    = Send (transmit static-code signal)
 *   DOWN  = Save (push SaveName scene)
 *   BACK  = Return to Read scene
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

extern bool subghz_protocol_is_static_ext(uint16_t protocol);
extern bool subghz_transmit_static_signal_ext(const SubGHz_History_Entry_t *entry);

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        case SubGhzEventBack:
            subghz_scene_pop(app);
            return true;

        case SubGhzEventOk:
        {
            /* Send signal (static-code protocols only) */
            const SubGHz_History_Entry_t *e = subghz_history_get(&app->history, app->history_sel);
            if (e && subghz_protocol_is_static_ext(e->info.protocol))
            {
                subghz_transmit_static_signal_ext(e);
                app->need_redraw = true;
            }
            return true;
        }

        case SubGhzEventDown:
            /* Save signal */
            subghz_scene_push(app, SubGhzSceneSaveName);
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
    char line[48];

    const SubGHz_History_Entry_t *e = subghz_history_get(&app->history, app->history_sel);
    if (!e)
    {
        /* Should not happen — pop back */
        return;
    }

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Protocol name (large font) */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 2, 10, protocol_text[e->info.protocol]);

    /* Separator */
    u8g2_DrawHLine(&m1_u8g2, 0, 12, M1_LCD_DISPLAY_WIDTH);

    /* Details */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    snprintf(line, sizeof(line), "Key:  0x%lX", (uint32_t)e->info.key);
    u8g2_DrawStr(&m1_u8g2, 2, 22, line);

    snprintf(line, sizeof(line), "Bits: %d    TE: %d us", e->info.bit_len, e->info.te);
    u8g2_DrawStr(&m1_u8g2, 2, 30, line);

    /* Use integer arithmetic — embedded printf (nano.specs) has no %f support */
    snprintf(line, sizeof(line), "%lu.%02lu MHz",
             (unsigned long)(e->frequency / 1000000UL),
             (unsigned long)((e->frequency % 1000000UL) / 10000UL));
    u8g2_DrawStr(&m1_u8g2, 2, 38, line);

    snprintf(line, sizeof(line), "RSSI: %d dBm", e->info.rssi);
    {
        uint8_t tw = u8g2_GetStrWidth(&m1_u8g2, line);
        u8g2_DrawStr(&m1_u8g2, M1_LCD_DISPLAY_WIDTH - tw - 2, 38, line);
    }

    /* Serial / rolling / button if available */
    if (e->info.serial_number || e->info.rolling_code)
    {
        snprintf(line, sizeof(line), "SN:%lX RC:%lX B:%d",
                 (unsigned long)e->info.serial_number,
                 (unsigned long)e->info.rolling_code,
                 e->info.button_id);
        u8g2_DrawStr(&m1_u8g2, 2, 46, line);
    }
    else if (e->count > 1)
    {
        snprintf(line, sizeof(line), "Received x%d", e->count);
        u8g2_DrawStr(&m1_u8g2, 2, 46, line);
    }

    /* Bottom bar */
    bool can_send = subghz_protocol_is_static_ext(e->info.protocol);
    subghz_button_bar_draw(
        arrowdown_8x8, "Save",
        NULL, can_send ? "Send" : NULL,
        NULL, NULL);

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_receiver_info_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
