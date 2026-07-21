/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_set_button.c
 * @brief  Sub-GHz Create-from-scratch KeeLoq SetButton editor scene (Phase 8c-2).
 *
 * Hex-digit editor for the KeeLoq-family button code.  The editor's bit
 * width is taken from the picked protocol's
 * @ref SubGhzCreateProtoSpec::button_bits (4 bits for the KeeLoq /
 * Star Line / Jarolift entries shipped by Phase 8c-1).  On OK the
 * assembled value is masked to `button_bits` and persisted into
 * @ref SubGhzApp::create_button; the scene pops back to whichever
 * scene pushed it.  Phase 8c-3 will push @ref SubGhzSceneSetCounter next.
 *
 * Hardware-coupled UI rendering only — the underlying digit/cursor
 * state lives in the host-tested pure `subghz_hex_editor` module.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "u8g2.h"

#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_subghz_scene.h"
#include "subghz_create_proto.h"
#include "subghz_hex_editor.h"

/*============================================================================*/
/* Scene state                                                                */
/*============================================================================*/

static subghz_hex_editor_t          s_editor;
static const SubGhzCreateProtoSpec *s_spec;

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

static uint8_t button_bits_for(SubGhzApp *app)
{
    /* Phase 9b — edit-signal mode reuses this scene for the KeeLoq family
     * (KeeLoq / Star Line / Jarolift), all of which use a 4-bit button.
     * The cached signal-fields module lives in
     * `m1_subghz_scene_signal_settings.c`; the spec table is only
     * relevant in Create-from-scratch mode. */
    if (app->signal_edit_active)
    {
        s_spec = NULL;
        return 4U;
    }
    s_spec = subghz_create_proto_spec((SubGhzCreateProtoId)app->create_proto_id);
    if (s_spec && s_spec->button_bits > 0U)
        return s_spec->button_bits;
    return 1U;
}

static uint64_t button_mask_for(uint8_t bits)
{
    return (bits >= 64U) ? ~0ULL : ((1ULL << bits) - 1ULL);
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    uint8_t bits = button_bits_for(app);
    subghz_hex_editor_init(&s_editor, bits);
    /* Phase 9b — seed from the loaded signal's button when editing a
     * .sub file; otherwise from `create_button` (Create-from-scratch). */
    uint64_t initial = app->signal_edit_active
                       ? (uint64_t)subghz_signal_settings_get_button()
                       : (uint64_t)app->create_button;
    subghz_hex_editor_set_value(&s_editor, initial & button_mask_for(bits));
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        case SubGhzEventBack:
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            subghz_hex_editor_up(&s_editor);
            app->need_redraw = true;
            return true;

        case SubGhzEventDown:
            subghz_hex_editor_down(&s_editor);
            app->need_redraw = true;
            return true;

        case SubGhzEventLeft:
            subghz_hex_editor_left(&s_editor);
            app->need_redraw = true;
            return true;

        case SubGhzEventRight:
            subghz_hex_editor_right(&s_editor);
            app->need_redraw = true;
            return true;

        case SubGhzEventOk:
        {
            uint8_t  bits   = s_editor.bit_count;
            uint64_t masked = subghz_hex_editor_value(&s_editor) & button_mask_for(bits);

            if (app->signal_edit_active)
            {
                /* Phase 9b — save back to the loaded .sub file via the
                 * SignalSettings cross-scene API.  Whether the save
                 * succeeds or fails we pop back to SignalSettings so
                 * the user is never trapped here; the on_enter there
                 * will reload from disk and reflect reality (and clear
                 * the edit-active flag).  Future polish (9c+) can add
                 * an error confirm dialog on save failure. */
                (void)subghz_signal_settings_apply_button((uint8_t)masked);
                subghz_scene_pop(app);
                return true;
            }

            app->create_button = (uint8_t)masked;
            /* Create-from-scratch flow (Phase 8c-3) — chain forward to
             * the Counter editor. */
            subghz_scene_push(app, SubGhzSceneSetCounter);
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

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

static void draw(SubGhzApp *app)
{
    (void)app;

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    u8g2_DrawStr(&m1_u8g2, 2, 10, "Button");

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    if (s_spec && s_spec->label)
    {
        char sub[28];
        snprintf(sub, sizeof(sub), "%s  %ub",
                 s_spec->label, (unsigned)s_editor.bit_count);
        u8g2_DrawStr(&m1_u8g2, 2, 20, sub);
    }

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    char hex_str[2 + SUBGHZ_HEX_EDITOR_MAX_DIGITS + 1];
    int p = 0;
    hex_str[p++] = '0';
    hex_str[p++] = 'x';
    for (uint8_t d = 0U; d < s_editor.digit_count; ++d)
        hex_str[p++] = "0123456789ABCDEF"[s_editor.digits[d] & 0x0FU];
    hex_str[p] = '\0';
    u8g2_DrawStr(&m1_u8g2, 4, 38, hex_str);

    uint8_t cx = (uint8_t)(4U + ((uint16_t)s_editor.cursor + 2U) * 8U);
    u8g2_DrawHLine(&m1_u8g2, cx, 40, 7);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 0, 56, "\x18\x19:Hex L/R:Move OK:Save");

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_set_button_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
