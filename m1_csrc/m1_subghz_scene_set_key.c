/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_set_key.c
 * @brief  Sub-GHz Create-from-scratch SetKey hex-entry scene (Phase 8b-3).
 *
 * Pushed by the Phase 8b-2 SetType picker after the user picks a
 * protocol.  Renders a hex-digit editor for the protocol's key bit
 * width (driven by the @ref SubGhzCreateProtoSpec catalog entry) and
 * lets the user build a 64-bit key with UP/DOWN cycling a single
 * digit and LEFT/RIGHT moving the cursor.  On OK the value is masked
 * via @ref subghz_create_proto_key_in_range, written to a temp `.sub`
 * file via @ref flipper_subghz_save_key, and the @ref
 * SubGhzSceneTransmitter scene is pushed with `tx_autostart=true` so
 * the signal fires immediately.
 *
 * The hex-digit editing logic lives in the host-tested pure-logic
 * `subghz_hex_editor` module so the same state machine can drive the
 * upcoming Phase 8c SetSerial / SetButton / SetCounter editor scenes
 * without duplicating cursor/digit-cycling code.
 *
 * Hardware-coupled UI rendering only — the underlying digit/cursor
 * state lives in a host-tested pure module.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "u8g2.h"

#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_subghz_scene.h"
#include "flipper_subghz.h"
#include "subghz_create_proto.h"
#include "subghz_hex_editor.h"

/*============================================================================*/
/* Scene state                                                                */
/*============================================================================*/

/** Temp file path used to hand the user-built key to the Transmitter
 *  scene.  The Transmitter calls `sub_ghz_replay_prepare_flipper()`
 *  which spins its own intermediate `_flipper_tmp.sgh`, so this `.sub`
 *  is purely an input artefact owned by SetKey.  We unlink it on
 *  resume-from-child and on scene_on_exit. */
#define SETKEY_TMP_PATH   "0:/SUBGHZ/_setkey_tmp.sub"

static subghz_hex_editor_t s_editor;
static const SubGhzCreateProtoSpec *s_spec;
static bool s_overflow_flash;   /**< Brief "Key out of range" overlay flag */

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

static void unlink_tmp(void)
{
    f_unlink(SETKEY_TMP_PATH);
}

static void load_spec(SubGhzApp *app)
{
    s_spec = subghz_create_proto_spec((SubGhzCreateProtoId)app->create_proto_id);
}

/** Push SubGhzSceneTransmitter with the user-built key written to the
 *  temp .sub file.  Returns true on success; false if the file write
 *  or value range check fails. */
static bool push_transmitter_with_key(SubGhzApp *app)
{
    if (!s_spec)
        return false;

    /* Mask the assembled value via the catalog helper.  The editor
     * already masks to bit_count, so this is belt-and-braces. */
    uint64_t raw   = subghz_hex_editor_value(&s_editor);
    uint64_t masked = 0ULL;
    bool in_range = subghz_create_proto_key_in_range(
        (SubGhzCreateProtoId)app->create_proto_id, raw, &masked);

    /* The editor's mask matches the catalog's mask, so overflow can
     * only happen if the catalog spec's bit_count differs from the
     * editor's bit_count — which shouldn't happen because we init the
     * editor from the same spec.  Still, surface the failure as a
     * visible error rather than silently truncating. */
    if (!in_range)
    {
        s_overflow_flash = true;
        app->need_redraw = true;
        return false;
    }

    /* Write the temp .sub file.  Pre-clean any stale file from a
     * previous push so we never accidentally fire an old key. */
    unlink_tmp();

    if (!flipper_subghz_save_key(SETKEY_TMP_PATH,
                                 s_spec->freq_hz,
                                 "FuriHalSubGhzPresetOok650Async",
                                 s_spec->proto_name,
                                 s_spec->bit_count,
                                 masked,
                                 s_spec->te))
    {
        s_overflow_flash = true;  /* reuse the error flash for write failure */
        app->need_redraw = true;
        return false;
    }

    /* Populate Transmitter inputs.  SETKEY_TMP_PATH fits comfortably
     * in tx_path (24 chars vs 72). */
    strncpy(app->tx_path, SETKEY_TMP_PATH, sizeof(app->tx_path) - 1);
    app->tx_path[sizeof(app->tx_path) - 1] = '\0';
    app->tx_repeat_count = 1U;    /* static / one-shot OOK */
    app->tx_mode         = 0U;    /* SUBGHZ_TX_MODE_SINGLE */
    app->tx_autostart    = true;  /* one-press fire UX, matches Add Manually */

    /* Pass the protocol name so the Transmitter can decide whether to
     * enable button cycling.  Most Create-from-scratch protocols are
     * static OOK families and won't enable cycling, but rolling-code
     * remotes (KeeLoq-family in Phase 8c) will. */
    strncpy(app->tx_protocol_name, s_spec->proto_name,
            sizeof(app->tx_protocol_name) - 1);
    app->tx_protocol_name[sizeof(app->tx_protocol_name) - 1] = '\0';

    app->resume_from_child = true;
    subghz_scene_push(app, SubGhzSceneTransmitter);
    return true;
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    load_spec(app);

    /* On resume-from-child (returned from Transmitter), unlink the
     * temp file but keep the editor state intact so the user can
     * tweak a digit and fire again. */
    if (app->resume_from_child)
    {
        app->resume_from_child = false;
        unlink_tmp();
    }
    else
    {
        /* Fresh entry — initialise the editor for the picked protocol.
         * If the spec is missing (defensive: only possible if the SetType
         * scene set create_proto_id out of range) we still init to 1 bit
         * so the editor is in a sane state and a BACK from this scene
         * returns the user to SetType without a crash. */
        uint8_t bits = (s_spec ? (uint8_t)s_spec->bit_count : 1U);
        subghz_hex_editor_init(&s_editor, bits);
    }

    s_overflow_flash = false;
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* Any keypress dismisses the overflow flash before doing anything
     * else, so the user gets the next frame back. */
    if (s_overflow_flash)
    {
        s_overflow_flash = false;
        app->need_redraw = true;
    }

    switch (event)
    {
        case SubGhzEventBack:
            unlink_tmp();
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
            push_transmitter_with_key(app);
            return true;

        default:
            break;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    /* When pushing the Transmitter scene, subghz_scene_push() calls our
     * on_exit BEFORE the Transmitter's on_enter — and the Transmitter's
     * on_enter is what consumes the temp .sub file.  push_transmitter
     * sets app->resume_from_child=true immediately before the push, so
     * we use that flag as the "we're handing off, do not unlink" gate.
     * On a real exit (BACK / search_and_pop) the flag is false and the
     * defensive unlink runs as before. */
    if (app && app->resume_from_child)
        return;
    unlink_tmp();
}

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

static void draw_overflow_flash(void)
{
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    m1_draw_text(&m1_u8g2, 0, 30, M1_LCD_DISPLAY_WIDTH,
                 "Save failed", TEXT_ALIGN_CENTER);
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 0, 42, M1_LCD_DISPLAY_WIDTH,
                 "Press any key", TEXT_ALIGN_CENTER);
}

static void draw(SubGhzApp *app)
{
    (void)app;

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Title — protocol label */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    u8g2_DrawStr(&m1_u8g2, 2, 10,
                 (s_spec && s_spec->label) ? s_spec->label : "Set Key");

    if (s_overflow_flash)
    {
        draw_overflow_flash();
        m1_u8g2_nextpage();
        return;
    }

    /* Frequency line */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    if (s_spec)
    {
        char freq_str[20];
        snprintf(freq_str, sizeof(freq_str), "%lu.%02lu MHz",
                 (unsigned long)(s_spec->freq_hz / 1000000UL),
                 (unsigned long)((s_spec->freq_hz % 1000000UL) / 10000UL));
        u8g2_DrawStr(&m1_u8g2, 2, 20, freq_str);
    }

    /* Hex key in large font with cursor underline.  Layout matches
     * the legacy Add Manually screen exactly — "0x" prefix, 8 px per
     * digit, underline 7 px wide directly beneath the cursor. */
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

    /* Footer hint bar — same shorthand as Add Manually so users moving
     * from the legacy delegate see the same prompt. */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 0, 56, "\x18\x19:Hex L/R:Move OK:Send");

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_set_key_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
