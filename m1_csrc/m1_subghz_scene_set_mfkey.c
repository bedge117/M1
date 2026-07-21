/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_set_mfkey.c
 * @brief  Sub-GHz Create-from-scratch KeeLoq SetMfKey picker scene
 *         (Phase 8c-2 picker + Phase 8c-3 final assembly).
 *
 * Scrollable list picker over the loaded KeeLoq manufacturer-key table
 * (built-in flash or SD-card fallback — see
 * `Sub_Ghz/subghz_keeloq_mfkeys.{h,c}`).  Lets the user pick the
 * manufacturer whose master key + learn type should be used to encrypt
 * the assembled hop word.
 *
 * On OK (Phase 8c-3): the picked manufacturer name is stored in
 * @ref SubGhzApp::create_mfkey_name, the 64-bit Flipper-format key is
 * built via @ref subghz_keeloq_create_key, written to a temp `.sub`
 * file with the `Manufacture:` field set, and the Transmitter scene is
 * pushed with `tx_autostart=true` so the signal fires immediately.  On
 * resume-from-child the temp file is unlinked and the picker keeps its
 * selection so the user can fire again with one OK press.
 *
 * If the keystore is empty (host build, missing SD card, or vault
 * absent) the picker draws a short "No KeeLoq mfkeys loaded" placeholder
 * and BACK pops out without modifying the app state.
 *
 * Selection persists across pushes/pops via the Phase 2 per-scene state
 * slot keyed by @ref SubGhzSceneSetMfKey so re-entering the picker
 * returns the user to the same row.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ff.h"
#include "u8g2.h"

#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_submenu.h"
#include "m1_subghz_scene.h"
#include "flipper_subghz.h"
#include "subghz_create_proto.h"
#include "subghz_keeloq_create.h"
#include "subghz_keeloq_mfkeys.h"
#include "subghz_submenu_model.h"

/*============================================================================*/
/* Scene state                                                                */
/*============================================================================*/

/** Maximum number of manufacturer-key entries shown in the picker.  The
 *  builtin vault and typical SD-card keystore both stay well under this
 *  cap; entries beyond it are simply not displayed (BACK and BACK-from-
 *  parent still work).  Sized to keep this scene's static footprint
 *  reasonable — each label slot is a single name pointer. */
#define SETMFKEY_MAX_ENTRIES   64U

/** Temp file path used to hand the user-built KeeLoq key to the
 *  Transmitter scene.  Same lifetime contract as the SetKey scene's
 *  temp file: created on OK, unlinked on resume-from-child / BACK /
 *  scene_on_exit / assembly failure. */
#define SETMFKEY_TMP_PATH   "0:/SUBGHZ/_setkl_tmp.sub"

static subghz_submenu_model_t s_model;
/** Cached copy of each visible entry's name (NUL-terminated).  We keep
 *  our own copy rather than pointing into the mfkeys table because the
 *  table could in principle be freed/reloaded between scene entries. */
static char        s_names[SETMFKEY_MAX_ENTRIES][sizeof(((KeeLoqMfrEntry*)0)->name)];
static const char *s_labels[SETMFKEY_MAX_ENTRIES];
static uint32_t    s_loaded_count;
/** Brief "Save failed" / "Bad protocol" overlay flag — cleared on next
 *  keypress.  Mirrors the SetKey scene's overflow-flash pattern. */
static bool        s_error_flash;

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

static void load_entries(void)
{
    /* Lazy-load the manufacturer-key store on first entry.  This ensures
     * build-time embedded keys (from subghz_keeloq_mfkeys_builtin.c, generated
     * by scripts/gen_keeloq_mfkeys_builtin.py when the KEELOQ_KEY_VAULT secret
     * is set) are visible in the picker even when the user has never opened
     * a saved KeeLoq file yet.  Mirrors the lazy-load guard in
     * m1_sub_ghz.c::sub_ghz_replay_flipper_file(). */
    if (keeloq_mfkeys_count() == 0)
        keeloq_mfkeys_load();

    uint32_t total = keeloq_mfkeys_count();
    if (total > SETMFKEY_MAX_ENTRIES)
        total = SETMFKEY_MAX_ENTRIES;

    s_loaded_count = 0U;
    for (uint32_t i = 0U; i < total; ++i)
    {
        KeeLoqMfrEntry e;
        if (!keeloq_mfkeys_get_at(i, &e))
            break;
        /* Truncate-safe copy of the name into our scene-local buffer. */
        strncpy(s_names[s_loaded_count], e.name, sizeof(s_names[0]) - 1U);
        s_names[s_loaded_count][sizeof(s_names[0]) - 1U] = '\0';
        s_labels[s_loaded_count] = s_names[s_loaded_count];
        ++s_loaded_count;
    }
}

/** Find the index of @p name in the cached label list, or UINT32_MAX. */
static uint32_t find_index_by_name(const char *name)
{
    if (!name || name[0] == '\0')
        return UINT32_MAX;
    for (uint32_t i = 0U; i < s_loaded_count; ++i)
    {
        if (strncmp(s_names[i], name, sizeof(s_names[0])) == 0)
            return i;
    }
    return UINT32_MAX;
}

static void unlink_tmp(void)
{
    f_unlink(SETMFKEY_TMP_PATH);
}

/** Assemble the 64-bit Flipper-format KeeLoq key from the picked
 *  manufacturer and the previously-entered serial/button/counter, write
 *  it to a temp .sub with `Manufacture:` set, and push the Transmitter
 *  scene with `tx_autostart=true` so the signal fires immediately.
 *  Returns true on success; false on lookup / assembly / write failure
 *  (in which case the error flash overlay is shown). */
static bool push_transmitter_with_assembled_key(SubGhzApp *app)
{
    const SubGhzCreateProtoSpec *spec = subghz_create_proto_spec(
        (SubGhzCreateProtoId)app->create_proto_id);
    if (!spec || !spec->proto_name)
        return false;

    /* Look up the manufacturer key + learn type by name. */
    KeeLoqMfrEntry mfr;
    if (!keeloq_mfkeys_find(app->create_mfkey_name, &mfr))
        return false;

    /* Assemble the 64-bit Flipper-format key with the picked
     * serial / button / counter / mfr_key / learn type. */
    KeeLoqCreateParams cparams;
    cparams.protocol   = spec->proto_name;
    cparams.serial     = app->create_serial;
    cparams.button     = app->create_button;
    cparams.counter    = app->create_counter;
    cparams.mfr_key    = mfr.key;
    cparams.learn_type = mfr.learn_type;

    uint64_t key64 = 0ULL;
    if (subghz_keeloq_create_key(&cparams, &key64) != KEELOQ_CREATE_OK)
        return false;

    /* Pre-clean any stale file from a previous push. */
    unlink_tmp();

    /* Write the temp .sub with Manufacture: set so the replay path can
     * decrypt → increment → re-encrypt via the existing KeeLoq
     * counter-mode encoder. */
    if (!flipper_subghz_save_key_with_manufacture(
            SETMFKEY_TMP_PATH,
            spec->freq_hz,
            "FuriHalSubGhzPresetOok650Async",
            spec->proto_name,
            spec->bit_count,
            key64,
            spec->te,
            mfr.name))
    {
        return false;
    }

    /* Populate Transmitter inputs — same contract as the SetKey scene. */
    strncpy(app->tx_path, SETMFKEY_TMP_PATH, sizeof(app->tx_path) - 1);
    app->tx_path[sizeof(app->tx_path) - 1] = '\0';
    app->tx_repeat_count = 1U;
    app->tx_mode         = 0U;          /* SUBGHZ_TX_MODE_SINGLE */
    app->tx_autostart    = true;

    strncpy(app->tx_protocol_name, spec->proto_name,
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
    load_entries();

    subghz_submenu_model_init(&s_model,
                              (uint8_t)s_loaded_count,
                              M1_MENU_VIS(s_loaded_count));

    /* On resume-from-child (returned from Transmitter), unlink the
     * temp file but keep the selection intact so the user can re-fire
     * the same manufacturer with one OK press. */
    if (app->resume_from_child)
    {
        app->resume_from_child = false;
        unlink_tmp();
    }

    /* Prefer the currently-selected mfkey (if it's still in the table) as
     * the initial cursor position; otherwise restore the last cursor
     * position from the per-scene state slot. */
    uint32_t want = find_index_by_name(app->create_mfkey_name);
    if (want == UINT32_MAX)
        want = subghz_scene_get_state(app, SubGhzSceneSetMfKey);

    if (want < s_loaded_count)
    {
        while (s_model.selected < want)
            subghz_submenu_model_down(&s_model);
        while (s_model.selected > want)
            subghz_submenu_model_up(&s_model);
    }

    s_error_flash = false;
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* Any keypress dismisses the error flash before doing anything else. */
    if (s_error_flash)
    {
        s_error_flash = false;
        app->need_redraw = true;
    }

    /* Resync visible_count on every event in case the user changed the
     * text-size preference while a child scene was on top. */
    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(s_loaded_count));

    switch (event)
    {
        case SubGhzEventBack:
            unlink_tmp();
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            if (s_loaded_count > 0U)
            {
                subghz_submenu_model_up(&s_model);
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventDown:
            if (s_loaded_count > 0U)
            {
                subghz_submenu_model_down(&s_model);
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventOk:
            if (s_loaded_count > 0U && s_model.selected < s_loaded_count)
            {
                /* Commit the user's pick before assembling the key. */
                strncpy(app->create_mfkey_name,
                        s_names[s_model.selected],
                        sizeof(app->create_mfkey_name) - 1U);
                app->create_mfkey_name[sizeof(app->create_mfkey_name) - 1U] = '\0';
                subghz_scene_set_state(app, SubGhzSceneSetMfKey,
                                       s_model.selected);

                /* Phase 8c-3 — assemble the 64-bit key and push the
                 * Transmitter.  On failure surface a brief error flash
                 * so the user sees that something went wrong instead of
                 * the picker silently doing nothing. */
                if (!push_transmitter_with_assembled_key(app))
                {
                    s_error_flash = true;
                    app->need_redraw = true;
                }
            }
            return true;

        default:
            break;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    /* When pushing the Transmitter scene, subghz_scene_push() calls our
     * on_exit BEFORE the Transmitter's on_enter consumes the temp .sub
     * file.  push_transmitter_with_assembled_key() sets
     * app->resume_from_child=true immediately before the push, so we
     * use that flag as the "we're handing off, do not unlink" gate.
     * On a real exit (BACK / search_and_pop) the flag is false and the
     * defensive unlink runs as before. */
    if (app && app->resume_from_child)
        return;
    unlink_tmp();
}

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

static void draw_empty(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    u8g2_DrawStr(&m1_u8g2, 2, 10, "Manufacturer");
    u8g2_DrawHLine(&m1_u8g2, 0, 11, M1_LCD_DISPLAY_WIDTH);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 0, 30, M1_LCD_DISPLAY_WIDTH,
                 "No KeeLoq mfkeys", TEXT_ALIGN_CENTER);
    m1_draw_text(&m1_u8g2, 0, 40, M1_LCD_DISPLAY_WIDTH,
                 "loaded", TEXT_ALIGN_CENTER);

    m1_u8g2_nextpage();
}

static void draw_error_flash(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    u8g2_DrawStr(&m1_u8g2, 2, 10, "Manufacturer");
    u8g2_DrawHLine(&m1_u8g2, 0, 11, M1_LCD_DISPLAY_WIDTH);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    m1_draw_text(&m1_u8g2, 0, 30, M1_LCD_DISPLAY_WIDTH,
                 "Save failed", TEXT_ALIGN_CENTER);
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 0, 42, M1_LCD_DISPLAY_WIDTH,
                 "Press any key", TEXT_ALIGN_CENTER);

    m1_u8g2_nextpage();
}

static void draw(SubGhzApp *app)
{
    (void)app;

    if (s_error_flash)
    {
        draw_error_flash();
        return;
    }

    if (s_loaded_count == 0U)
    {
        draw_empty();
        return;
    }

    subghz_submenu_model_set_visible_count(&s_model,
                                           M1_MENU_VIS(s_loaded_count));
    m1_submenu_draw(&s_model, "Manufacturer", s_labels);
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_set_mfkey_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
