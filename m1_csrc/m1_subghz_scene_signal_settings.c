/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_signal_settings.c
 * @brief  Sub-GHz per-file SignalSettings scene — Phase 9a-2 scaffold.
 *
 * Read-only display of the protocol-specific signal fields (Serial,
 * Button, and the encrypted HOP word) extracted from the currently-
 * loaded .sub file pointed to by `app->saved_filepath`.  Pushed by the
 * Saved action menu when the user picks the new "Settings" entry.
 *
 * Phase 9a-2 scope (this file): read-only display for the KeeLoq family
 * (KeeLoq / Star Line / Jarolift), using the host-tested pure-logic
 * extractor in `Sub_Ghz/subghz_signal_fields.c` (Phase 9a-1).  Editing
 * lands in Phases 9b (button) and 9c (counter); extension to
 * Nice FloR-S / CAME Atomo / Alutech AT-4N / Phoenix V2 lands in 9e.
 *
 * Navigation:
 *   - BACK pops back to the SavedMenu action menu.
 *   - OK is a no-op in this scaffold (will push SetButton in 9b).
 *
 * Field extraction is fully reversible (see subghz_signal_fields.h);
 * the display is non-destructive — no file I/O happens beyond the
 * `flipper_subghz_load()` invocation on scene_on_enter.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_subghz_scene.h"
#include "flipper_subghz.h"
#include "subghz_signal_fields.h"
#include "subghz_keeloq.h"
#include "subghz_keeloq_mfkeys.h"
#include "subghz_nice_flor_s.h"
#include "subghz_nice_flor_s_table_builtin.h"
#include "subghz_came_atomo.h"
#include "subghz_alutech_at_4n.h"
#include "subghz_alutech_at_4n_table_builtin.h"

/*============================================================================*/
/* Local state                                                                 */
/*============================================================================*/

/** Cached .sub metadata loaded on scene_on_enter from app->saved_filepath. */
static flipper_subghz_signal_t s_signal;
/** Extracted plaintext fields (valid only when s_supported == true and
 *  s_is_nice_flor_s == false and s_is_came_atomo == false and
 *  s_is_alutech_at_4n == false). */
static subghz_keeloq_fields_t  s_fields;
/** True when the loaded protocol is a recognised supported member. */
static bool                    s_supported;
/** True when the loaded file's protocol is Nice FloR-S. */
static bool                    s_is_nice_flor_s;
/** Extracted Nice FloR-S fields (valid only when s_is_nice_flor_s). */
static subghz_nice_flor_s_fields_t s_nfs_fields;
/** True when the loaded file's protocol is CAME Atomo. */
static bool                    s_is_came_atomo;
/** Extracted CAME Atomo fields (valid only when s_is_came_atomo). */
static subghz_came_atomo_fields_t s_atomo_fields;
/** True when the loaded file's protocol is Alutech AT-4N. */
static bool                    s_is_alutech_at_4n;
/** Extracted Alutech AT-4N fields (valid only when s_is_alutech_at_4n). */
static subghz_alutech_at_4n_fields_t s_alutech_fields;
/** True when the file load itself succeeded — drives the error placeholder. */
static bool                    s_loaded;
/** True when the loaded file's manufacturer key was resolved and the
 *  16-bit rolling counter was decoded successfully.  When false the scene
 *  shows a "key?" placeholder instead of a numeric counter. */
static bool                    s_has_counter;
/** Decoded 16-bit rolling counter (valid iff s_has_counter == true). */
static uint16_t                s_counter;
/** Cached 64-bit derived device key — populated alongside s_counter by
 *  resolve_counter() and reused by apply_counter() to re-encrypt the
 *  updated HOP word without re-deriving from the mfkey table. */
static uint64_t                s_device_key;
/** Cached absolute load path ("0:/SUBGHZ/<name>") used by the Phase 9b
 *  save-back helper (`subghz_signal_settings_apply_button`).  Populated
 *  by scene_on_enter from `app->saved_filepath`; cleared on exit so a
 *  stale path can never be written to. */
static char                    s_saved_full_path[80];

/** UI selection cursor — selects which field is acted upon when the user
 *  presses OK.  Phase 9c-3 added Button + Counter; Phase 9d-3 inserts the
 *  CounterMode toggle between them so it is always reachable even when
 *  the manufacturer key did not resolve (Counter row is gated on
 *  s_has_counter; CounterMode is not, since it does not require cipher
 *  state to flip).  Values match the row order in draw_keeloq_fields(). */
enum {
    SIG_CURSOR_BUTTON       = 0,
    SIG_CURSOR_COUNTER_MODE = 1,
    SIG_CURSOR_COUNTER      = 2,
    SIG_CURSOR_COUNT
};
static uint8_t                 s_cursor;

/*============================================================================*/
/* Counter resolution (Phase 9c-2)                                             */
/*============================================================================*/

/* Resolve the device key for the currently-extracted KeeLoq-family
 * fields by looking up the manufacturer entry in the loaded mfkeys table
 * and deriving the device key via the recorded learning mode (Normal or
 * Simple — Secure is rejected without a seed).
 *
 * Returns true and writes the device key to *device_key_out iff:
 *   - signal.manufacture is non-empty,
 *   - the manufacturer entry exists in the mfkeys table,
 *   - the learning mode is Normal or Simple.
 *
 * Pure-logic glue — depends only on already-tested modules. */
static bool resolve_device_key(const flipper_subghz_signal_t *signal,
                               const subghz_keeloq_fields_t  *fields,
                               uint64_t                       *device_key_out)
{
    if (!signal || !fields || !device_key_out)
        return false;
    if (signal->manufacture[0] == '\0')
        return false;

    /* Lazy-load the manufacturer-key store so build-time embedded keys
     * (subghz_keeloq_mfkeys_builtin.c) are available even when this scene
     * is the first KeeLoq consumer this boot.  Mirrors m1_sub_ghz.c lazy-load. */
    if (keeloq_mfkeys_count() == 0)
        keeloq_mfkeys_load();

    KeeLoqMfrEntry mfr;
    if (!keeloq_mfkeys_find(signal->manufacture, &mfr))
        return false;

    switch (mfr.learn_type)
    {
        case KEELOQ_LEARN_NORMAL:
            *device_key_out = keeloq_learn_normal(fields->serial, mfr.key);
            return true;
        case KEELOQ_LEARN_SIMPLE:
            *device_key_out = keeloq_learn_simple(fields->serial, mfr.key);
            return true;
        case KEELOQ_LEARN_SECURE:
            /* Seed not recoverable from the .sub file — refuse rather than
             * silently fall back to Normal and produce a garbage counter. */
            return false;
        case KEELOQ_LEARN_MAGIC_XOR_TYPE1:
            *device_key_out = keeloq_learn_magic_xor_type1(fields->serial, mfr.key);
            return true;
        case KEELOQ_LEARN_FAAC_SLH:
            /* FAAC SLH requires a seed not recoverable from the .sub file. */
            return false;
        default:
            return false;
    }
}

/* Resolve the rolling counter for the currently-extracted KeeLoq-family
 * fields.  See @ref resolve_device_key for the resolution preconditions.
 * On success the caller can reuse @p device_key_out to re-encrypt an
 * updated counter without re-deriving from the mfkey table. */
static bool resolve_counter(const flipper_subghz_signal_t *signal,
                            const subghz_keeloq_fields_t  *fields,
                            uint16_t                       *counter_out,
                            uint64_t                       *device_key_out)
{
    if (!counter_out || !device_key_out)
        return false;

    if (!resolve_device_key(signal, fields, device_key_out))
        return false;

    *counter_out = subghz_signal_fields_keeloq_counter_decode(fields->enc_hop,
                                                              *device_key_out);
    return true;
}

/*============================================================================*/
/* Scene callbacks                                                             */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Returning from a pushed editor (SetButton, etc.) — clear the
     * cross-scene edit context flag so a subsequent fresh entry from the
     * Saved menu starts with a clean state. */
    app->signal_edit_active = false;

    s_loaded          = false;
    s_supported       = false;
    s_is_nice_flor_s  = false;
    s_is_came_atomo   = false;
    s_is_alutech_at_4n = false;
    s_has_counter     = false;
    s_counter         = 0U;
    s_device_key      = 0U;
    s_cursor          = SIG_CURSOR_BUTTON;
    memset(&s_signal, 0, sizeof(s_signal));
    memset(&s_fields, 0, sizeof(s_fields));
    memset(&s_nfs_fields, 0, sizeof(s_nfs_fields));
    memset(&s_atomo_fields, 0, sizeof(s_atomo_fields));
    memset(&s_alutech_fields, 0, sizeof(s_alutech_fields));
    s_saved_full_path[0] = '\0';

    if (app->saved_filepath[0] == '\0')
    {
        app->need_redraw = true;
        return;
    }

    char full_path[80];
    snprintf(full_path, sizeof(full_path), "0:%s", app->saved_filepath);

    if (!flipper_subghz_load(full_path, &s_signal))
    {
        app->need_redraw = true;
        return;
    }
    s_loaded = true;
    /* Stash the absolute path so the Phase 9b save-back path can rewrite
     * the exact file we loaded from without re-deriving from app state. */
    strncpy(s_saved_full_path, full_path, sizeof(s_saved_full_path) - 1);
    s_saved_full_path[sizeof(s_saved_full_path) - 1] = '\0';

    /* Only parsed (PACKET / Key) files have a 64-bit `key` to dissect.
     * RAW captures do not — the SavedMenu must already be gating this
     * scene on parsed files, but defend against being reached otherwise. */
    if (s_signal.type != FLIPPER_SUBGHZ_TYPE_PARSED)
    {
        app->need_redraw = true;
        return;
    }

    if (subghz_signal_fields_is_keeloq_family(s_signal.protocol))
    {
        s_supported = subghz_signal_fields_keeloq_extract(s_signal.protocol,
                                                          s_signal.key,
                                                          &s_fields);
        if (s_supported)
        {
            /* Phase 9c-2 — best-effort counter resolution.  Failure
             * (missing Manufacture line, unknown manufacturer, or Secure
             * learning) leaves s_has_counter == false and the scene draws
             * a "key?" placeholder; the rest of the fields still display
             * normally so the user knows the file loaded successfully. */
            s_has_counter = resolve_counter(&s_signal, &s_fields,
                                            &s_counter, &s_device_key);
        }
    }
    else if (subghz_signal_fields_is_nice_flor_s(s_signal.protocol))
    {
        /* P3 — Nice FloR-S cipher support.  Button and repeat are
         * plaintext in the 52-bit key.  Serial and counter require the
         * 32-byte rainbow table to decrypt.  When the builtin table is
         * available (baked in at build time via the
         * NICE_FLOR_S_RAINBOW_TABLE secret), decode immediately;
         * otherwise show "table?" placeholders. */
        s_supported = subghz_signal_fields_nice_flor_s_extract(
                          s_signal.key, &s_nfs_fields);
        s_is_nice_flor_s = s_supported;

        if (s_supported && nice_flor_s_table_builtin_available())
        {
            s_counter     = subghz_signal_fields_nice_flor_s_counter_decode(
                                s_nfs_fields.enc_payload,
                                nice_flor_s_table_builtin);
            s_has_counter = true;
        }
    }
    else if (subghz_signal_fields_is_came_atomo(s_signal.protocol))
    {
        /* P4 — CAME Atomo LFSR cipher support.  No external key material
         * needed — the cipher is self-contained.  All fields (serial,
         * counter, button) are decoded immediately. */
        s_supported = subghz_signal_fields_came_atomo_extract(
                          s_signal.key, &s_atomo_fields);
        s_is_came_atomo = s_supported;

        if (s_supported)
        {
            s_counter     = s_atomo_fields.counter;
            s_has_counter = true;
        }
    }
    else if (subghz_signal_fields_is_alutech_at_4n(s_signal.protocol))
    {
        /* P4 — Alutech AT-4N TEA-variant cipher support.  Requires the
         * 32-byte rainbow table (injected at build time via the
         * ALUTECH_AT_4N_RAINBOW_TABLE secret).  When the table is
         * available, decode serial/counter immediately; otherwise show
         * "table?" placeholders. */
        s_is_alutech_at_4n = true;
        if (alutech_at_4n_table_builtin_available())
        {
            s_supported = subghz_signal_fields_alutech_at_4n_extract(
                              s_signal.key,
                              alutech_at_4n_table_builtin,
                              &s_alutech_fields);
            if (s_supported)
            {
                s_counter     = s_alutech_fields.counter;
                s_has_counter = true;
            }
        }
        else
        {
            /* Table not available — mark as supported for UI display
             * but counter/serial will show "table?" placeholders. */
            s_supported = true;
        }
    }

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
        case SubGhzEventDown:
            /* Phase 9c-3 / 9d-3 — cycle the selection cursor between the
             * editable rows.  Button is always selectable; CounterMode is
             * always selectable once the file loaded and the protocol is
             * supported (toggling does not require the manufacturer key);
             * Counter is only reachable when the manufacturer key
             * resolved (otherwise editing has no meaning and the row
             * shows "key?"). */
            if (s_loaded && s_supported)
            {
                /* Step direction with wrap; skip the COUNTER row when
                 * the counter is unresolvable so it is never highlighted. */
                int8_t step = (event == SubGhzEventUp) ? -1 : +1;
                uint8_t next = s_cursor;
                for (uint8_t guard = 0; guard < SIG_CURSOR_COUNT; ++guard)
                {
                    next = (uint8_t)((next + (uint8_t)SIG_CURSOR_COUNT + step)
                                     % (uint8_t)SIG_CURSOR_COUNT);
                    if (next == SIG_CURSOR_COUNTER && !s_has_counter)
                        continue;
                    break;
                }
                if (next != s_cursor)
                {
                    s_cursor = next;
                    app->need_redraw = true;
                }
            }
            return true;

        case SubGhzEventOk:
            /* Phase 9c-3 / 9d-3 — dispatch to the editor for the
             * selected row.  Unsupported protocols, RAW files, and load
             * failures swallow the OK so the scene stays put. */
            if (s_loaded && s_supported)
            {
                if (s_cursor == SIG_CURSOR_COUNTER_MODE)
                {
                    /* CounterMode is a binary toggle — no sub-scene
                     * needed.  Flip in place, persist via the
                     * save_key_full helper (preserves Manufacture: and
                     * elides the field when Increment), and redraw.
                     * Do NOT set signal_edit_active — that flag is used
                     * by SetButton/SetCounter to know they were pushed
                     * for an edit; no scene is pushed here. */
                    (void)subghz_signal_settings_toggle_counter_mode();
                    app->need_redraw = true;
                }
                else
                {
                    app->signal_edit_active = true;
                    if (s_cursor == SIG_CURSOR_COUNTER && s_has_counter)
                        subghz_scene_push(app, SubGhzSceneSetCounter);
                    else
                        subghz_scene_push(app, SubGhzSceneSetButton);
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
    (void)app;
    /* Defensive: never leave a stale absolute path resident in BSS where
     * a later mis-routed save could overwrite the wrong file. */
    s_saved_full_path[0] = '\0';
    /* Clear the counter cache so a later cross-scene accessor caller can
     * never read a stale value from a previous file. */
    s_has_counter = false;
    s_counter     = 0U;
    s_device_key  = 0U;
}

/*============================================================================*/
/* Draw                                                                        */
/*============================================================================*/

static void draw_header(void)
{
    u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
    u8g2_DrawStr(&m1_u8g2, 2, 10, "Signal Settings");
    u8g2_DrawHLine(&m1_u8g2, 0, 12, M1_LCD_DISPLAY_WIDTH);
}

static void draw_placeholder(const char *line1, const char *line2)
{
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 4, 32, line1);
    if (line2)
        u8g2_DrawStr(&m1_u8g2, 4, 42, line2);
}

static void draw_keeloq_fields(void)
{
    char line[40];

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    snprintf(line, sizeof(line), "Proto: %s", s_signal.protocol);
    u8g2_DrawStr(&m1_u8g2, 2, 22, line);

    snprintf(line, sizeof(line), "Serial : 0x%07lX",
             (unsigned long)s_fields.serial);
    u8g2_DrawStr(&m1_u8g2, 2, 32, line);

    /* Phase 9c-3 / 9d-3 — selection cursor on the editable rows.  The
     * Button row is always editable; the CounterMode toggle is always
     * editable (cipher state not required to flip it); the Counter row
     * is only reachable when the manufacturer key resolved (otherwise
     * we draw "key?" and the cursor cannot move to it). */
    const char button_marker  = (s_cursor == SIG_CURSOR_BUTTON) ? '>' : ' ';
    const char cmode_marker   = (s_cursor == SIG_CURSOR_COUNTER_MODE) ? '>' : ' ';
    const bool counter_selectable = s_has_counter;
    const char counter_marker = (counter_selectable && s_cursor == SIG_CURSOR_COUNTER)
                                ? '>' : ' ';

    snprintf(line, sizeof(line), "%c Button : 0x%X", button_marker,
             (unsigned)(s_fields.button & 0x0F));
    u8g2_DrawStr(&m1_u8g2, 2, 42, line);

    /* CounterMode — Phase 9d-3.  Binary toggle persisted as the
     * optional `CounterMode:` field of the .sub file.  Increment is the
     * Flipper-compatible default and gets elided on save; Static causes
     * the replay path to skip the counter+1 step. */
    snprintf(line, sizeof(line), "%c CntMode: %s", cmode_marker,
             (s_signal.counter_mode == FLIPPER_SUBGHZ_COUNTER_MODE_STATIC)
             ? "Static" : "Increment");
    u8g2_DrawStr(&m1_u8g2, 2, 52, line);

    /* Counter — Phase 9c-2.  Resolved on scene_on_enter via the
     * manufacturer-key store + the learning mode recorded in the keystore
     * entry.  When resolution fails (no Manufacture line, manufacturer
     * absent from the keystore, or Secure learning without a seed) we
     * show a short "key?" hint so the user can tell the field is gated
     * on mfkey availability rather than the file being broken. */
    if (s_has_counter)
        snprintf(line, sizeof(line), "%c Counter: %u", counter_marker,
                 (unsigned)s_counter);
    else
        snprintf(line, sizeof(line), "  Counter: key?");
    u8g2_DrawStr(&m1_u8g2, 2, 62, line);
}

static void draw_nice_flor_s_fields(void)
{
    char line[40];

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    snprintf(line, sizeof(line), "Proto: %s", s_signal.protocol);
    u8g2_DrawStr(&m1_u8g2, 2, 22, line);

    /* Serial is inside the encrypted payload — requires the rainbow
     * table to decrypt.  When the builtin table is available, decrypt
     * and display the 28-bit serial; otherwise show "table?". */
    if (s_has_counter && nice_flor_s_table_builtin_available())
    {
        const uint64_t plain = nice_flor_s_decrypt(
            s_nfs_fields.enc_payload, nice_flor_s_table_builtin);
        const uint32_t serial = (uint32_t)((plain >> 16) & 0x0FFFFFFFUL);
        snprintf(line, sizeof(line), "Serial : %07lX",
                 (unsigned long)serial);
    }
    else
    {
        snprintf(line, sizeof(line), "Serial : table?");
    }
    u8g2_DrawStr(&m1_u8g2, 2, 32, line);

    const char button_marker  = (s_cursor == SIG_CURSOR_BUTTON) ? '>' : ' ';
    const char cmode_marker   = (s_cursor == SIG_CURSOR_COUNTER_MODE) ? '>' : ' ';

    snprintf(line, sizeof(line), "%c Button : 0x%X", button_marker,
             (unsigned)(s_nfs_fields.button & 0x0F));
    u8g2_DrawStr(&m1_u8g2, 2, 42, line);

    snprintf(line, sizeof(line), "%c CntMode: %s", cmode_marker,
             (s_signal.counter_mode == FLIPPER_SUBGHZ_COUNTER_MODE_STATIC)
             ? "Static" : "Increment");
    u8g2_DrawStr(&m1_u8g2, 2, 52, line);

    /* Counter requires the rainbow table to decrypt. */
    if (s_has_counter)
        snprintf(line, sizeof(line), "  Counter: %u",
                 (unsigned)s_counter);
    else
        snprintf(line, sizeof(line), "  Counter: table?");
    u8g2_DrawStr(&m1_u8g2, 2, 62, line);
}

static void draw_came_atomo_fields(void)
{
    char line[40];

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    snprintf(line, sizeof(line), "Proto: %s", s_signal.protocol);
    u8g2_DrawStr(&m1_u8g2, 2, 22, line);

    /* CAME Atomo serial and counter are always available — the LFSR
     * cipher requires no external key material. */
    snprintf(line, sizeof(line), "Serial : %08lX",
             (unsigned long)s_atomo_fields.serial);
    u8g2_DrawStr(&m1_u8g2, 2, 32, line);

    const char button_marker  = (s_cursor == SIG_CURSOR_BUTTON) ? '>' : ' ';
    const char cmode_marker   = (s_cursor == SIG_CURSOR_COUNTER_MODE) ? '>' : ' ';
    const bool counter_selectable = s_has_counter;
    const char counter_marker = (counter_selectable && s_cursor == SIG_CURSOR_COUNTER)
                                ? '>' : ' ';

    snprintf(line, sizeof(line), "%c Button : 0x%X", button_marker,
             (unsigned)(s_atomo_fields.button & 0x0F));
    u8g2_DrawStr(&m1_u8g2, 2, 42, line);

    snprintf(line, sizeof(line), "%c CntMode: %s", cmode_marker,
             (s_signal.counter_mode == FLIPPER_SUBGHZ_COUNTER_MODE_STATIC)
             ? "Static" : "Increment");
    u8g2_DrawStr(&m1_u8g2, 2, 52, line);

    snprintf(line, sizeof(line), "%c Counter: %u", counter_marker,
             (unsigned)s_counter);
    u8g2_DrawStr(&m1_u8g2, 2, 62, line);
}

static void draw_alutech_at_4n_fields(void)
{
    char line[40];

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    snprintf(line, sizeof(line), "Proto: %s", s_signal.protocol);
    u8g2_DrawStr(&m1_u8g2, 2, 22, line);

    /* Serial and counter require the rainbow table to decrypt. */
    if (s_has_counter && alutech_at_4n_table_builtin_available())
    {
        snprintf(line, sizeof(line), "Serial : %08lX",
                 (unsigned long)s_alutech_fields.serial);
    }
    else
    {
        snprintf(line, sizeof(line), "Serial : table?");
    }
    u8g2_DrawStr(&m1_u8g2, 2, 32, line);

    const char button_marker  = (s_cursor == SIG_CURSOR_BUTTON) ? '>' : ' ';
    const char cmode_marker   = (s_cursor == SIG_CURSOR_COUNTER_MODE) ? '>' : ' ';

    if (s_has_counter)
    {
        snprintf(line, sizeof(line), "%c Button : 0x%02X", button_marker,
                 (unsigned)s_alutech_fields.button);
    }
    else
    {
        snprintf(line, sizeof(line), "%c Button : table?", button_marker);
    }
    u8g2_DrawStr(&m1_u8g2, 2, 42, line);

    snprintf(line, sizeof(line), "%c CntMode: %s", cmode_marker,
             (s_signal.counter_mode == FLIPPER_SUBGHZ_COUNTER_MODE_STATIC)
             ? "Static" : "Increment");
    u8g2_DrawStr(&m1_u8g2, 2, 52, line);

    if (s_has_counter)
        snprintf(line, sizeof(line), "  Counter: %u",
                 (unsigned)s_counter);
    else
        snprintf(line, sizeof(line), "  Counter: table?");
    u8g2_DrawStr(&m1_u8g2, 2, 62, line);
}

static void draw(SubGhzApp *app)
{
    (void)app;
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    draw_header();

    if (!s_loaded)
    {
        draw_placeholder("File not loaded", NULL);
    }
    else if (s_signal.type != FLIPPER_SUBGHZ_TYPE_PARSED)
    {
        draw_placeholder("RAW file —", "no parsed fields");
    }
    else if (!s_supported)
    {
        char line[40];
        snprintf(line, sizeof(line), "Proto: %s", s_signal.protocol);
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 2, 22, line);

        /* Phase 9e-1: show the documented deferred-implementation reason
         * for CAME Atomo / Alutech AT-4N / Phoenix V2 so the user can
         * distinguish a deferred protocol from a wholly unsupported one.
         * Nice FloR-S has been promoted to SUPPORTED (P3). */
        const char *reason = NULL;
        const subghz_counter_edit_status_t st =
            subghz_signal_fields_counter_edit_status(s_signal.protocol, &reason);
        if (st == SUBGHZ_COUNTER_EDIT_DEFERRED && reason && reason[0] != '\0')
        {
            /* Split the reason string at the first ':' so the two
             * placeholder rows display the protocol tag on one line
             * and the blocker on the next, keeping each within the
             * 128-px display width. */
            const char *colon = strchr(reason, ':');
            if (colon)
            {
                char head[24];
                size_t hl = (size_t)(colon - reason);
                if (hl >= sizeof(head)) hl = sizeof(head) - 1U;
                memcpy(head, reason, hl);
                head[hl] = '\0';
                const char *tail = colon + 1;
                while (*tail == ' ') ++tail;
                draw_placeholder(head, tail);
            }
            else
            {
                draw_placeholder(reason, "edit deferred (9e)");
            }
        }
        else
        {
            draw_placeholder("Field display not", "supported");
        }
    }
    else
    {
        if (s_is_nice_flor_s)
            draw_nice_flor_s_fields();
        else if (s_is_came_atomo)
            draw_came_atomo_fields();
        else if (s_is_alutech_at_4n)
            draw_alutech_at_4n_fields();
        else
            draw_keeloq_fields();
    }
    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                               */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_signal_settings_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};

/*============================================================================*/
/* Phase 9b — Cross-scene API (consumed by SubGhzSceneSetButton)              */
/*============================================================================*/

uint8_t subghz_signal_settings_get_button(void)
{
    if (!s_loaded || !s_supported)
        return 0U;
    if (s_is_nice_flor_s)
        return (uint8_t)(s_nfs_fields.button & 0x0FU);
    if (s_is_came_atomo)
        return (uint8_t)(s_atomo_fields.button & 0x0FU);
    if (s_is_alutech_at_4n)
        return s_alutech_fields.button;
    return (uint8_t)(s_fields.button & 0x0FU);
}

bool subghz_signal_settings_apply_button(uint8_t new_button)
{
    if (!s_loaded || !s_supported)
        return false;
    if (s_signal.type != FLIPPER_SUBGHZ_TYPE_PARSED)
        return false;
    if (s_saved_full_path[0] == '\0')
        return false;

    uint64_t new_key = 0;

    if (s_is_nice_flor_s)
    {
        subghz_nice_flor_s_fields_t updated = s_nfs_fields;
        updated.button = (uint8_t)(new_button & 0x0FU);
        if (!subghz_signal_fields_nice_flor_s_assemble(&updated, &new_key))
            return false;
        bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                               s_signal.frequency,
                                               s_signal.preset,
                                               s_signal.protocol,
                                               s_signal.bit_count,
                                               new_key,
                                               s_signal.te,
                                               s_signal.manufacture,
                                               s_signal.counter_mode);
        if (!ok)
            return false;
        s_nfs_fields = updated;
        s_signal.key = new_key;
        return true;
    }
    else if (s_is_came_atomo)
    {
        subghz_came_atomo_fields_t updated = s_atomo_fields;
        updated.button = (uint8_t)(new_button & 0x0FU);
        if (!subghz_signal_fields_came_atomo_assemble(&updated, &new_key))
            return false;
        bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                               s_signal.frequency,
                                               s_signal.preset,
                                               s_signal.protocol,
                                               s_signal.bit_count,
                                               new_key,
                                               s_signal.te,
                                               s_signal.manufacture,
                                               s_signal.counter_mode);
        if (!ok)
            return false;
        s_atomo_fields = updated;
        s_signal.key = new_key;
        return true;
    }
    else if (s_is_alutech_at_4n)
    {
        if (!alutech_at_4n_table_builtin_available())
            return false;
        subghz_alutech_at_4n_fields_t updated = s_alutech_fields;
        updated.button = new_button;
        if (!subghz_signal_fields_alutech_at_4n_assemble(&updated,
                                                          alutech_at_4n_table_builtin,
                                                          &new_key))
            return false;
        bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                               s_signal.frequency,
                                               s_signal.preset,
                                               s_signal.protocol,
                                               s_signal.bit_count,
                                               new_key,
                                               s_signal.te,
                                               s_signal.manufacture,
                                               s_signal.counter_mode);
        if (!ok)
            return false;
        s_alutech_fields = updated;
        s_signal.key = new_key;
        return true;
    }

    /* KeeLoq family — substitute the button field and re-assemble. */
    subghz_keeloq_fields_t updated = s_fields;
    updated.button = (uint8_t)(new_button & 0x0FU);

    if (!subghz_signal_fields_keeloq_assemble(s_signal.protocol,
                                              &updated, &new_key))
        return false;

    /* The cached absolute load path is populated by scene_on_enter; if
     * empty (no file loaded) the save is rejected.  The Manufacture line
     * is propagated verbatim — if the source file had none, the field is
     * "" and the save helper falls back to plain key-save behaviour. */
    bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                            s_signal.frequency,
                                            s_signal.preset,
                                            s_signal.protocol,
                                            s_signal.bit_count,
                                            new_key,
                                            s_signal.te,
                                            s_signal.manufacture,
                                            s_signal.counter_mode);
    if (!ok)
        return false;

    /* Update the cache so a redraw without re-entering the scene shows
     * the new value immediately.  The scene's on_enter will overwrite
     * this anyway when control returns. */
    s_fields    = updated;
    s_signal.key = new_key;
    return true;
}

/*============================================================================*/
/* Phase 9c-2 — Cross-scene API (counter accessors for SubGhzSceneSetCounter)  */
/*============================================================================*/

bool subghz_signal_settings_has_counter(void)
{
    return s_loaded && s_supported && s_has_counter;
}

uint16_t subghz_signal_settings_get_counter(void)
{
    if (!s_loaded || !s_supported || !s_has_counter)
        return 0U;
    return s_counter;
}

/*============================================================================*/
/* Phase 9c-3 — Cross-scene API (counter writer for SubGhzSceneSetCounter)     */
/*============================================================================*/

bool subghz_signal_settings_apply_counter(uint16_t new_counter)
{
    if (!s_loaded || !s_supported)
        return false;
    if (s_signal.type != FLIPPER_SUBGHZ_TYPE_PARSED)
        return false;
    if (!s_has_counter)
        return false;
    if (s_saved_full_path[0] == '\0')
        return false;

    uint64_t new_key = 0;

    if (s_is_nice_flor_s)
    {
        if (!nice_flor_s_table_builtin_available())
            return false;
        uint64_t new_enc_payload = subghz_signal_fields_nice_flor_s_counter_encode(
            s_nfs_fields.enc_payload, new_counter, nice_flor_s_table_builtin);
        subghz_nice_flor_s_fields_t updated = s_nfs_fields;
        updated.enc_payload = new_enc_payload;
        if (!subghz_signal_fields_nice_flor_s_assemble(&updated, &new_key))
            return false;
        bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                               s_signal.frequency,
                                               s_signal.preset,
                                               s_signal.protocol,
                                               s_signal.bit_count,
                                               new_key,
                                               s_signal.te,
                                               s_signal.manufacture,
                                               s_signal.counter_mode);
        if (!ok)
            return false;
        s_nfs_fields  = updated;
        s_signal.key  = new_key;
        s_counter     = new_counter;
        return true;
    }
    else if (s_is_came_atomo)
    {
        new_key = subghz_signal_fields_came_atomo_counter_encode(s_signal.key,
                                                                  new_counter);
        subghz_came_atomo_fields_t updated;
        (void)subghz_signal_fields_came_atomo_extract(new_key, &updated);
        bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                               s_signal.frequency,
                                               s_signal.preset,
                                               s_signal.protocol,
                                               s_signal.bit_count,
                                               new_key,
                                               s_signal.te,
                                               s_signal.manufacture,
                                               s_signal.counter_mode);
        if (!ok)
            return false;
        s_atomo_fields = updated;
        s_signal.key   = new_key;
        s_counter      = new_counter;
        return true;
    }
    else if (s_is_alutech_at_4n)
    {
        if (!alutech_at_4n_table_builtin_available())
            return false;
        new_key = subghz_signal_fields_alutech_at_4n_counter_encode(
            s_signal.key, new_counter, alutech_at_4n_table_builtin);
        subghz_alutech_at_4n_fields_t updated;
        (void)subghz_signal_fields_alutech_at_4n_extract(
            new_key, alutech_at_4n_table_builtin, &updated);
        bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                               s_signal.frequency,
                                               s_signal.preset,
                                               s_signal.protocol,
                                               s_signal.bit_count,
                                               new_key,
                                               s_signal.te,
                                               s_signal.manufacture,
                                               s_signal.counter_mode);
        if (!ok)
            return false;
        s_alutech_fields = updated;
        s_signal.key     = new_key;
        s_counter        = new_counter;
        return true;
    }

    /* KeeLoq family — without a resolvable manufacturer key the encrypted
     * hop word cannot be re-encrypted; refuse the edit rather than
     * overwriting the file with corrupt cipher text. */
    /* Re-encrypt the counter via the host-tested pure-logic helper
     * (Phase 9c-1) — preserves the lower 16 plaintext bits (button,
     * VLOW, discriminant, overflow counter). */
    uint32_t new_enc_hop =
        subghz_signal_fields_keeloq_counter_encode(s_fields.enc_hop,
                                                   new_counter,
                                                   s_device_key);

    subghz_keeloq_fields_t updated = s_fields;
    updated.enc_hop = new_enc_hop;

    if (!subghz_signal_fields_keeloq_assemble(s_signal.protocol,
                                              &updated, &new_key))
        return false;

    bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                            s_signal.frequency,
                                            s_signal.preset,
                                            s_signal.protocol,
                                            s_signal.bit_count,
                                            new_key,
                                            s_signal.te,
                                            s_signal.manufacture,
                                            s_signal.counter_mode);
    if (!ok)
        return false;

    /* Refresh the in-memory caches so a redraw without re-entering the
     * scene reflects the new value immediately.  Re-entry's on_enter will
     * overwrite this from disk anyway. */
    s_fields      = updated;
    s_signal.key  = new_key;
    s_counter     = new_counter;
    return true;
}

/*============================================================================*/
/* Phase 9d-3 — CounterMode toggle + save                                      */
/*============================================================================*/

bool subghz_signal_settings_toggle_counter_mode(void)
{
    if (!s_loaded || !s_supported)
        return false;
    if (s_signal.type != FLIPPER_SUBGHZ_TYPE_PARSED)
        return false;
    if (s_saved_full_path[0] == '\0')
        return false;

    flipper_subghz_counter_mode_t new_mode =
        (s_signal.counter_mode == FLIPPER_SUBGHZ_COUNTER_MODE_STATIC)
        ? FLIPPER_SUBGHZ_COUNTER_MODE_INCREMENT
        : FLIPPER_SUBGHZ_COUNTER_MODE_STATIC;

    bool ok = flipper_subghz_save_key_full(s_saved_full_path,
                                            s_signal.frequency,
                                            s_signal.preset,
                                            s_signal.protocol,
                                            s_signal.bit_count,
                                            s_signal.key,
                                            s_signal.te,
                                            s_signal.manufacture,
                                            new_mode);
    if (!ok)
        return false;

    /* Refresh the in-memory cache so an immediate redraw shows the new
     * mode.  The scene's on_enter will reload from disk on re-entry. */
    s_signal.counter_mode = new_mode;
    return true;
}
