/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_saved_menu.c
 * @brief  Sub-GHz Saved-file action menu scene (Phase 5).
 *
 * Pushed by the Saved file-browser scene once a file is selected.
 *
 * RAW files are transparently redirected to the Read Raw scene in Loaded
 * state (Momentum parity): the user sees the graphical waveform viewer
 * with Send / New / More buttons instead of a text action menu.  This
 * scene replaces itself in the scene stack with SubGhzSceneReadRaw via
 * subghz_scene_replace(), so Back in Read Raw returns to the file browser.
 *
 * Parsed files show the action menu (Emulate / Info / Rename / Delete).
 *   Parsed files: Emulate, Info, Rename, Delete
 *
 * "Emulate" for parsed files pushes the Transmitter scene which drives
 * the async-TX state machine.
 *
 * "Delete" pushes the dedicated Delete confirmation scene.
 *
 * Phase 5 split: action menu / info / decode previously lived inside
 * `m1_subghz_scene_saved.c`; that scene is now a pure file browser.
 *
 * Phase 7c-3: action-menu rendering and scroll/selection math come from
 * the reusable `subghz_submenu_model` + `m1_submenu_draw` widget so the
 * menu honours `Settings → LCD & Notifications → Text Size` consistently
 * with the Sub-GHz top-level Menu and the Read-Raw MoreRAW menu.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_storage.h"
#include "m1_scene.h"
#include "m1_submenu.h"
#include "m1_subghz_scene.h"
#include "flipper_subghz.h"
#include "m1_sub_ghz.h"
#include "subghz_protocol_registry.h"
#include "m1_virtual_kb.h"
#include "subghz_submenu_model.h"
#include "subghz_signal_fields.h"
#include "subghz_signal_format.h"

/*============================================================================*/
/* Action menu — unified action IDs                                           */
/*============================================================================*/

enum {
    SAVED_ACTION_EMULATE = 0,
    SAVED_ACTION_INFO,
    SAVED_ACTION_RENAME,
    SAVED_ACTION_DELETE,
    SAVED_ACTION_SETTINGS,        /**< Phase 9a-2 — per-file SignalSettings */
};

static const char *const parsed_action_labels[] = { "Emulate", "Info", "Rename", "Delete" };
/* Parsed-file label set when the protocol supports the SignalSettings
 * scene (Phase 9a-2: KeeLoq family only; broader gating in Phase 9e). */
static const char *const parsed_settings_labels[] = { "Emulate", "Info", "Settings", "Rename", "Delete" };

#define PARSED_ACTION_COUNT          4
#define PARSED_SETTINGS_ACTION_COUNT 5

/* Phase 7c-3: action menu uses the reusable pure-logic submenu model.
 * Selection is reset on every scene_on_enter to match prior behaviour. */
static subghz_submenu_model_t s_action_model;
static uint8_t action_count;
static const char *const *active_labels;

static bool in_info_screen;
static bool is_raw_file;
/* Phase 9a-2 — true when the loaded parsed file's protocol is supported
 * by the SignalSettings scene; gates the "Settings" entry in the menu. */
static bool has_settings_entry;

/* Cached .sub file metadata (loaded eagerly on scene entry) */
static flipper_subghz_signal_t saved_signal;

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

/**
 * @brief  Map selected action index to unified action ID.
 *         When the SignalSettings entry is present (Phase 9a-2), it
 *         occupies index 2 between Info and Rename.
 */
static uint8_t map_action(uint8_t sel)
{
    if (has_settings_entry)
    {
        /* Parsed-with-Settings layout: 0→Emulate, 1→Info, 2→Settings,
         * 3→Rename, 4→Delete. */
        if (sel == 2) return SAVED_ACTION_SETTINGS;
        if (sel <= 1) return sel;   /* 0→Emulate, 1→Info */
        return sel - 1;             /* 3→Rename(2), 4→Delete(3) */
    }
    return sel;                /* 0→Emulate, 1→Info, 2→Rename, 3→Delete */
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

/**
 * @brief  Load the file metadata from app->saved_filepath into saved_signal.
 *
 * Called from scene_on_enter both on first push and on resume-from-child
 * (after Rename / Delete-cancel / etc.) so the action menu always reflects
 * the up-to-date file state.
 *
 * @retval true   File loaded successfully.
 * @retval false  Load failed — caller should pop the scene.
 */
static bool load_signal(const SubGhzApp *app)
{
    if (app->saved_filepath[0] == '\0')
        return false;

    char full_path[80];
    snprintf(full_path, sizeof(full_path), "0:%s", app->saved_filepath);

    memset(&saved_signal, 0, sizeof(saved_signal));
    if (!flipper_subghz_load(full_path, &saved_signal))
        return false;

    is_raw_file = (saved_signal.type == FLIPPER_SUBGHZ_TYPE_RAW
                   && saved_signal.raw_count > 0);

    /* Phase 9a-2/9e-1: gate the Settings entry on parsed files whose
     * protocol either has a host-tested field extractor (KeeLoq family —
     * full read-only display + Button/Counter/CounterMode editing) or
     * has a documented deferred-implementation status (Nice FloR-S /
     * CAME Atomo / Alutech AT-4N / Phoenix V2 — the scene shows a
     * clear "edit not yet supported" placeholder with the specific
     * blocker for that protocol).  All other protocols hide the entry. */
    {
        const subghz_counter_edit_status_t edit_status =
            subghz_signal_fields_counter_edit_status(saved_signal.protocol, NULL);
        has_settings_entry = (!is_raw_file)
                           && (saved_signal.type == FLIPPER_SUBGHZ_TYPE_PARSED)
                           && (edit_status == SUBGHZ_COUNTER_EDIT_SUPPORTED
                               || edit_status == SUBGHZ_COUNTER_EDIT_DEFERRED);
    }

    if (has_settings_entry)
    {
        action_count  = PARSED_SETTINGS_ACTION_COUNT;
        active_labels = parsed_settings_labels;
    }
    else
    {
        action_count  = PARSED_ACTION_COUNT;
        active_labels = parsed_action_labels;
    }
    subghz_submenu_model_init(&s_action_model,
                              action_count,
                              M1_MENU_VIS(action_count));
    return true;
}

static void scene_on_enter(SubGhzApp *app)
{
    in_info_screen = false;

    if (!load_signal(app))
    {
        /* File missing / unreadable (e.g. deleted) — return to file browser. */
        subghz_scene_pop(app);
        return;
    }

    /* Momentum parity: RAW files open directly in the Read Raw waveform
     * viewer (Loaded state) rather than a text action menu.  Replace
     * ourselves in the scene stack with ReadRaw so Back returns to the
     * file browser.  Decode / Rename / Delete remain accessible via the
     * More button inside Read Raw. */
    if (is_raw_file)
    {
        strncpy(app->raw_load_path, app->saved_filepath, sizeof(app->raw_load_path) - 1);
        app->raw_load_path[sizeof(app->raw_load_path) - 1] = '\0';
        app->raw_load_is_native = saved_signal.is_m1_native;
        app->raw_load_freq_hz   = saved_signal.frequency;
        {
            uint8_t mod = flipper_subghz_preset_to_modulation(saved_signal.preset);
            app->raw_load_mod = (mod == MODULATION_UNKNOWN) ? MODULATION_OOK : mod;
        }
        subghz_scene_replace(app, SubGhzSceneReadRaw);
        return;
    }

    app->need_redraw = true;
}

static bool handle_action(SubGhzApp *app, uint8_t action)
{
    switch (action)
    {
        case SAVED_ACTION_EMULATE:
        {
            /* Parsed (static-key) files: push the Transmitter scene which
             * drives the async-TX state machine on top of our scene. */
            strncpy(app->tx_path, app->saved_filepath, sizeof(app->tx_path) - 1);
            app->tx_path[sizeof(app->tx_path) - 1] = '\0';
            app->tx_repeat_count = 5U;             /* static keys: 5 bursts for reliable TX */
            app->tx_mode         = 0U;             /* SUBGHZ_TX_MODE_SINGLE */
            /* Phase 4b — propagate the parsed protocol name so the
             * Transmitter scene can query button-cycling capability. */
            strncpy(app->tx_protocol_name, saved_signal.protocol,
                    sizeof(app->tx_protocol_name) - 1);
            app->tx_protocol_name[sizeof(app->tx_protocol_name) - 1] = '\0';
            subghz_scene_push(app, SubGhzSceneTransmitter);
            return true;
        }
        case SAVED_ACTION_INFO:
        {
            in_info_screen = true;
            app->need_redraw = true;
            return true;
        }
        case SAVED_ACTION_RENAME:
        {
            char new_name[32];
            /* Strip extension for rename prompt */
            char base[32];
            strncpy(base, app->saved_filename, sizeof(base) - 1);
            base[sizeof(base) - 1] = '\0';
            char *dot = strrchr(base, '.');
            if (dot) *dot = '\0';

            if (m1_vkb_get_filename("Rename to:", base, new_name))
            {
                char old_path[80], new_path[80];
                snprintf(old_path, sizeof(old_path), "0:%s", app->saved_filepath);
                /* Determine extension */
                const char *ext = strrchr(app->saved_filename, '.');
                if (!ext) ext = ".sub";
                snprintf(new_path, sizeof(new_path), "0:/SUBGHZ/%s%s", new_name, ext);
                f_rename(old_path, new_path);
            }
            /* The renamed (or unchanged) file is no longer guaranteed to
             * exist at app->saved_filepath — return to the file browser. */
            subghz_scene_pop(app);
            return true;
        }
        case SAVED_ACTION_DELETE:
        {
            /* Hand off to the dedicated confirmation scene.  On confirm,
             * the Delete scene unlinks the file and pops back through us
             * to the Saved file browser; on cancel, it pops back to us. */
            subghz_scene_push(app, SubGhzSceneDelete);
            return true;
        }
        case SAVED_ACTION_SETTINGS:
        {
            /* Phase 9a-2 — push the read-only SignalSettings scene.  The
             * scene reloads from app->saved_filepath and dissects the
             * 64-bit key into protocol-specific fields. */
            subghz_scene_push(app, SubGhzSceneSignalSettings);
            return true;
        }
    }
    return false;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* --- Info screen --- */
    if (in_info_screen)
    {
        if (event == SubGhzEventBack)
        {
            in_info_screen = false;
            app->need_redraw = true;
            return true;
        }
        return false;
    }

    /* --- Action menu --- */

    /* Re-sync visible_count in case the user changed the text-size
     * preference while a child scene was on top. */
    subghz_submenu_model_set_visible_count(&s_action_model,
                                           M1_MENU_VIS(action_count));

    switch (event)
    {
        case SubGhzEventBack:
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            subghz_submenu_model_up(&s_action_model);
            app->need_redraw = true;
            return true;

        case SubGhzEventDown:
            subghz_submenu_model_down(&s_action_model);
            app->need_redraw = true;
            return true;

        case SubGhzEventOk:
            handle_action(app, map_action(s_action_model.selected));
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

/*============================================================================*/
/* Draw helpers                                                               */
/*============================================================================*/

static void draw_info_screen(void)
{
    char line[48];

    u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
    u8g2_DrawStr(&m1_u8g2, 2, 10, "Signal Info");
    u8g2_DrawHLine(&m1_u8g2, 0, 12, M1_LCD_DISPLAY_WIDTH);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    if (saved_signal.type == FLIPPER_SUBGHZ_TYPE_PARSED)
    {
        /* Phase 11-1: consult the protocol's polymorphic get_string vtable
         * slot when present.  Currently installed only on the KeeLoq family
         * (KeeLoq / Star Line / Jarolift); all other parsed protocols fall
         * through to the generic Proto / Key / Bits / TE layout below. */
        int16_t proto_idx = subghz_protocol_find_by_name(saved_signal.protocol);
        const SubGhzProtocolDef *proto =
            (proto_idx >= 0) ? subghz_protocol_get((uint16_t)proto_idx) : NULL;

        if (proto && proto->get_string)
        {
            char info[96];
            SubGhzSignalView view = {
                .protocol  = saved_signal.protocol,
                .key       = saved_signal.key,
                .bit_count = saved_signal.bit_count,
                .te        = saved_signal.te,
            };
            proto->get_string(&view, info, sizeof(info));

            /* Render up to 3 newline-separated rows from the formatter
             * output at y=22/30/38, leaving y=46 for Freq and y=54 for Mod. */
            int y = 22;
            const char *p = info;
            for (int row = 0; row < 3 && *p; row++)
            {
                size_t n = 0;
                while (p[n] && p[n] != '\n' && n < sizeof(line) - 1)
                {
                    line[n] = p[n];
                    n++;
                }
                line[n] = '\0';
                if (n > 0)
                    u8g2_DrawStr(&m1_u8g2, 2, y, line);
                y += 8;
                if (p[n] == '\n') p += n + 1;
                else              p += n;
            }
        }
        else
        {
            snprintf(line, sizeof(line), "Proto: %s", saved_signal.protocol);
            u8g2_DrawStr(&m1_u8g2, 2, 22, line);

            snprintf(line, sizeof(line), "Key: 0x%lX", (uint32_t)saved_signal.key);
            u8g2_DrawStr(&m1_u8g2, 2, 30, line);

            snprintf(line, sizeof(line), "Bits: %lu  TE: %lu us",
                     (unsigned long)saved_signal.bit_count,
                     (unsigned long)saved_signal.te);
            u8g2_DrawStr(&m1_u8g2, 2, 38, line);
        }
    }
    else
    {
        u8g2_DrawStr(&m1_u8g2, 2, 22, "Type: RAW");

        snprintf(line, sizeof(line), "Samples: %u", saved_signal.raw_count);
        u8g2_DrawStr(&m1_u8g2, 2, 30, line);
    }

    /* Use integer arithmetic — embedded printf (nano.specs) has no %f support */
    snprintf(line, sizeof(line), "Freq: %lu.%02lu MHz",
             (unsigned long)(saved_signal.frequency / 1000000UL),
             (unsigned long)((saved_signal.frequency % 1000000UL) / 10000UL));
    u8g2_DrawStr(&m1_u8g2, 2, 46, line);

    if (saved_signal.preset[0])
    {
        snprintf(line, sizeof(line), "Mod: %s", saved_signal.preset);
        u8g2_DrawStr(&m1_u8g2, 2, 54, line);
    }
}

static void draw_action_menu(const SubGhzApp *app)
{
    /* Phase 7c-3: render via the reusable submenu widget.  The filename
     * (truncated to a manageable length) is used as the title; the model
     * derives row height, scroll math, and scrollbar geometry from the
     * user's text-size preference. */
    char title[22];
    strncpy(title, app->saved_filename, sizeof(title) - 1);
    title[sizeof(title) - 1] = '\0';

    /* Re-sync visible_count for the current text-size setting before
     * drawing — matches the Phase 7c-1 / 7c-2 pattern. */
    subghz_submenu_model_set_visible_count(&s_action_model,
                                           M1_MENU_VIS(action_count));
    m1_submenu_draw(&s_action_model, title, active_labels);
}

static void draw(SubGhzApp *app)
{
    if (in_info_screen)
    {
        m1_u8g2_firstpage();
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        draw_info_screen();
        m1_u8g2_nextpage();
    }
    else
    {
        /* m1_submenu_draw owns its own firstpage/nextpage. */
        draw_action_menu(app);
    }
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_saved_menu_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
