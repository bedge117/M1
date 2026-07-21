/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_remote.c
 * @brief  Sub-GHz Remote scene — multi-button RF remote control.
 *
 * The user loads a `.rem` manifest file that maps the five hardware buttons
 * (UP / DOWN / LEFT / RIGHT / OK) to individual `.sub` signal files.
 * Pressing a mapped button fires that signal immediately.
 *
 * Manifest format (plain text, one directive per line):
 *   # Any comment line is ignored
 *   up:    /SUBGHZ/gate_up.sub
 *   down:  /SUBGHZ/gate_down.sub
 *   left:  /SUBGHZ/ch_left.sub
 *   right: /SUBGHZ/ch_right.sub
 *   ok:    /SUBGHZ/arm.sub
 *
 * Any button not listed in the manifest is shown as "---" and does nothing.
 * Paths may use Flipper-style notation (/ext/subghz/…) — they are remapped
 * automatically via subghz_remap_flipper_path().
 *
 * UI layout (128×64 OLED):
 *   ┌────────────────────────────┐
 *   │  Remote: profile_name.rem  │  ← title row (12 px)
 *   │    ↑ Gate Open             │
 *   │◄ Ch─  │ OK: Arm  │ Ch+ ►  │
 *   │    ↓ Gate Close            │
 *   │   Press to send            │  ← hint line
 *   └────────────────────────────┘
 *
 * Navigation:
 *   BACK = exit Remote (returns to Sub-GHz menu)
 *   UP/DOWN/LEFT/RIGHT/OK = transmit corresponding signal
 *
 * The scene pushes itself from the Sub-GHz menu and is a standard scene.
 * TX is handed off to the async SubGhzSceneTransmitter (Phase 3b-2b-iv);
 * the radio is no longer driven from this scene's event handler. */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include "ff.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_api.h"
#include "m1_subghz_scene.h"
#include "m1_storage.h"
#include "subghz_playlist_parser.h"

/*============================================================================*/
/* Constants                                                                  */
/*============================================================================*/

/* Button indices — must match SUBGHZ_REMOTE_BUTTON_COUNT order */
#define REM_BTN_UP     0
#define REM_BTN_DOWN   1
#define REM_BTN_LEFT   2
#define REM_BTN_RIGHT  3
#define REM_BTN_OK     4

/* TX status indication is now handled entirely by the Transmitter scene
 * (Phase 3b-2b-iv migration); this scene no longer maintains its own
 * "Sent!" overlay state. */


/*============================================================================*/
/* Helpers — extern declarations                                              */
/*============================================================================*/

/* Note (Phase 3b-2b-iv): TX is no longer performed by the blocking
 * `sub_ghz_replay_flipper_file()` wrapper from this scene.  remote_fire()
 * now pushes SubGhzSceneTransmitter with tx_autostart=true so each
 * button press hands off to the async Transmitter state machine. */

/*============================================================================*/
/* Manifest parser                                                            */
/*============================================================================*/

static void remote_clear(SubGhzApp *app)
{
    for (uint8_t i = 0; i < SUBGHZ_REMOTE_BUTTON_COUNT; i++)
    {
        app->remote_label[i][0] = '\0';
        app->remote_files[i][0] = '\0';
    }
    app->remote_loaded = false;
}

/**
 * @brief Parse a .rem manifest file into app->remote_*.
 * @return true if at least one button was mapped.
 */
static bool remote_parse(SubGhzApp *app, const char *path)
{
    remote_clear(app);

    FIL fil;
    char fat_path[72];
    snprintf(fat_path, sizeof(fat_path), "0:%s", path);
    if (f_open(&fil, fat_path, FA_READ) != FR_OK)
        return false;

    char line[128];
    while (f_gets(line, sizeof(line), &fil))
    {
        /* Strip trailing CR/LF */
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
            line[--len] = '\0';

        if (len == 0 || line[0] == '#')
            continue;

        /* Parse "key: path" (label is derived from the file basename) */
        uint8_t btn_idx = 0xFF;
        const char *pfx = NULL;

        if      (strncasecmp(line, "up:",    3) == 0) { btn_idx = REM_BTN_UP;    pfx = line + 3; }
        else if (strncasecmp(line, "down:",  5) == 0) { btn_idx = REM_BTN_DOWN;  pfx = line + 5; }
        else if (strncasecmp(line, "left:",  5) == 0) { btn_idx = REM_BTN_LEFT;  pfx = line + 5; }
        else if (strncasecmp(line, "right:", 6) == 0) { btn_idx = REM_BTN_RIGHT; pfx = line + 6; }
        else if (strncasecmp(line, "ok:",    3) == 0) { btn_idx = REM_BTN_OK;    pfx = line + 3; }

        if (btn_idx == 0xFF || !pfx)
            continue;

        /* Skip leading whitespace */
        while (*pfx == ' ' || *pfx == '\t')
            pfx++;

        if (*pfx == '\0')
            continue;

        /* Remap Flipper paths */
        subghz_remap_flipper_path(pfx,
                                  app->remote_files[btn_idx],
                                  sizeof(app->remote_files[btn_idx]));

        /* Derive label: basename without extension */
        const char *base = strrchr(app->remote_files[btn_idx], '/');
        base = base ? base + 1 : app->remote_files[btn_idx];
        strncpy(app->remote_label[btn_idx], base,
                sizeof(app->remote_label[btn_idx]) - 1);
        app->remote_label[btn_idx][sizeof(app->remote_label[btn_idx]) - 1] = '\0';
        /* Strip .sub extension if present */
        char *ext = strrchr(app->remote_label[btn_idx], '.');
        if (ext)
            *ext = '\0';
    }

    f_close(&fil);

    /* Check at least one button is mapped */
    for (uint8_t i = 0; i < SUBGHZ_REMOTE_BUTTON_COUNT; i++)
        if (app->remote_files[i][0] != '\0')
            return true;

    return false;
}

/*============================================================================*/
/* Transmit helper                                                            */
/*============================================================================*/

static void remote_fire(SubGhzApp *app, uint8_t btn_idx)
{
    if (btn_idx >= SUBGHZ_REMOTE_BUTTON_COUNT || app->remote_files[btn_idx][0] == '\0')
        return;

    /* Build absolute (FatFS) path of the mapped .sub file. */
    char fat_path[72];
    snprintf(fat_path, sizeof(fat_path), "0:%s", app->remote_files[btn_idx]);

    /* Hand off to the async Transmitter scene with autostart so the
     * signal fires in a single button press (the user does not see a
     * READY/Press-OK confirmation screen).  On TX completion or BACK,
     * Transmitter pops and our scene_on_enter() runs again — the
     * Remote screen is redrawn and ready for the next press. */
    strncpy(app->tx_path, fat_path, sizeof(app->tx_path) - 1);
    app->tx_path[sizeof(app->tx_path) - 1] = '\0';
    app->tx_repeat_count = 1U;             /* static keys: 1 burst */
    app->tx_mode         = 0U;             /* SUBGHZ_TX_MODE_SINGLE */
    app->tx_autostart    = true;           /* one-press fire UX */
    /* Phase 4b — Remote is a one-shot fire UX (the button on the M1
     * remote view maps to a saved .sub file, and pressing it should
     * send the captured key as-is).  Button cycling is not part of
     * the Remote workflow, so clear tx_protocol_name and let the
     * Transmitter scene render the legacy single-button view. */
    app->tx_protocol_name[0] = '\0';

    subghz_scene_push(app, SubGhzSceneTransmitter);
}

static bool remote_load_manifest(SubGhzApp *app)
{
    S_M1_file_info *f_info = storage_browse("0:/SUBGHZ");
    if (f_info && f_info->file_is_selected)
    {
        snprintf(app->remote_path, sizeof(app->remote_path),
                 "/SUBGHZ/%s", f_info->file_name);
        return remote_parse(app, app->remote_path);
    }
    return false;
}

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

#define BTN_NAME(app, i) ((app)->remote_label[i][0] ? (app)->remote_label[i] : "---")

static void scene_draw(SubGhzApp *app)
{
    u8g2_ClearBuffer(&m1_u8g2);

    /* Title */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    const char *rname = strrchr(app->remote_path, '/');
    rname = rname ? rname + 1 : app->remote_path;
    char title_buf[32];
    snprintf(title_buf, sizeof(title_buf), "Remote: %.12s", rname);  /* "Remote: "=8 + 12 = 20 chars ≤ 120px */
    /* Strip .rem extension */
    char *dot = strrchr(title_buf, '.');
    if (dot)
        *dot = '\0';
    m1_draw_text(&m1_u8g2, 2, 9, 124, title_buf, TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);

    if (!app->remote_loaded)
    {
        /* No manifest — show hint */
        u8g2_DrawStr(&m1_u8g2, 4, 28, "No remote loaded.");
        u8g2_DrawStr(&m1_u8g2, 4, 38, "OK = browse .rem");
        u8g2_SendBuffer(&m1_u8g2);
        return;
    }

    /* Layout — 5 button cells */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    /* UP button (top-center) */
    u8g2_DrawStr(&m1_u8g2, 32, 21, "\x18");  /* ↑ glyph */
    u8g2_DrawStr(&m1_u8g2, 42, 21, BTN_NAME(app, REM_BTN_UP));

    /* LEFT button */
    u8g2_DrawStr(&m1_u8g2, 0, 34, "\x1b");  /* ← glyph */
    u8g2_DrawStr(&m1_u8g2, 8, 34, BTN_NAME(app, REM_BTN_LEFT));

    /* OK button (center) */
    u8g2_DrawStr(&m1_u8g2, 52, 34, "OK:");
    u8g2_DrawStr(&m1_u8g2, 68, 34, BTN_NAME(app, REM_BTN_OK));

    /* RIGHT button */
    u8g2_DrawStr(&m1_u8g2, 96, 34, BTN_NAME(app, REM_BTN_RIGHT));
    u8g2_DrawStr(&m1_u8g2, 122, 34, "\x1a");  /* → glyph */

    /* DOWN button (bottom-center) */
    u8g2_DrawStr(&m1_u8g2, 32, 47, "\x19");  /* ↓ glyph */
    u8g2_DrawStr(&m1_u8g2, 42, 47, BTN_NAME(app, REM_BTN_DOWN));

    /* Hint — TX status itself is shown by the Transmitter scene when
     * a button is pressed (Phase 3b-2b-iv migration), so this scene
     * just shows what to do. */
    u8g2_DrawStr(&m1_u8g2, 4, 62, "Press to send");

    u8g2_SendBuffer(&m1_u8g2);
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Clear the autostart hint defensively — a sibling caller (e.g.
     * a previous Saved push) may have set it, and we don't want it
     * to leak into the Transmitter push that follows our remote_fire().
     * remote_fire() sets it explicitly each time. */
    app->tx_autostart = false;

    /* If no manifest loaded yet, auto-launch file browser */
    if (!app->remote_loaded)
    {
        app->remote_loaded = remote_load_manifest(app);
        if (!app->remote_loaded)
        {
            /* User cancelled or file invalid — pop back to menu */
            subghz_scene_pop(app);
            return;
        }
    }
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        case SubGhzEventBack:
            remote_clear(app);
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            remote_fire(app, REM_BTN_UP);
            return true;

        case SubGhzEventDown:
            remote_fire(app, REM_BTN_DOWN);
            return true;

        case SubGhzEventLeft:
            remote_fire(app, REM_BTN_LEFT);
            return true;

        case SubGhzEventRight:
            remote_fire(app, REM_BTN_RIGHT);
            return true;

        case SubGhzEventOk:
            if (!app->remote_loaded)
            {
                /* Browse for a .rem file */
                app->remote_loaded = remote_load_manifest(app);
            }
            else
            {
                remote_fire(app, REM_BTN_OK);
            }
            app->need_redraw = true;
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
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_remote_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = scene_draw,
};
