/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_playlist.c
 * @brief  Sub-GHz Playlist Scene — sequential .sub file transmitter.
 *
 * Reads a plain-text playlist (.txt) from /SUBGHZ/playlist/ containing
 * one "sub: <path>" entry per line, then transmits each referenced .sub
 * file in order via the async-driven SubGhzSceneTransmitter (one push
 * per file).  When the Transmitter pops back to us, scene_on_enter
 * detects the resume and advances to the next file or finalises the
 * pass — no synchronous TX loop, no main-task blocking.
 *
 * Playlist format (one entry per line):
 *   sub: /SUBGHZ/Tesla_charge_AM270.sub
 *   sub: /SUBGHZ/Tesla_charge_AM650.sub
 *
 * Lines not starting with "sub:" are ignored (comments, blank lines).
 * Flipper-style paths (/ext/subghz/...) are automatically remapped to
 * /SUBGHZ/... for compatibility with UberGuidoZ playlist collections.
 *
 * Controls:
 *   OK    = start playback (pushes Transmitter scene per file)
 *   L/R   = adjust repeat count (1–9, then ∞) (idle only)
 *   BACK  = re-open file browser (idle only — during playback BACK is
 *           handled by the child Transmitter scene which aborts the
 *           current file and triggers playlist stop on resume)
 *   U/D   = scroll file list (idle only)
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "ff.h"
#include "app_freertos.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_storage.h"
#include "m1_scene.h"
#include "m1_subghz_scene.h"
#include "m1_subghz_button_bar.h"
#include "subghz_playlist_parser.h"

/*============================================================================*/
/* Constants                                                                  */
/*============================================================================*/

#define PLAYLIST_DIR        "/SUBGHZ/playlist"
#define PLAYLIST_MAX_FILES  16
#define PLAYLIST_PATH_MAX   64
#define PLAYLIST_LINE_MAX   256

/** Playlist list area: progress bar sits at y=46, so list spans y=12..45 = 34px */
#define PLAYLIST_LIST_AREA_H   34

/** Maximum visible items in the file list (font-aware) */
#define PLAYLIST_LIST_VISIBLE  ((uint8_t)(PLAYLIST_LIST_AREA_H / m1_menu_item_h()))

/*============================================================================*/
/* Local state                                                                */
/*============================================================================*/

static uint8_t list_scroll = 0;     /**< Top visible item in file list */
static bool    browse_done = false; /**< File selected, in playback view */

/*============================================================================*/
/* Playlist parser                                                            */
/*============================================================================*/

/* remap_flipper_path() has been extracted to subghz_playlist_parser.c/h
 * as subghz_remap_flipper_path().  Included via subghz_playlist_parser.h. */

/**
 * @brief  Parse a playlist .txt file and populate app->playlist_files[].
 *
 * @param  app   Application context (playlist fields updated)
 * @param  path  Full FatFS path to the playlist .txt (e.g. "/SUBGHZ/playlist/Tesla.txt")
 * @retval true  At least one entry was loaded
 * @retval false Parse failed or file empty
 */
static bool playlist_parse(SubGhzApp *app, const char *path)
{
    FIL fil;
    char line[PLAYLIST_LINE_MAX];

    app->playlist_count = 0;

    /* Clear any leftover delay values from a previously loaded playlist */
    for (uint8_t i = 0; i < PLAYLIST_MAX_FILES; i++)
        app->playlist_delays[i] = 0;

    /* Create playlist directory on demand */
    f_mkdir("/SUBGHZ");
    f_mkdir(PLAYLIST_DIR);

    if (f_open(&fil, path, FA_READ) != FR_OK)
        return false;

    while (f_gets(line, sizeof(line), &fil))
    {
        if (app->playlist_count >= PLAYLIST_MAX_FILES)
            break;

        /* Strip trailing CR/LF */
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
            line[--len] = '\0';

        /* Skip blank lines */
        if (len == 0)
            continue;

        /* Check for delay directive: "# delay: <ms>"
         * The delay applies to the NEXT entry added after this line.
         * If multiple delay lines appear consecutively, the last one wins. */
        if (line[0] == '#')
        {
            uint16_t dms = 0;
            if (subghz_playlist_parse_delay(line, &dms))
            {
                /* Record as the delay for the next sub: entry */
                if (app->playlist_count < PLAYLIST_MAX_FILES)
                    app->playlist_delays[app->playlist_count] = dms;
            }
            continue;
        }

        /* Look for "sub:" prefix */
        if (strncmp(line, "sub:", 4) != 0)
            continue;

        /* Extract path — skip "sub:" and leading whitespace */
        const char *p = line + 4;
        while (*p == ' ' || *p == '\t')
            p++;

        if (*p == '\0')
            continue;

        /* Remap Flipper paths and store */
        subghz_remap_flipper_path(p,
                           app->playlist_files[app->playlist_count],
                           PLAYLIST_PATH_MAX);

        app->playlist_count++;
    }

    f_close(&fil);
    return (app->playlist_count > 0);
}

/*============================================================================*/
/* Playback                                                                   */
/*============================================================================*/

/**
 * @brief  Set up tx_* fields and push the Transmitter scene to send the
 *         file at app->playlist_files[app->playlist_current].
 *
 * The Transmitter scene drives the async TX state machine
 * (prepare_flipper + start_async + continue_async + abort) on top of
 * our scene.  When TX completes (or the user aborts via BACK), the
 * Transmitter pops back to us and our scene_on_enter detects the
 * pop-back via app->resume_from_child and decides whether to advance
 * to the next file (natural completion) or stop (user abort).
 *
 * Note: this uses the CONVERT path uniformly for all file types
 * (Flipper RAW, M1 native NOISE, Flipper Key, M1 native PACKET).  The
 * Transmitter scene's prepare_flipper handles all four — at the small
 * cost of a temp-file conversion per NOISE file.  This is acceptable
 * for playlist's serial playback.
 */
static void playlist_push_transmitter(SubGhzApp *app)
{
    if (app->playlist_current >= app->playlist_count)
        return;

    const char *path = app->playlist_files[app->playlist_current];
    strncpy(app->tx_path, path, sizeof(app->tx_path) - 1);
    app->tx_path[sizeof(app->tx_path) - 1] = '\0';
    app->tx_repeat_count = 1U;            /* one burst per playlist item */
    app->tx_mode         = 0U;            /* SUBGHZ_TX_MODE_SINGLE */
    /* Phase 4b — Playlist runs items back-to-back automatically; button
     * cycling is not part of the workflow.  Clear tx_protocol_name so
     * the Transmitter scene treats every item as a single-button TX. */
    app->tx_protocol_name[0] = '\0';

    /* Flag so the Transmitter pop returns to our scene_on_enter and we
     * recognise it as a resume rather than a fresh entry. */
    app->resume_from_child = true;

    subghz_scene_push(app, SubGhzSceneTransmitter);
}

/*============================================================================*/
/* Helper: extract filename from path                                         */
/*============================================================================*/

static const char *basename_from_path(const char *path)
{
    const char *slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

/**
 * @brief  Open the playlist directory browser, parse the selected file.
 *
 * @retval true   A valid playlist was loaded.
 * @retval false  User cancelled or parse failed (caller should pop).
 */
static bool open_playlist_browser(SubGhzApp *app)
{
    /* Ensure playlist directory exists */
    f_mkdir("/SUBGHZ");
    f_mkdir(PLAYLIST_DIR);

    S_M1_file_info *f_info = storage_browse("0:" PLAYLIST_DIR);
    if (f_info && f_info->file_is_selected)
    {
        snprintf(app->playlist_path, sizeof(app->playlist_path),
                 PLAYLIST_DIR "/%s", f_info->file_name);

        if (playlist_parse(app, app->playlist_path))
        {
            browse_done = true;
            list_scroll = 0;
            return true;
        }
    }
    return false;
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Detect resume-from-child: set by us before pushing the Transmitter
     * scene.  When the Transmitter pops back, scene_on_enter is invoked
     * by the scene manager and this branch advances the playlist instead
     * of resetting state and re-opening the file browser.
     *
     * tx_completed_naturally is set by the Transmitter scene:
     *   true  → TX ran to completion: advance to next file / next pass.
     *   false → user aborted via BACK or dismissed an error: stop. */
    if (app->resume_from_child)
    {
        app->resume_from_child = false;

        if (!app->tx_completed_naturally)
        {
            /* User abort — stop playback and remain in READY state. */
            app->playlist_running = false;
            app->playlist_current = 0;
            app->need_redraw = true;
            return;
        }

        /* Natural completion of the just-played file.  Advance the
         * playlist cursor and decide what to do next. */
        app->playlist_current++;

        if (app->playlist_current < app->playlist_count)
        {
            /* More files in this pass — push next Transmitter. */
            playlist_push_transmitter(app);
            return;
        }

        /* End of pass — bump repeat counter and decide. */
        app->playlist_repeat_done++;
        if (app->playlist_repeat_total > 0 &&
            app->playlist_repeat_done >= app->playlist_repeat_total)
        {
            /* All requested passes completed — stop. */
            app->playlist_running = false;
            app->playlist_current = 0;
            app->need_redraw = true;
            return;
        }

        /* Start another pass from the top. */
        app->playlist_current = 0;
        playlist_push_transmitter(app);
        return;
    }

    /* Fresh entry — reset state and open the file browser. */
    browse_done = false;
    list_scroll = 0;
    app->playlist_count = 0;
    app->playlist_current = 0;
    app->playlist_running = false;
    app->playlist_repeat_total = 1;
    app->playlist_repeat_done = 0;
    for (uint8_t i = 0; i < PLAYLIST_MAX_FILES; i++)
        app->playlist_delays[i] = 0;

    /* Open file browser immediately — no intermediate prompt screen */
    if (!open_playlist_browser(app))
    {
        subghz_scene_pop(app);
        return;
    }
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    /* Playback control mode (browse_done is always true when events arrive) */
    switch (event)
    {
        case SubGhzEventBack:
            app->playlist_running = false;
            /* Re-open file browser; pop if user cancels */
            if (!open_playlist_browser(app))
            {
                subghz_scene_pop(app);
                return true;
            }
            /* Reset playback state for the newly selected playlist */
            app->playlist_current = 0;
            app->playlist_repeat_done = 0;
            app->need_redraw = true;
            return true;

        case SubGhzEventOk:
        {
            if (!app->playlist_running)
            {
                /* Start playback — initialise counters and push the
                 * Transmitter scene for the first file.  Subsequent
                 * files are pushed from scene_on_enter on pop-back. */
                if (app->playlist_count == 0)
                    return true;

                app->playlist_running     = true;
                app->playlist_repeat_done = 0;
                app->playlist_current     = 0;
                app->need_redraw          = true;
                playlist_push_transmitter(app);
            }
            /* If already running, OK is ignored — playback control during
             * TX is handled by the Transmitter scene's BACK button. */
            return true;
        }

        case SubGhzEventLeft:
            /* Decrease repeat count (min 1) */
            if (!app->playlist_running)
            {
                if (app->playlist_repeat_total == 0)
                    app->playlist_repeat_total = 9;
                else if (app->playlist_repeat_total > 1)
                    app->playlist_repeat_total--;
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventRight:
            /* Increase repeat count (max 9, then infinite=0) */
            if (!app->playlist_running)
            {
                if (app->playlist_repeat_total > 0 &&
                    app->playlist_repeat_total < 9)
                    app->playlist_repeat_total++;
                else if (app->playlist_repeat_total >= 9)
                    app->playlist_repeat_total = 0; /* infinite */
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventUp:
            if (!app->playlist_running && list_scroll > 0)
            {
                list_scroll--;
                app->need_redraw = true;
            }
            return true;

        case SubGhzEventDown:
            if (!app->playlist_running &&
                list_scroll + PLAYLIST_LIST_VISIBLE < app->playlist_count)
            {
                list_scroll++;
                app->need_redraw = true;
            }
            return true;

        default:
            break;
    }
    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    app->playlist_running = false;
}

static void draw(SubGhzApp *app)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Playback mode (file browser opens directly on enter/back) */
    {
        /* Playback mode */
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);

        /* Header: playlist name + repeat info */
        {
            const char *pname = basename_from_path(app->playlist_path);
            char header[28];
            /* Truncate name to fit */
            snprintf(header, sizeof(header), "%.13s", pname);  /* 13 × 6px ≤ 78px; counter at x=90 */
            u8g2_DrawStr(&m1_u8g2, 2, 9, header);
        }

        /* Repeat counter on right side of header */
        {
            char rep[12];
            if (app->playlist_repeat_total == 0)
            {
                if (app->playlist_running)
                    snprintf(rep, sizeof(rep), "R:%u/Inf",
                             app->playlist_repeat_done + 1);
                else
                    snprintf(rep, sizeof(rep), "R:Inf");
            }
            else
            {
                if (app->playlist_running)
                    snprintf(rep, sizeof(rep), "R:%u/%u",
                             app->playlist_repeat_done + 1,
                             app->playlist_repeat_total);
                else
                    snprintf(rep, sizeof(rep), "R:%u",
                             app->playlist_repeat_total);
            }
            u8g2_DrawStr(&m1_u8g2, 90, 9, rep);
        }

        u8g2_DrawHLine(&m1_u8g2, 0, 10, M1_LCD_DISPLAY_WIDTH);

        /* File list */
        const uint8_t item_h   = m1_menu_item_h();
        const uint8_t text_ofs = item_h - 1;
        u8g2_SetFont(&m1_u8g2, m1_menu_font());
        for (uint8_t i = 0; i < PLAYLIST_LIST_VISIBLE &&
                            (list_scroll + i) < app->playlist_count; i++)
        {
            uint8_t idx = list_scroll + i;
            uint8_t y = M1_MENU_AREA_TOP + i * item_h;
            const char *fname = basename_from_path(app->playlist_files[idx]);

            /* Highlight currently transmitting file */
            if (app->playlist_running && idx == app->playlist_current)
            {
                u8g2_DrawRBox(&m1_u8g2, 1, y, M1_MENU_TEXT_W, item_h, 2);
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            }

            /* File index + name (truncated) */
            char line[26];
            snprintf(line, sizeof(line), "%u.%.20s", idx + 1, fname);
            u8g2_DrawStr(&m1_u8g2, 2, y + text_ofs, line);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        }

        /* Progress bar (below file list, above bottom bar at y=52) */
        {
            uint8_t bar_y = 46;
            u8g2_DrawFrame(&m1_u8g2, 2, bar_y, 124, 4);
            if (app->playlist_count > 0)
            {
                uint8_t fill = (uint8_t)(
                    (uint32_t)app->playlist_current * 122 / app->playlist_count);
                if (fill > 0)
                    u8g2_DrawBox(&m1_u8g2, 3, bar_y + 1, fill, 2);
            }
        }

        /* Bottom bar */
        if (app->playlist_running)
        {
            subghz_button_bar_draw(
                arrowleft_8x8, "Stop",
                NULL, NULL,
                NULL, NULL);
        }
        else
        {
            subghz_button_bar_draw(
                NULL, NULL,
                NULL, "R-/R+",
                NULL, "OK:Play");
        }
    }

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_playlist_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
