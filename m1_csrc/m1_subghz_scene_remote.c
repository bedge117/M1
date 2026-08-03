/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_remote.c
 * @brief  Sub-GHz Remote scene — multi-button RF remote control (load + create).
 *
 * A `.rem` manifest maps the five hardware buttons (UP / DOWN / LEFT / RIGHT /
 * OK) to individual `.sub` signal files. Pressing a mapped button fires that
 * signal immediately.
 *
 * On entry the scene offers two choices:
 *   - Load existing : browse 0:/SUBGHZ for a `.rem` and use it.
 *   - Create new    : assign a saved `.sub` to each of the five buttons (or skip
 *                     a button), name it, and save a `.rem` to 0:/SUBGHZ/ — then
 *                     it's immediately usable and reloadable later.
 *
 * Manifest format (plain text, one directive per line):
 *   # comment
 *   up:    /SUBGHZ/gate_up.sub
 *   down:  /SUBGHZ/gate_down.sub
 *   left:  /SUBGHZ/ch_left.sub
 *   right: /SUBGHZ/ch_right.sub
 *   ok:    /SUBGHZ/arm.sub
 *
 * Any button not listed is shown as "---" and does nothing. Flipper-style paths
 * (/ext/subghz/…) are remapped automatically via subghz_remap_flipper_path().
 *
 * TX is handed off to the async SubGhzSceneTransmitter (Phase 3b-2b-iv). */

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
#include "m1_virtual_kb.h"
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

/* Scene sub-states */
enum { REM_STATE_CHOICE = 0, REM_STATE_CREATE, REM_STATE_PAD };

static uint8_t rem_state;        /* current sub-state */
static uint8_t rem_choice_sel;   /* CHOICE: 0 = Load existing, 1 = Create new */
static uint8_t rem_create_btn;   /* CREATE: button currently being assigned (0..4) */

static const char *const rem_btn_names[SUBGHZ_REMOTE_BUTTON_COUNT] =
    { "UP", "DOWN", "LEFT", "RIGHT", "OK" };
static const char *const rem_btn_keys[SUBGHZ_REMOTE_BUTTON_COUNT] =
    { "up", "down", "left", "right", "ok" };

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

/* Derive a button label (basename without .sub extension) from a file path. */
static void remote_set_label(SubGhzApp *app, uint8_t btn, const char *file_path)
{
    const char *base = strrchr(file_path, '/');
    base = base ? base + 1 : file_path;
    strncpy(app->remote_label[btn], base, sizeof(app->remote_label[btn]) - 1);
    app->remote_label[btn][sizeof(app->remote_label[btn]) - 1] = '\0';
    char *ext = strrchr(app->remote_label[btn], '.');
    if (ext)
        *ext = '\0';
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
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
            line[--len] = '\0';

        if (len == 0 || line[0] == '#')
            continue;

        uint8_t btn_idx = 0xFF;
        const char *pfx = NULL;

        if      (strncasecmp(line, "up:",    3) == 0) { btn_idx = REM_BTN_UP;    pfx = line + 3; }
        else if (strncasecmp(line, "down:",  5) == 0) { btn_idx = REM_BTN_DOWN;  pfx = line + 5; }
        else if (strncasecmp(line, "left:",  5) == 0) { btn_idx = REM_BTN_LEFT;  pfx = line + 5; }
        else if (strncasecmp(line, "right:", 6) == 0) { btn_idx = REM_BTN_RIGHT; pfx = line + 6; }
        else if (strncasecmp(line, "ok:",    3) == 0) { btn_idx = REM_BTN_OK;    pfx = line + 3; }

        if (btn_idx == 0xFF || !pfx)
            continue;

        while (*pfx == ' ' || *pfx == '\t')
            pfx++;
        if (*pfx == '\0')
            continue;

        subghz_remap_flipper_path(pfx, app->remote_files[btn_idx],
                             sizeof(app->remote_files[btn_idx]));
        remote_set_label(app, btn_idx, app->remote_files[btn_idx]);
    }

    f_close(&fil);

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

    char fat_path[72];
    snprintf(fat_path, sizeof(fat_path), "0:%s", app->remote_files[btn_idx]);

    strncpy(app->tx_path, fat_path, sizeof(app->tx_path) - 1);
    app->tx_path[sizeof(app->tx_path) - 1] = '\0';
    app->tx_repeat_count = 1U;             /* static keys: 1 burst */
    app->tx_mode         = 0U;             /* SUBGHZ_TX_MODE_SINGLE */
    app->tx_autostart    = true;           /* one-press fire UX */
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
/* Create flow                                                                */
/*============================================================================*/

/* Browse for a .sub and assign it to the current create button. Cancelling the
 * browser leaves the button unassigned (the "skip" outcome). */
static void remote_assign_current(SubGhzApp *app)
{
    S_M1_file_info *f_info = storage_browse("0:/SUBGHZ");
    if (f_info && f_info->file_is_selected)
    {
        snprintf(app->remote_files[rem_create_btn],
                 sizeof(app->remote_files[rem_create_btn]),
                 "/SUBGHZ/%s", f_info->file_name);
        remote_set_label(app, rem_create_btn, app->remote_files[rem_create_btn]);
    }
}

/* After all buttons are assigned: name it, write the .rem, and mark loaded.
 * Returns true on success (a usable remote now lives in app->remote_*). */
static bool remote_finish_create(SubGhzApp *app)
{
    /* Require at least one mapped button. */
    bool any = false;
    for (uint8_t i = 0; i < SUBGHZ_REMOTE_BUTTON_COUNT; i++)
        if (app->remote_files[i][0]) any = true;
    if (!any)
        return false;

    char def_name[24]  = "remote";
    char new_name[24]  = "";
    if (!m1_vkb_get_text("Remote name:", def_name, new_name, sizeof(new_name)))
        return false;                       /* cancelled */
    if (new_name[0] == '\0')
        return false;

    /* Save to 0:/SUBGHZ/<name>.rem — same folder the Load browser lists. */
    char fat_path[80];
    snprintf(fat_path, sizeof(fat_path), "0:/SUBGHZ/%s.rem", new_name);

    FIL fil;
    if (f_open(&fil, fat_path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return false;

    for (uint8_t b = 0; b < SUBGHZ_REMOTE_BUTTON_COUNT; b++)
    {
        if (app->remote_files[b][0] == '\0')
            continue;
        char line[100];
        UINT bw = 0;
        int n = snprintf(line, sizeof(line), "%s: %s\n",
                         rem_btn_keys[b], app->remote_files[b]);
        if (n > 0)
            f_write(&fil, line, (UINT)n, &bw);
    }
    f_close(&fil);

    snprintf(app->remote_path, sizeof(app->remote_path),
             "/SUBGHZ/%s.rem", new_name);
    app->remote_loaded = true;
    return true;
}

/* Advance to the next create button; on the last, name + save and go to the pad
 * (or back to the choice screen if the user cancels naming / nothing assigned). */
static void create_advance(SubGhzApp *app)
{
    rem_create_btn++;
    if (rem_create_btn < SUBGHZ_REMOTE_BUTTON_COUNT)
        return;

    if (remote_finish_create(app))
    {
        rem_state = REM_STATE_PAD;
    }
    else
    {
        remote_clear(app);
        rem_state = REM_STATE_CHOICE;
        rem_choice_sel = 1;                 /* keep "Create new" highlighted */
    }
}

/*============================================================================*/
/* Draw                                                                       */
/*============================================================================*/

#define BTN_NAME(app, i) ((app)->remote_label[i][0] ? (app)->remote_label[i] : "---")

static void draw_choice(SubGhzApp *app)
{
    (void)app;
    u8g2_ClearBuffer(&m1_u8g2);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 124, "Remote", TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    static const char *opts[2] = { "Load existing", "Create new" };
    for (uint8_t i = 0; i < 2; i++)
    {
        uint8_t y = 24 + i * 14;
        if (i == rem_choice_sel)
        {
            u8g2_DrawRBox(&m1_u8g2, 1, y - 9, 126, 12, 2);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        }
        u8g2_DrawStr(&m1_u8g2, 6, y, opts[i]);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    }
    u8g2_DrawStr(&m1_u8g2, 4, 62, "OK: select   BACK: exit");
    u8g2_SendBuffer(&m1_u8g2);
}

static void draw_create(SubGhzApp *app)
{
    u8g2_ClearBuffer(&m1_u8g2);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 124, "Create Remote", TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    char buf[40];
    snprintf(buf, sizeof(buf), "Assign %s button", rem_btn_names[rem_create_btn]);
    u8g2_DrawStr(&m1_u8g2, 4, 24, buf);

    /* Running tally of what's assigned so far. */
    uint8_t assigned = 0;
    for (uint8_t i = 0; i < SUBGHZ_REMOTE_BUTTON_COUNT; i++)
        if (app->remote_files[i][0]) assigned++;
    snprintf(buf, sizeof(buf), "Button %u/%u  (%u set)",
             (unsigned)(rem_create_btn + 1),
             (unsigned)SUBGHZ_REMOTE_BUTTON_COUNT, (unsigned)assigned);
    u8g2_DrawStr(&m1_u8g2, 4, 38, buf);

    u8g2_DrawStr(&m1_u8g2, 4, 62, "OK:pick  R:skip  BACK:cancel");
    u8g2_SendBuffer(&m1_u8g2);
}

static void draw_pad(SubGhzApp *app)
{
    u8g2_ClearBuffer(&m1_u8g2);

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    const char *rname = strrchr(app->remote_path, '/');
    rname = rname ? rname + 1 : app->remote_path;
    char title_buf[32];
    snprintf(title_buf, sizeof(title_buf), "Remote: %.12s", rname);
    char *dot = strrchr(title_buf, '.');
    if (dot)
        *dot = '\0';
    m1_draw_text(&m1_u8g2, 2, 9, 124, title_buf, TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 32, 21, "\x18");
    u8g2_DrawStr(&m1_u8g2, 42, 21, BTN_NAME(app, REM_BTN_UP));
    u8g2_DrawStr(&m1_u8g2, 0, 34, "\x1b");
    u8g2_DrawStr(&m1_u8g2, 8, 34, BTN_NAME(app, REM_BTN_LEFT));
    u8g2_DrawStr(&m1_u8g2, 52, 34, "OK:");
    u8g2_DrawStr(&m1_u8g2, 68, 34, BTN_NAME(app, REM_BTN_OK));
    u8g2_DrawStr(&m1_u8g2, 96, 34, BTN_NAME(app, REM_BTN_RIGHT));
    u8g2_DrawStr(&m1_u8g2, 122, 34, "\x1a");
    u8g2_DrawStr(&m1_u8g2, 32, 47, "\x19");
    u8g2_DrawStr(&m1_u8g2, 42, 47, BTN_NAME(app, REM_BTN_DOWN));
    u8g2_DrawStr(&m1_u8g2, 4, 62, "Press to send");

    u8g2_SendBuffer(&m1_u8g2);
}

static void scene_draw(SubGhzApp *app)
{
    switch (rem_state)
    {
        case REM_STATE_CHOICE: draw_choice(app); break;
        case REM_STATE_CREATE: draw_create(app); break;
        default:               draw_pad(app);    break;
    }
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Clear the autostart hint defensively (a prior push may have set it). */
    app->tx_autostart = false;

    /* Returning from the Transmitter (after a fire) keeps us on the pad; a fresh
     * entry with nothing loaded shows the Load/Create choice. */
    if (app->remote_loaded)
    {
        rem_state = REM_STATE_PAD;
    }
    else
    {
        rem_state = REM_STATE_CHOICE;
        rem_choice_sel = 0;
    }
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    if (rem_state == REM_STATE_CHOICE)
    {
        switch (event)
        {
            case SubGhzEventUp:
            case SubGhzEventDown:
                rem_choice_sel ^= 1u;
                app->need_redraw = true;
                return true;
            case SubGhzEventOk:
                if (rem_choice_sel == 0)
                {
                    /* Load existing */
                    if (remote_load_manifest(app))
                    {
                        app->remote_loaded = true;
                        rem_state = REM_STATE_PAD;
                    }
                    else
                    {
                        subghz_scene_pop(app);   /* cancelled / invalid */
                        return true;
                    }
                }
                else
                {
                    /* Create new */
                    remote_clear(app);
                    rem_create_btn = 0;
                    rem_state = REM_STATE_CREATE;
                }
                app->need_redraw = true;
                return true;
            case SubGhzEventBack:
                subghz_scene_pop(app);
                return true;
            default:
                break;
        }
        return false;
    }

    if (rem_state == REM_STATE_CREATE)
    {
        switch (event)
        {
            case SubGhzEventOk:
                remote_assign_current(app);   /* blocking browse; may skip on cancel */
                create_advance(app);
                app->need_redraw = true;
                return true;
            case SubGhzEventRight:
                create_advance(app);          /* skip this button */
                app->need_redraw = true;
                return true;
            case SubGhzEventBack:
                remote_clear(app);
                rem_state = REM_STATE_CHOICE;
                rem_choice_sel = 1;
                app->need_redraw = true;
                return true;
            default:
                break;
        }
        return false;
    }

    /* REM_STATE_PAD — fire mapped signals */
    switch (event)
    {
        case SubGhzEventBack:
            remote_clear(app);
            subghz_scene_pop(app);
            return true;
        case SubGhzEventUp:    remote_fire(app, REM_BTN_UP);    return true;
        case SubGhzEventDown:  remote_fire(app, REM_BTN_DOWN);  return true;
        case SubGhzEventLeft:  remote_fire(app, REM_BTN_LEFT);  return true;
        case SubGhzEventRight: remote_fire(app, REM_BTN_RIGHT); return true;
        case SubGhzEventOk:    remote_fire(app, REM_BTN_OK);    return true;
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
