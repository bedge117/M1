/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_config.c
 * @brief  Sub-GHz Config Scene — frequency, modulation, hopping, sound,
 *         TX power, and ISM region settings.
 *
 * Navigation:
 *   UP/DOWN   = select item
 *   LEFT/RIGHT = change value (clearly shown with ◀▶ arrows)
 *   BACK      = exit config (always)
 *   OK        = change value (same as RIGHT)
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_sub_ghz.h"
#include "m1_subghz_scene.h"
#include "m1_settings.h"
#include "m1_system.h"
#include "m1_virtual_kb.h"
#include "subghz_freq_presets.h"

/*============================================================================*/
/* Frequency & modulation preset tables (shared with m1_sub_ghz.c)            */
/* These are defined in m1_sub_ghz.c — we reference them via extern.          */
/*============================================================================*/

/* Frequency preset count & labels — derived from subghz_freq_presets.h so
 * the Config scene stays in sync if the preset table grows.
 * SUBGHZ_FREQ_PRESET_COUNT real entries (indices 0..COUNT-1) + 1 Custom entry
 * at index SUBGHZ_FREQ_PRESET_CUSTOM (== COUNT). */
#define CFG_FREQ_COUNT  (SUBGHZ_FREQ_PRESET_COUNT + 1)
#define CFG_MOD_COUNT    4

/* We need these string labels for display — pull from the same source */
extern const char *subghz_freq_labels[CFG_FREQ_COUNT];
extern const char *subghz_mod_labels[CFG_MOD_COUNT];

/* TX power accessors (static data in m1_sub_ghz.c) */
extern uint8_t     subghz_get_tx_power_idx_ext(void);
extern void        subghz_set_tx_power_idx_ext(uint8_t idx);
extern const char *subghz_get_tx_power_label_ext(uint8_t idx);
extern uint8_t     subghz_get_tx_power_count_ext(void);

/* Save format accessors (static data in m1_sub_ghz.c) */
extern uint8_t subghz_get_save_fmt_ext(void);
extern void    subghz_set_save_fmt_ext(uint8_t fmt);

/* Radio config accessors (static data in m1_sub_ghz.c) */
extern void subghz_set_freq_idx_ext(uint8_t idx);
extern void subghz_set_mod_idx_ext(uint8_t idx);
extern void subghz_set_hopping_ext(bool v);
extern void subghz_set_sound_ext(bool v);
extern void subghz_set_autosave_ext(bool v);
extern int8_t subghz_get_rssi_threshold_ext(void);
extern void   subghz_set_rssi_threshold_ext(int8_t v);
extern uint32_t subghz_get_user_custom_freq_ext(void);
extern void     subghz_set_user_custom_freq_ext(uint32_t hz);

/* Phase 12 — receiver history quality-of-life toggles */
extern bool subghz_get_remove_duplicates_ext(void);
extern void subghz_set_remove_duplicates_ext(bool v);
extern bool subghz_get_delete_old_signals_ext(void);
extern void subghz_set_delete_old_signals_ext(bool v);

/* ISM region data (non-static globals in m1_sub_ghz.c / m1_sub_ghz.h) */
extern const char *subghz_ism_regions_text[];

/*============================================================================*/
/* Config items                                                               */
/*============================================================================*/

#define CFG_ITEMS       11
#define CFG_FREQUENCY   0
#define CFG_HOPPING     1
#define CFG_MODULATION  2
#define CFG_SOUND       3
#define CFG_AUTOSAVE    4
#define CFG_TX_POWER    5
#define CFG_ISM_REGION  6
#define CFG_SAVE_FMT    7
#define CFG_RSSI_THRESH 8
#define CFG_REMOVE_DUPS 9   /* Phase 12 */
#define CFG_DELETE_OLD  10  /* Phase 12 */

static uint8_t cfg_sel = 0;
static uint8_t cfg_scroll = 0;  /* First visible item (scrolling if items > visible) */
static bool    cfg_hide_hopping = false;  /* true when opened from Read Raw */

/* Effective item count and display→real item index mapping.
 * When hopping is hidden (Read Raw context), the Hopping row is
 * skipped so display indices above it are shifted down by one. */
static uint8_t cfg_count(void)
{
    return cfg_hide_hopping ? CFG_ITEMS - 1 : CFG_ITEMS;
}

static uint8_t cfg_real(uint8_t display_idx)
{
    if (!cfg_hide_hopping || display_idx < CFG_HOPPING)
        return display_idx;
    return display_idx + 1;  /* skip the hidden Hopping slot */
}

/* Layout constants — aligned with menu scene (no button bar) */
#define CFG_AREA_TOP     12   /* Y below title + separator line      */
#define CFG_TEXT_W      122   /* Highlight / text area width           */
#define CFG_SCROLLBAR_X 124   /* Scrollbar left edge (3px wide)       */
#define CFG_SCROLLBAR_W   3   /* Scrollbar track width                */

static const char *cfg_item_labels[CFG_ITEMS] = {
    "Frequency:",
    "Hopping:",
    "Modulation:",
    "Sound:",
    "Autosave:",
    "TX Power:",
    "ISM Region:",
    "Save Format:",
    "RSSI Thresh:",
    "Remove Dups:",   /* Phase 12 */
    "Delete Old:",    /* Phase 12 */
};

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

static const char *get_value_text(SubGhzApp *app, uint8_t item)
{
    static char freq_buf[16];
    switch (item)
    {
        case CFG_FREQUENCY:
            if (subghz_freq_labels)
            {
                /* For Custom entry (index 62), show the stored MHz label.
                 * subghz_set_user_custom_freq_ext() keeps freq_labels[Custom]
                 * updated with the formatted MHz string, so we can use it
                 * directly — falling through to the normal return path. */
                if (app->freq_idx == CFG_FREQ_COUNT - 1)
                    return subghz_freq_labels[app->freq_idx];
                return subghz_freq_labels[app->freq_idx];
            }
            snprintf(freq_buf, sizeof(freq_buf), "Preset %d", app->freq_idx);
            return freq_buf;
        case CFG_HOPPING:
            return app->hopping ? "ON" : "OFF";
        case CFG_MODULATION:
            if (subghz_mod_labels)
                return subghz_mod_labels[app->mod_idx];
            return "?";
        case CFG_SOUND:
            return app->sound ? "ON" : "OFF";
        case CFG_AUTOSAVE:
            return app->autosave ? "ON" : "OFF";
        case CFG_TX_POWER:
            return subghz_get_tx_power_label_ext(subghz_get_tx_power_idx_ext());
        case CFG_ISM_REGION:
            return subghz_ism_regions_text[m1_device_stat.config.ism_band_region];
        case CFG_SAVE_FMT:
            return subghz_get_save_fmt_ext() ? "M1 (.sgh)" : "Flipper (.sub)";
        case CFG_RSSI_THRESH:
        {
            static char rssi_buf[8];
            snprintf(rssi_buf, sizeof(rssi_buf), "%d dBm", (int)subghz_get_rssi_threshold_ext());
            return rssi_buf;
        }
        case CFG_REMOVE_DUPS:
            return subghz_get_remove_duplicates_ext() ? "ON" : "OFF";
        case CFG_DELETE_OLD:
            return subghz_get_delete_old_signals_ext() ? "ON" : "OFF";
        default:
            return "";
    }
}

static void change_value(SubGhzApp *app, uint8_t item, int8_t dir)
{
    switch (item)
    {
        case CFG_FREQUENCY:
            if (dir > 0)
                app->freq_idx = (app->freq_idx + 1) % CFG_FREQ_COUNT;
            else
                app->freq_idx = (app->freq_idx > 0) ?
                    app->freq_idx - 1 : CFG_FREQ_COUNT - 1;
            break;
        case CFG_HOPPING:
            app->hopping = !app->hopping;
            break;
        case CFG_MODULATION:
            if (dir > 0)
                app->mod_idx = (app->mod_idx + 1) % CFG_MOD_COUNT;
            else
                app->mod_idx = (app->mod_idx > 0) ?
                    app->mod_idx - 1 : CFG_MOD_COUNT - 1;
            break;
        case CFG_SOUND:
            app->sound = !app->sound;
            break;
        case CFG_AUTOSAVE:
            app->autosave = !app->autosave;
            break;
        case CFG_TX_POWER:
        {
            uint8_t tx_count = subghz_get_tx_power_count_ext();
            uint8_t tx_idx   = subghz_get_tx_power_idx_ext();
            if (dir > 0)
                tx_idx = (tx_idx + 1) % tx_count;
            else
                tx_idx = (tx_idx > 0) ? tx_idx - 1 : tx_count - 1;
            subghz_set_tx_power_idx_ext(tx_idx);
            break;
        }
        case CFG_ISM_REGION:
            if (dir > 0)
            {
                m1_device_stat.config.ism_band_region++;
                if (m1_device_stat.config.ism_band_region >= SUBGHZ_ISM_BAND_REGIONS_LIST)
                    m1_device_stat.config.ism_band_region = 0;
            }
            else
            {
                if (m1_device_stat.config.ism_band_region > 0)
                    m1_device_stat.config.ism_band_region--;
                else
                    m1_device_stat.config.ism_band_region = SUBGHZ_ISM_BAND_REGIONS_LIST - 1;
            }
            break;
        case CFG_SAVE_FMT:
            subghz_set_save_fmt_ext(subghz_get_save_fmt_ext() ^ 1);
            break;
        case CFG_RSSI_THRESH:
        {
            /* Step in 5 dBm increments, range -50 to -100 */
            int8_t t = subghz_get_rssi_threshold_ext();
            t = (int8_t)(t + dir * (-5));
            subghz_set_rssi_threshold_ext(t);
            break;
        }
        case CFG_REMOVE_DUPS:
            (void)dir;
            subghz_set_remove_duplicates_ext(!subghz_get_remove_duplicates_ext());
            break;
        case CFG_DELETE_OLD:
            (void)dir;
            subghz_set_delete_old_signals_ext(!subghz_get_delete_old_signals_ext());
            break;
    }
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    cfg_sel = 0;
    cfg_scroll = 0;
    /* Hopping is not meaningful in Read Raw — hide it when Config is
     * opened from that scene so the user cannot accidentally enable it.
     * scene_depth reflects the post-push state, so the current scene (Config)
     * is at [depth-1] and the parent is at [depth-2].  depth >= 2 guarantees
     * depth-2 >= 0, which is always a valid zero-based stack index. */
    cfg_hide_hopping = (app->scene_depth >= 2 &&
        app->scene_stack[app->scene_depth - 2] == SubGhzSceneReadRaw);
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (event)
    {
        case SubGhzEventBack:
            /* Sync app state to persistent subghz_cfg fields */
            subghz_set_freq_idx_ext(app->freq_idx);
            subghz_set_mod_idx_ext(app->mod_idx);
            subghz_set_hopping_ext(app->hopping);
            subghz_set_sound_ext(app->sound);
            subghz_set_autosave_ext(app->autosave);
            /* TX power is already updated via its accessor when changed. */
            /* Persist all settings to SD */
            settings_save_to_sd();
            subghz_scene_pop(app);
            return true;

        case SubGhzEventUp:
            cfg_sel = (cfg_sel > 0) ? cfg_sel - 1 : cfg_count() - 1;
            /* Adjust scroll window */
            if (cfg_sel < cfg_scroll)
                cfg_scroll = cfg_sel;
            if (cfg_sel == cfg_count() - 1)
            {
                uint8_t vis = M1_MENU_VIS(cfg_count());
                cfg_scroll = (cfg_count() > vis) ? cfg_count() - vis : 0;
            }
            app->need_redraw = true;
            return true;

        case SubGhzEventDown:
            cfg_sel = (cfg_sel + 1) % cfg_count();
            /* Adjust scroll window */
            {
                uint8_t vis = M1_MENU_VIS(cfg_count());
                if (cfg_sel >= cfg_scroll + vis)
                    cfg_scroll = cfg_sel - vis + 1;
            }
            if (cfg_sel == 0)
                cfg_scroll = 0;
            app->need_redraw = true;
            return true;

        case SubGhzEventRight:
        case SubGhzEventOk:
            /* Special case: OK on the Custom frequency entry opens VKB */
            if (cfg_real(cfg_sel) == CFG_FREQUENCY && app->freq_idx == CFG_FREQ_COUNT - 1)
            {
                char mhz_str[12] = "";
                uint32_t cur_hz = subghz_get_user_custom_freq_ext();
                snprintf(mhz_str, sizeof(mhz_str), "%lu.%02lu",
                         (unsigned long)(cur_hz / 1000000UL),
                         (unsigned long)((cur_hz % 1000000UL) / 10000UL));

                char new_mhz[12] = "";
                if (m1_vkb_get_text("Freq (MHz):", mhz_str, new_mhz, sizeof(new_mhz)))
                {
                    /* Parse the entered value (e.g. "868.35") */
                    char *dot = NULL;
                    unsigned long whole = strtoul(new_mhz, &dot, 10);
                    unsigned long frac  = 0;
                    if (dot && *dot == '.')
                    {
                        char frac_buf[7] = "000000";
                        const char *fp = dot + 1;
                        for (uint8_t fi = 0; fi < 6 && isdigit((unsigned char)fp[fi]); fi++)
                            frac_buf[fi] = fp[fi];
                        frac = strtoul(frac_buf, NULL, 10);
                    }
                    uint32_t hz = (uint32_t)(whole * 1000000UL + frac);
                    /* Validate SI4463 range (300–930 MHz) */
                    if (hz >= 300000000UL && hz <= 930000000UL)
                        subghz_set_user_custom_freq_ext(hz);
                }
            }
            else
            {
                change_value(app, cfg_real(cfg_sel), +1);
            }
            app->need_redraw = true;
            return true;

        case SubGhzEventLeft:
            change_value(app, cfg_real(cfg_sel), -1);
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

static void draw(SubGhzApp *app)
{
    const uint8_t item_h   = m1_menu_item_h();
    const uint8_t text_ofs = item_h - 1;

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Title */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 120, "Config", TEXT_ALIGN_CENTER);

    /* Separator line (consistent with menu scene) */
    u8g2_DrawHLine(&m1_u8g2, 0, 10, M1_LCD_DISPLAY_WIDTH);

    /* Config items (scrollable) */
    u8g2_SetFont(&m1_u8g2, m1_menu_font());

    /* Compute value x-position so the longest label never abuts its value.
     * In medium/large fonts the labels are wider, so the value column
     * shifts right automatically, but keep it inside the text area so
     * long values and selected-row arrows still fit before the scrollbar. */
    u8g2_uint_t max_lw = 0;
    for (uint8_t j = 0; j < cfg_count(); j++)
    {
        u8g2_uint_t w = u8g2_GetStrWidth(&m1_u8g2, cfg_item_labels[cfg_real(j)]);
        if (w > max_lw) max_lw = w;
    }
    u8g2_uint_t val_x_pref = 4 + max_lw + 4;   /* label start + max width + gap */
    u8g2_uint_t val_x_max  = CFG_TEXT_W - 18;   /* keep value/arrows within text area */
    u8g2_uint_t val_x      = (val_x_pref < val_x_max) ? val_x_pref : val_x_max;
    u8g2_uint_t arrow_x    = val_x - 6;         /* "<" sits one glyph-cell left  */

    uint8_t visible = M1_MENU_VIS(cfg_count());
    for (uint8_t v = 0; v < visible; v++)
    {
        uint8_t i = cfg_scroll + v;
        if (i >= cfg_count()) break;
        uint8_t ri = cfg_real(i);  /* real item index for labels/values */

        uint8_t y = CFG_AREA_TOP + v * item_h;

        if (i == cfg_sel)
        {
            /* Highlight selected row — rounded corners, leave room for scrollbar */
            u8g2_DrawRBox(&m1_u8g2, 1, y, CFG_TEXT_W, item_h, 2);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        }

        /* Label on left */
        u8g2_DrawStr(&m1_u8g2, 4, y + text_ofs, cfg_item_labels[ri]);

        /* Value on right with ◀ ▶ arrows for selected item */
        const char *val = get_value_text(app, ri);
        if (i == cfg_sel)
        {
            u8g2_DrawStr(&m1_u8g2, arrow_x, y + text_ofs, "<");
            u8g2_DrawStr(&m1_u8g2, val_x, y + text_ofs, val);
            u8g2_uint_t vw = u8g2_GetStrWidth(&m1_u8g2, val);
            u8g2_DrawStr(&m1_u8g2, val_x + vw + 2, y + text_ofs, ">");
        }
        else
        {
            u8g2_DrawStr(&m1_u8g2, val_x, y + text_ofs, val);
        }

        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    }

    /* Scrollbar — only when items exceed visible area */
    if (cfg_count() > visible)
    {
        uint8_t sb_area_h   = visible * item_h;
        uint8_t sb_handle_h = sb_area_h / cfg_count();
        if (sb_handle_h < 6) sb_handle_h = 6;
        uint8_t sb_travel_h = (sb_area_h > sb_handle_h) ? (sb_area_h - sb_handle_h) : 0;
        uint8_t sb_handle_y = CFG_AREA_TOP;
        if (cfg_count() > 1)
            sb_handle_y += (uint8_t)((uint16_t)sb_travel_h * cfg_sel / (cfg_count() - 1));

        u8g2_DrawVLine(&m1_u8g2, CFG_SCROLLBAR_X + CFG_SCROLLBAR_W / 2,
                       CFG_AREA_TOP, sb_area_h);
        u8g2_DrawRBox(&m1_u8g2, CFG_SCROLLBAR_X, sb_handle_y,
                      CFG_SCROLLBAR_W, sb_handle_h, 1);
    }

    /* No button bar — the < > arrows on selected items clearly indicate
     * that LEFT/RIGHT changes the value.  UP/DOWN is self-evident. */

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_config_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = draw,
};
