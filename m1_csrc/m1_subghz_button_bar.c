/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_button_bar.c
 * @brief  Standardized bottom bar, status bar, and RSSI bar for Sub-GHz scenes.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_subghz_button_bar.h"

/*============================================================================*/
/* Layout constants                                                           */
/*============================================================================*/

/* Bottom bar: 3-column layout on a 128px wide display.
 * Momentum style: three individual rounded-corner buttons with 1px gaps.
 *   42 + 1 + 42 + 1 + 42 = 128 px (exact fill, no remainder).
 */
#define BAR_Y_TOP      52   /* Top of bottom bar area (12px high) */
#define BAR_H          12   /* Height of bottom bar */
#define BAR_ICON_Y     54   /* Y for 8x8 icons (1px from bar top for centering) */
#define BAR_TEXT_Y     61   /* Y baseline for text */

/* Button geometry */
#define BTN_W          42   /* Width of each button */
#define BTN_GAP         1   /* Gap between buttons (background color shows through) */
#define BTN_R           2   /* Corner radius */
#define BTN_0_X         0                         /* Left button X */
#define BTN_1_X        (BTN_0_X + BTN_W + BTN_GAP) /* Center button X */
#define BTN_2_X        (BTN_1_X + BTN_W + BTN_GAP) /* Right button X */

/* Status bar */
#define STATUS_BAR_H   10   /* Height of top status bar */
#define STATUS_BAR_Y    0   /* Y position */

/* RSSI bar */
#define RSSI_BAR_Y     11   /* Just below status bar */
#define RSSI_BAR_H      3   /* Height of RSSI bar */
#define RSSI_MIN     -110   /* dBm mapped to 0 pixels */
#define RSSI_MAX      -30   /* dBm mapped to full width (128px) */

/*============================================================================*/
/* Bottom bar                                                                 */
/*============================================================================*/

/* Draw one Momentum-style button: filled rounded box, white content inside.
 * The combined icon+text block is centred horizontally within the button.
 * icon_on_right: false = icon left of text (left/center slots),
 *                true  = text left of icon (right slot). */
static void draw_btn(uint8_t bx,
                     const uint8_t *icon, const char *text,
                     bool icon_on_right)
{
    /* Filled rounded box */
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_DrawRBox(&m1_u8g2, bx, BAR_Y_TOP, BTN_W, BAR_H, BTN_R);
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);

    /* Measure content and centre it within the button.
     * Use u8g2_uint_t to avoid uint8_t truncation when text is wide. */
    u8g2_uint_t text_w    = text ? u8g2_GetStrWidth(&m1_u8g2, text) : 0u;
    u8g2_uint_t icon_w    = icon ? 8u : 0u;
    u8g2_uint_t gap       = (icon && text) ? 2u : 0u;
    u8g2_uint_t content_w = icon_w + gap + text_w;
    u8g2_uint_t cx = (u8g2_uint_t)bx + (content_w < BTN_W ? (BTN_W - content_w) / 2u : 1u);

    if (!icon_on_right)
    {
        /* Left/center slot: icon first (left of text) */
        if (icon) u8g2_DrawXBMP(&m1_u8g2, (u8g2_uint_t)cx,              BAR_ICON_Y, 8, 8, icon);
        if (text) u8g2_DrawStr( &m1_u8g2, (u8g2_uint_t)(cx + icon_w + gap), BAR_TEXT_Y, text);
    }
    else
    {
        /* Right slot: text first, icon to its right */
        if (text) u8g2_DrawStr( &m1_u8g2, (u8g2_uint_t)cx,                     BAR_TEXT_Y, text);
        if (icon) u8g2_DrawXBMP(&m1_u8g2, (u8g2_uint_t)(cx + text_w + gap), BAR_ICON_Y, 8, 8, icon);
    }
}

void subghz_button_bar_draw(
    const uint8_t *left_icon,   const char *left_text,
    const uint8_t *center_icon, const char *center_text,
    const uint8_t *right_icon,  const char *right_text)
{
    /* Use slim font for button labels */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    /* Draw each occupied slot as an individual rounded button.
     * Positions are fixed so gaps always appear between buttons. */
    if (left_icon   || left_text)
        draw_btn(BTN_0_X, left_icon,   left_text,   false);
    if (center_icon || center_text)
        draw_btn(BTN_1_X, center_icon, center_text, false);
    if (right_icon  || right_text)
        draw_btn(BTN_2_X, right_icon,  right_text,  true);

    /* Restore draw color */
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
}

/*============================================================================*/
/* Status bar                                                                 */
/*============================================================================*/

void subghz_status_bar_draw(
    const char *freq_text,
    const char *mod_text,
    const char *state_text,
    bool hopping)
{
    /* Black text on white background — matches Momentum/Flipper header style */
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    /* Left side: frequency or "Hopping" */
    if (hopping)
        u8g2_DrawStr(&m1_u8g2, 2, 8, "HOP");
    else if (freq_text)
        u8g2_DrawStr(&m1_u8g2, 2, 8, freq_text);

    /* Center: modulation */
    if (mod_text)
    {
        /* Centre the modulation text — use u8g2_uint_t to avoid truncation */
        u8g2_uint_t tw = u8g2_GetStrWidth(&m1_u8g2, mod_text);
        u8g2_uint_t cx = tw < M1_LCD_DISPLAY_WIDTH ? (M1_LCD_DISPLAY_WIDTH - tw) / 2u : 0u;
        u8g2_DrawStr(&m1_u8g2, cx, 8, mod_text);
    }

    /* Right side: state indicator */
    if (state_text)
    {
        u8g2_uint_t tw = u8g2_GetStrWidth(&m1_u8g2, state_text);
        u8g2_uint_t x  = tw + 2u < M1_LCD_DISPLAY_WIDTH ? M1_LCD_DISPLAY_WIDTH - tw - 2u : 0u;
        u8g2_DrawStr(&m1_u8g2, x, 8, state_text);
    }

    /* Separator line at bottom of status bar */
    u8g2_DrawHLine(&m1_u8g2, 0, STATUS_BAR_H, M1_LCD_DISPLAY_WIDTH);
}

/*============================================================================*/
/* RSSI bar                                                                   */
/*============================================================================*/

void subghz_rssi_bar_draw(int16_t rssi_dbm)
{
    char rssi_str[8];

    /* Clamp RSSI to displayable range */
    if (rssi_dbm < RSSI_MIN) rssi_dbm = RSSI_MIN;
    if (rssi_dbm > RSSI_MAX) rssi_dbm = RSSI_MAX;

    /* Map to 0-108 pixels (leave 20px for text) */
    uint8_t bar_max_w = M1_LCD_DISPLAY_WIDTH - 20;
    uint8_t bar_w = (uint8_t)((uint32_t)(rssi_dbm - RSSI_MIN) * bar_max_w / (RSSI_MAX - RSSI_MIN));

    /* Clear RSSI bar area */
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
    u8g2_DrawBox(&m1_u8g2, 0, RSSI_BAR_Y, M1_LCD_DISPLAY_WIDTH, RSSI_BAR_H + 1);

    /* Draw filled bar */
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    if (bar_w > 0)
        u8g2_DrawBox(&m1_u8g2, 0, RSSI_BAR_Y, bar_w, RSSI_BAR_H);

    /* Draw RSSI text */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    snprintf(rssi_str, sizeof(rssi_str), "%d", rssi_dbm);
    u8g2_DrawStr(&m1_u8g2, bar_max_w + 2, RSSI_BAR_Y + RSSI_BAR_H, rssi_str);
}
