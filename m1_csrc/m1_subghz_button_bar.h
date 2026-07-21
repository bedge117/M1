/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_button_bar.h
 * @brief  Standardized 3-column bottom bar renderer for Sub-GHz scenes.
 *
 * Provides consistent button hint rendering across all Sub-GHz screens.
 * Fixed layout: left (x=2), center (x=48), right (x=96), y=53/61.
 */

#ifndef M1_SUBGHZ_BUTTON_BAR_H_
#define M1_SUBGHZ_BUTTON_BAR_H_

#include <stdint.h>

/**
 * @brief  Draw a standardized bottom bar with up to 3 button hints.
 *
 * Each slot can have an icon (8x8 bitmap) and/or a text label.
 * Pass NULL for unused slots.
 *
 * @param left_icon   Left slot icon (8x8 XBM), or NULL
 * @param left_text   Left slot label, or NULL
 * @param center_icon Center slot icon (8x8 XBM), or NULL
 * @param center_text Center slot label, or NULL
 * @param right_icon  Right slot icon (8x8/10x10 XBM), or NULL
 * @param right_text  Right slot label, or NULL
 */
void subghz_button_bar_draw(
    const uint8_t *left_icon,   const char *left_text,
    const uint8_t *center_icon, const char *center_text,
    const uint8_t *right_icon,  const char *right_text);

/**
 * @brief  Draw a standardized top status bar showing frequency, modulation,
 *         and an optional state indicator.
 *
 * @param freq_text  Frequency string (e.g. "433.92")
 * @param mod_text   Modulation string (e.g. "AM650")
 * @param state_text State indicator (e.g. "RX", "REC", NULL for none)
 * @param hopping    If true, show "HOP" instead of freq_text
 */
void subghz_status_bar_draw(
    const char *freq_text,
    const char *mod_text,
    const char *state_text,
    bool hopping);

/**
 * @brief  Draw an RSSI bar at a fixed position below the status bar.
 *
 * @param rssi_dbm  RSSI value in dBm (typically -120 to 0)
 */
void subghz_rssi_bar_draw(int16_t rssi_dbm);

#endif /* M1_SUBGHZ_BUTTON_BAR_H_ */
