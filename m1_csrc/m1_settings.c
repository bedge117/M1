/* See COPYING.txt for license details. */

/*
*
*  m1_settings.c
*
*  M1 RFID functions
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_settings.h"
#include "m1_rpc.h"       /* m1_rpc_esp_fw_str — ESP coprocessor fw for About */
#include "m1_wifi.h"      /* m1_wifi_boot_connect, m1_wifi_primary_ssid */
#include "m1_wifi_cred.h" /* WIFI_CRED_SSID_MAX_LEN */
#include "m1_buzzer.h"
#include "m1_lcd.h"
#include "m1_lp5814.h"
#include "m1_display.h"
#include "m1_scene.h"    /* m1_scene_draw_menu — portrait-aware list, for the orientation picker */
#include "ff.h"
#include "m1_log_debug.h"
#include "m1_fw_update_bl.h"
#include "m1_system.h"
#include "m1_file_util.h"
#include "m1_esp32_hal.h"  /* m1_esp32_init / m1_esp32_reboot / init status */
#include "m1_esp_client.h" /* m1_esp_client_ble_direct — Bluetooth Direct toggle */

/*************************** D E F I N E S ************************************/

#define SETTINGS_TAG              "SETT"
#define SETTINGS_FILE_PATH        "0:/System/settings.cfg"
#define SETTINGS_FILE_MAX_SIZE    512

#define SETTING_ABOUT_CHOICES_MAX		2 //5

#define ABOUT_BOX_Y_POS_ROW_1			10
#define ABOUT_BOX_Y_POS_ROW_2			20
#define ABOUT_BOX_Y_POS_ROW_3			30
#define ABOUT_BOX_Y_POS_ROW_4			40
#define ABOUT_BOX_Y_POS_ROW_5			50

/* LCD & Notifications menu items */
#define LCD_SETTINGS_ITEMS   5
#define LCD_SET_BRIGHTNESS   0
#define LCD_SET_BUZZER       1
#define LCD_SET_LED          2
#define LCD_SET_ORIENT       3
#define LCD_SET_SLEEP        4

//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/

static const uint8_t s_brightness_values[] = { 0, 64, 128, 192, 255 };
static const char *s_brightness_text[] = { "Off", "Low", "Med", "High", "Max" };
static const char *s_orient_text[] = { "Normal", "Southpaw", "Portrait" };
static const char *s_sleep_text[] = { "30s", "1 min", "5 min", "10 min", "15 min", "Never" };

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

void menu_settings_init(void);
void menu_settings_exit(void);
void settings_system(void);
void settings_about(void);
static void settings_about_display_choice(uint8_t choice);
static void settings_apply_orientation(uint8_t orient);
void settings_save_to_sd(void);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void menu_settings_init(void)
{
	;
} // void menu_settings_init(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void menu_settings_exit(void)
{
	;
} // void menu_settings_exit(void)



/* Nesting depth of active force-landscape scopes (leaf activities + forced
 * nodes). Restore only returns to the chosen system orientation at depth 0. */
static int s_orient_force_depth = 0;

/* Apply a rotation to the LIVE display only (does not change the chosen
 * system orientation). */
static void orient_set_display(uint8_t orient)
{
    m1_screen_orientation = orient;
    m1_southpaw_mode = (orient == M1_ORIENT_SOUTHPAW) ? 1 : 0;

    if (orient == M1_ORIENT_SOUTHPAW)
        u8g2_SetDisplayRotation(&m1_u8g2, U8G2_R0);
    else if (orient == M1_ORIENT_REMOTE)
        u8g2_SetDisplayRotation(&m1_u8g2, U8G2_R1);
    else
        u8g2_SetDisplayRotation(&m1_u8g2, U8G2_R2);
}

/*============================================================================*/
/**
  * @brief  Canonical set-orientation: updates the chosen system orientation AND
  *         the live display. Used at boot and for a direct set.
  */
/*============================================================================*/
static void settings_apply_orientation(uint8_t orient)
{
    m1_system_orientation = orient;
    orient_set_display(orient);
}

/* Record the user's chosen orientation. If a screen is currently forced to
 * landscape (depth > 0), the live display is left alone — the choice takes
 * effect when the force-scope unwinds (m1_orient_restore at depth 0). */
void m1_set_system_orientation(uint8_t orient)
{
    m1_system_orientation = orient;
    if (s_orient_force_depth == 0)
        orient_set_display(orient);
}

/*============================================================================*/
/**
  * @brief  Enter a force-landscape scope for a bespoke screen that can't render
  *         in 64px portrait. ONLY flips when the chosen orientation is Portrait —
  *         Southpaw (180° landscape) and Normal are left alone. Nestable.
  */
/*============================================================================*/
void m1_orient_enter_landscape(void)
{
    s_orient_force_depth++;
    if (m1_system_orientation == M1_ORIENT_PORTRAIT && m1_screen_orientation != M1_ORIENT_NORMAL)
        orient_set_display(M1_ORIENT_NORMAL);
}

/* Leave a force-landscape scope. When the outermost scope unwinds, restore the
 * display to the chosen system orientation (which may have changed meanwhile). */
void m1_orient_restore(void)
{
    if (s_orient_force_depth > 0) s_orient_force_depth--;
    if (s_orient_force_depth == 0 && m1_screen_orientation != m1_system_orientation)
        orient_set_display(m1_system_orientation);
}

/*============================================================================*/
/**
  * @brief  Orientation selector — highlight one and confirm with OK. Cycling in
  *         place is a trap: Southpaw remaps Left/Right, so a second Right press
  *         reads as Left and you can never pass it. A picker only applies on OK,
  *         so navigation never rotates and the buttons never flip mid-selection.
  */
/*============================================================================*/
static void settings_orientation_picker(void)
{
    static const char *const labels[] = { "Normal", "Southpaw", "Portrait" };
    S_M1_Buttons_Status btn;
    S_M1_Main_Q_t q_item;
    uint8_t sel = (m1_system_orientation <= 2) ? m1_system_orientation : 0;
    uint8_t redraw = 1;

    for (;;)
    {
        if (redraw) { m1_scene_draw_menu("Orientation", labels, 3, sel, 0, 3); redraw = 0; }

        if (xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY) != pdTRUE) continue;
        if (q_item.q_evt_type != Q_EVENT_KEYPAD) continue;
        if (xQueueReceive(button_events_q_hdl, &btn, 0) != pdTRUE) continue;

        if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
            return;                              /* cancel — leave orientation as-is */
        if (btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
        {
            m1_set_system_orientation(sel);      /* record choice; applied when the settings force-scope unwinds */
            return;
        }
        if (btn.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)   { sel = (sel == 0) ? 2 : (sel - 1); redraw = 1; }
        if (btn.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK) { sel = (sel + 1) % 3;               redraw = 1; }
    }
}


/*============================================================================*/
/**
  * @brief  LCD & Notifications settings — scrollable 5-item menu
  *         Brightness, Buzzer, LED Notify, Orientation, Sleep After
  */
/*============================================================================*/
void settings_lcd_and_notifications(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;

    uint8_t sel = 0;
    uint8_t needs_redraw = 1;

    while (1)
    {
        if (needs_redraw)
        {
            needs_redraw = 0;
            char line[32];

            m1_u8g2_firstpage();
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            m1_draw_text(&m1_u8g2, 2, 10, 124, "LCD & Notifications", TEXT_ALIGN_CENTER);

            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

            /* Scrollable window: show 3 items at a time */
            uint8_t visible_start = 0;
            if (sel > 1 && LCD_SETTINGS_ITEMS > 3)
                visible_start = (sel - 1 > LCD_SETTINGS_ITEMS - 3) ? LCD_SETTINGS_ITEMS - 3 : sel - 1;

            for (uint8_t vi = 0; vi < 3 && (visible_start + vi) < LCD_SETTINGS_ITEMS; vi++)
            {
                uint8_t i = visible_start + vi;
                uint8_t y = 24 + vi * 12;
                const char *label = "";
                const char *value = "";

                switch (i)
                {
                case LCD_SET_BRIGHTNESS:
                    label = "Brightness";
                    value = s_brightness_text[m1_brightness_level];
                    break;
                case LCD_SET_BUZZER:
                    label = "Buzzer";
                    value = m1_buzzer_on ? "On" : "Off";
                    break;
                case LCD_SET_LED:
                    label = "LED Notify";
                    value = m1_led_notify_on ? "On" : "Off";
                    break;
                case LCD_SET_ORIENT:
                    label = "Orientation";
                    value = s_orient_text[m1_system_orientation];
                    break;
                case LCD_SET_SLEEP:
                    label = "Sleep After";
                    value = s_sleep_text[m1_sleep_timeout_idx];
                    break;
                }

                if (i == sel)
                {
                    u8g2_DrawBox(&m1_u8g2, 0, y - 9, 128, 11);
                    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
                    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
                    snprintf(line, sizeof(line), "< %s: %s >", label, value);
                    u8g2_DrawStr(&m1_u8g2, 4, y, line);
                    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
                    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
                }
                else
                {
                    snprintf(line, sizeof(line), "  %s: %s", label, value);
                    u8g2_DrawStr(&m1_u8g2, 4, y, line);
                }
            }

            /* Bottom bar */
            u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 61, "U/D=Sel L/R=Change");
            m1_u8g2_nextpage();
        }

        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret != pdTRUE) continue;
        if (q_item.q_evt_type != Q_EVENT_KEYPAD) continue;

        ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
        if (ret != pdTRUE) continue;

        /* Back — save and exit */
        if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
        {
            settings_save_to_sd();
            xQueueReset(main_q_hdl);
            break;
        }

        /* OK — Orientation opens a highlight-to-select picker (no cycle trap) */
        if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (sel == LCD_SET_ORIENT)
                settings_orientation_picker();
            needs_redraw = 1;
        }

        /* Up/Down — navigate */
        if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
        {
            sel = (sel == 0) ? (LCD_SETTINGS_ITEMS - 1) : (sel - 1);
            needs_redraw = 1;
        }
        if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
        {
            sel = (sel + 1) % LCD_SETTINGS_ITEMS;
            needs_redraw = 1;
        }

        /* Left — decrement selected setting */
        if (this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (sel == LCD_SET_BRIGHTNESS)
            {
                m1_brightness_level = (m1_brightness_level == 0) ? 4 : (m1_brightness_level - 1);
                lp5814_backlight_on(s_brightness_values[m1_brightness_level]);
            }
            else if (sel == LCD_SET_BUZZER)
                m1_buzzer_on = !m1_buzzer_on;
            else if (sel == LCD_SET_LED)
                m1_led_notify_on = !m1_led_notify_on;
            else if (sel == LCD_SET_ORIENT)
                settings_orientation_picker();   /* highlight-to-select, not cycle */
            else if (sel == LCD_SET_SLEEP)
                m1_sleep_timeout_idx = (m1_sleep_timeout_idx == 0) ? 5 : (m1_sleep_timeout_idx - 1);
            needs_redraw = 1;
        }

        /* Right — increment selected setting */
        if (this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (sel == LCD_SET_BRIGHTNESS)
            {
                m1_brightness_level = (m1_brightness_level >= 4) ? 0 : (m1_brightness_level + 1);
                lp5814_backlight_on(s_brightness_values[m1_brightness_level]);
            }
            else if (sel == LCD_SET_BUZZER)
            {
                m1_buzzer_on = !m1_buzzer_on;
                if (m1_buzzer_on) m1_buzzer_notification();
            }
            else if (sel == LCD_SET_LED)
                m1_led_notify_on = !m1_led_notify_on;
            else if (sel == LCD_SET_ORIENT)
                settings_orientation_picker();   /* highlight-to-select, not cycle */
            else if (sel == LCD_SET_SLEEP)
                m1_sleep_timeout_idx = (m1_sleep_timeout_idx >= 5) ? 0 : (m1_sleep_timeout_idx + 1);
            needs_redraw = 1;
        }
    }
}



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void settings_buzzer(void)
{
	//buzzer_demo_play();
} // void settings_sound(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void settings_power(void)
{
	;
} // void settings_power(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void settings_system_draw(uint8_t sel)
{
    static const char *items[5] = { "ESP32 at boot", "WiFi on boot", "Bluetooth Direct",
                                    "Initialize ESP32", "Reboot ESP32" };
    const uint8_t SYS_SETTINGS_ITEMS = 5;

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);

    m1_draw_text(&m1_u8g2, 2, 10, 124, "System Settings", TEXT_ALIGN_CENTER);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    /* Scrollable window: show 3 of the items at a time. */
    uint8_t visible_start = 0;
    if (sel > 1 && SYS_SETTINGS_ITEMS > 3)
        visible_start = (sel - 1 > SYS_SETTINGS_ITEMS - 3) ? SYS_SETTINGS_ITEMS - 3 : sel - 1;

    for (uint8_t vi = 0; vi < 3 && (visible_start + vi) < SYS_SETTINGS_ITEMS; vi++)
    {
        uint8_t i = visible_start + vi;
        int y = 24 + vi * 12;
        if (i == sel)
        {
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_DrawBox(&m1_u8g2, 0, y - 9, 128, 12);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        }
        m1_draw_text(&m1_u8g2, 4, y, 96, items[i], TEXT_ALIGN_LEFT);
        if (i == 0)
            m1_draw_text(&m1_u8g2, 96, y, 30, m1_esp32_auto_init ? "ON" : "OFF", TEXT_ALIGN_LEFT);
        else if (i == 1)
            m1_draw_text(&m1_u8g2, 96, y, 30, m1_wifi_boot_connect ? "ON" : "OFF", TEXT_ALIGN_LEFT);
        else if (i == 2)
            m1_draw_text(&m1_u8g2, 96, y, 30, m1_ble_direct ? "ON" : "OFF", TEXT_ALIGN_LEFT);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    }

    m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "Select", arrowright_8x8);
    m1_u8g2_nextpage();
}

void settings_system(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    uint8_t sel = 0;

    settings_system_draw(sel);

    while (1)
    {
        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret != pdTRUE)
            continue;

        if (q_item.q_evt_type != Q_EVENT_KEYPAD)
            continue;

        ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
        if (ret != pdTRUE)
            continue;

        if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
        {
            xQueueReset(main_q_hdl);
            break;
        }
        else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (sel > 0) sel--;
            settings_system_draw(sel);
        }
        else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (sel < 4) sel++;
            settings_system_draw(sel);
        }
        else if ((sel == 0 || sel == 1 || sel == 2) &&
                 (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK ||
                  this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK ||
                  this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK))
        {
            if (sel == 0)
                /* ESP32 at boot: toggle ON/OFF */
                m1_esp32_auto_init = m1_esp32_auto_init ? 0 : 1;
            else if (sel == 1)
                /* Connect to WiFi (primary network) at boot: toggle ON/OFF */
                m1_wifi_boot_connect = m1_wifi_boot_connect ? 0 : 1;
            else
            {
                /* Bluetooth Direct: toggle + tell the ESP to start/stop NUS
                 * advertising now (best-effort; re-applied at boot too). */
                m1_ble_direct = m1_ble_direct ? 0 : 1;
                if (m1_esp32_get_init_status())
                    m1_esp_client_ble_direct(m1_ble_direct ? true : false, m1_bt_direct_name);
            }
            settings_save_to_sd();
            settings_system_draw(sel);
        }
        else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (sel == 3)
            {
                /* Initialize ESP32 (brings up the link if not already up) */
                if (!m1_esp32_get_init_status())
                {
                    m1_esp32_init();
                    m1_message_box(&m1_u8g2, "ESP32", "Initialized", "", " OK ");
                }
                else
                {
                    m1_message_box(&m1_u8g2, "ESP32", "Already", "initialized", " OK ");
                }
            }
            else if (sel == 4)
            {
                /* Reboot ESP32 only (M1 keeps running) */
                m1_esp32_reboot();
                m1_message_box(&m1_u8g2, "ESP32", "Rebooted", "", " OK ");
            }
            xQueueReset(main_q_hdl);
            settings_system_draw(sel);
        }
    }
}



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void settings_about(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t choice;

	/* Graphic work starts here */
	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	u8g2_DrawXBMP(&m1_u8g2, 1, 53, 8, 8, arrowleft_8x8); // draw arrowleft icon
	u8g2_DrawStr(&m1_u8g2, 11, 61, "Prev.");
	u8g2_DrawXBMP(&m1_u8g2, 119, 53, 8, 8, arrowright_8x8); // draw arrowright icon
	u8g2_DrawStr(&m1_u8g2, 97, 61, "Next");
	m1_u8g2_nextpage(); // Update display RAM

	choice = 0;
	settings_about_display_choice(choice);

	while (1 ) // Main loop of this task
	{
		;
		; // Do other parts of this task here
		;

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				// Notification is only sent to this task when there's any button activity,
				// so it doesn't need to wait when reading the event from the queue
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // user wants to exit?
				{
					; // Do extra tasks here if needed
					xQueueReset(main_q_hdl); // Reset main q before return
					break; // Exit and return to the calling task (subfunc_handler_task)
				} // if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				else if ( this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK ) // Previous?
				{
					choice--;
					if ( choice > SETTING_ABOUT_CHOICES_MAX )
						choice = SETTING_ABOUT_CHOICES_MAX;
					settings_about_display_choice(choice);
				} // else if ( this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )
				else if ( this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK ) // Next?
				{
					choice++;
					if ( choice > SETTING_ABOUT_CHOICES_MAX )
						choice = 0;
					settings_about_display_choice(choice);
				} // else if ( this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK )
			} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			else
			{
				; // Do other things for this task
			}
		} // if (ret==pdTRUE)
	} // while (1 ) // Main loop of this task

} // void settings_about(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void settings_about_display_choice(uint8_t choice)
{
	uint8_t prn_name[20];

	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Set background color
	u8g2_DrawBox(&m1_u8g2, 0, 0, M1_LCD_DISPLAY_WIDTH, ABOUT_BOX_Y_POS_ROW_5 + 1); // Clear old content
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // Set text color

	switch (choice)
	{
		case 0: // FW info
			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B); // Set bold font
			u8g2_DrawStr(&m1_u8g2, 0, ABOUT_BOX_Y_POS_ROW_1, "M1 by C3");
			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N); // Set normal font
			sprintf(prn_name, "%d.%d.%d.%d-C3.%d", m1_device_stat.config.fw_version_major, m1_device_stat.config.fw_version_minor, m1_device_stat.config.fw_version_build, m1_device_stat.config.fw_version_rc, M1_C3_REVISION);
			u8g2_DrawStr(&m1_u8g2, 0, ABOUT_BOX_Y_POS_ROW_2, prn_name);
			sprintf(prn_name, "Active bank: %d", (m1_device_stat.active_bank==BANK1_ACTIVE)?1:2);
			u8g2_DrawStr(&m1_u8g2, 0, ABOUT_BOX_Y_POS_ROW_3, prn_name);
			/* ESP32-C6 coprocessor firmware — same cached source qMonstatek uses,
			 * so the version is visible on-device without the desktop app. */
			{
				char esp_ver[32];
				char esp_line[40];
				m1_rpc_esp_fw_str(esp_ver, sizeof(esp_ver));
				snprintf(esp_line, sizeof(esp_line), "ESP: %s", esp_ver);
				u8g2_DrawStr(&m1_u8g2, 0, ABOUT_BOX_Y_POS_ROW_4, esp_line);
			}
			break;

		case 1: // Company info
			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N); // Set small font
			u8g2_DrawStr(&m1_u8g2, 0, ABOUT_BOX_Y_POS_ROW_1, "MonstaTek Inc.");
			u8g2_DrawStr(&m1_u8g2, 0, ABOUT_BOX_Y_POS_ROW_2, "San Jose, CA, USA");
			break;

		default:
			u8g2_DrawXBMP(&m1_u8g2, 23, 1, 82, 36, m1_device_82x36);
			break;
	} // switch (choice)

	m1_u8g2_nextpage(); // Update display RAM
} // static void settings_about_display_choice(uint8_t choice)


/*============================================================================*/
/**
  * @brief  Save user settings to SD card (0:/System/settings.cfg)
  */
/*============================================================================*/
void settings_save_to_sd(void)
{
    FIL fp;
    FRESULT fres;
    UINT bw;
    char buf[96];   /* wide enough for hotspot_pass=<up to 64 chars> */

    /* Ensure System directory exists */
    f_mkdir("0:/System");

    fres = f_open(&fp, SETTINGS_FILE_PATH, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK)
    {
        M1_LOG_W(SETTINGS_TAG, "Save failed (err=%d)\r\n", fres);
        return;
    }

    snprintf(buf, sizeof(buf), "brightness=%d\n", m1_brightness_level);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "buzzer=%d\n", m1_buzzer_on);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "led_notify=%d\n", m1_led_notify_on);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "orientation=%d\n", m1_system_orientation);   /* persist the chosen orientation, not a temp-forced one */
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "sleep_timeout=%d\n", m1_sleep_timeout_idx);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "esp32_auto_init=%d\n", m1_esp32_auto_init);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "ism_region=%d\n", m1_device_stat.config.ism_band_region);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "wifi_boot_connect=%d\n", m1_wifi_boot_connect);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "wifi_primary=%s\n", m1_wifi_primary_ssid);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "ble_direct=%d\n", m1_ble_direct);
    f_write(&fp, buf, strlen(buf), &bw);
    snprintf(buf, sizeof(buf), "bt_direct_name=%s\n", m1_bt_direct_name);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "hotspot_on=%d\n", m1_hotspot_on);
    f_write(&fp, buf, strlen(buf), &bw);
    snprintf(buf, sizeof(buf), "hotspot_ssid=%s\n", m1_hotspot_ssid);
    f_write(&fp, buf, strlen(buf), &bw);
    snprintf(buf, sizeof(buf), "hotspot_pass=%s\n", m1_hotspot_pass);
    f_write(&fp, buf, strlen(buf), &bw);

#ifdef M1_APP_BADBT_ENABLE
    snprintf(buf, sizeof(buf), "badbt_name=%s\n", m1_badbt_name);
    f_write(&fp, buf, strlen(buf), &bw);
#endif

    f_close(&fp);
}


/*============================================================================*/
/**
  * @brief  Load user settings from SD card (0:/System/settings.cfg)
  *         Sets m1_southpaw_mode and applies display rotation.
  */
/*============================================================================*/
void settings_load_from_sd(void)
{
    FIL fp;
    FRESULT fres;
    UINT br;
    char buf[SETTINGS_FILE_MAX_SIZE];
    char *p;
    int val;

    fres = f_open(&fp, SETTINGS_FILE_PATH, FA_READ);
    if (fres != FR_OK)
        goto apply;  /* No settings file yet — apply defaults */

    fres = f_read(&fp, buf, sizeof(buf) - 1, &br);
    f_close(&fp);

    if (fres != FR_OK || br == 0)
        goto apply;

    buf[br] = '\0';

    /* Parse "brightness=X" */
    p = strstr(buf, "brightness=");
    if (p != NULL)
    {
        val = (int)(*(p + 11) - '0');
        if (val >= 0 && val <= 4)
            m1_brightness_level = (uint8_t)val;
    }

    /* Parse "buzzer=X" */
    p = strstr(buf, "buzzer=");
    if (p != NULL)
    {
        val = (int)(*(p + 7) - '0');
        if (val == 0 || val == 1)
            m1_buzzer_on = (uint8_t)val;
    }

    /* Parse "led_notify=X" */
    p = strstr(buf, "led_notify=");
    if (p != NULL)
    {
        val = (int)(*(p + 11) - '0');
        if (val == 0 || val == 1)
            m1_led_notify_on = (uint8_t)val;
    }

    /* Parse "orientation=X" */
    p = strstr(buf, "orientation=");
    if (p != NULL)
    {
        val = (int)(*(p + 12) - '0');
        if (val >= 0 && val <= 2)
            m1_screen_orientation = (uint8_t)val;
    }

    /* Parse "sleep_timeout=X" */
    p = strstr(buf, "sleep_timeout=");
    if (p != NULL)
    {
        val = (int)(*(p + 14) - '0');
        if (val >= 0 && val <= 5)
            m1_sleep_timeout_idx = (uint8_t)val;
    }

    /* Parse "esp32_auto_init=X" */
    p = strstr(buf, "esp32_auto_init=");
    if (p != NULL)
    {
        val = (int)(*(p + 16) - '0');
        if (val == 0 || val == 1)
            m1_esp32_auto_init = (uint8_t)val;
    }

    /* Parse "ism_region=X" */
    p = strstr(buf, "ism_region=");
    if (p != NULL)
    {
        val = (int)(*(p + 11) - '0');
        if (val >= 0 && val <= 3)
            m1_device_stat.config.ism_band_region = (uint8_t)val;
    }

    /* Parse "wifi_boot_connect=X" */
    p = strstr(buf, "wifi_boot_connect=");
    if (p != NULL)
    {
        val = (int)(*(p + 18) - '0');
        if (val == 0 || val == 1)
            m1_wifi_boot_connect = (uint8_t)val;
    }

    /* Parse "ble_direct=X" */
    p = strstr(buf, "ble_direct=");
    if (p != NULL)
    {
        val = (int)(*(p + 11) - '0');
        if (val == 0 || val == 1)
            m1_ble_direct = (uint8_t)val;
    }

    /* Parse "bt_direct_name=XYZ" (Bluetooth Direct advertised name) */
    p = strstr(buf, "bt_direct_name=");
    if (p != NULL)
    {
        p += 15;  /* skip "bt_direct_name=" */
        char *end = strchr(p, '\n');
        if (!end) end = p + strlen(p);
        uint8_t len = end - p;
        if (len > BT_DIRECT_NAME_MAX_LEN) len = BT_DIRECT_NAME_MAX_LEN;
        if (len > 0)
        {
            memcpy(m1_bt_direct_name, p, len);
            m1_bt_direct_name[len] = '\0';
        }
    }

    /* Parse "hotspot_on=X" */
    p = strstr(buf, "hotspot_on=");
    if (p != NULL)
    {
        val = (int)(*(p + 11) - '0');
        if (val == 0 || val == 1)
            m1_hotspot_on = (uint8_t)val;
    }

    /* Parse "hotspot_ssid=..." (to end-of-line) */
    p = strstr(buf, "hotspot_ssid=");
    if (p != NULL)
    {
        p += 13;
        char *end = strchr(p, '\n');
        if (!end) end = p + strlen(p);
        uint16_t len = (uint16_t)(end - p);
        if (len >= WIFI_CRED_SSID_MAX_LEN) len = WIFI_CRED_SSID_MAX_LEN - 1;
        if (len > 0) { memcpy(m1_hotspot_ssid, p, len); m1_hotspot_ssid[len] = '\0'; }
    }

    /* Parse "hotspot_pass=..." (to end-of-line; may be empty = open) */
    p = strstr(buf, "hotspot_pass=");
    if (p != NULL)
    {
        p += 13;
        char *end = strchr(p, '\n');
        if (!end) end = p + strlen(p);
        uint16_t len = (uint16_t)(end - p);
        if (len >= WIFI_CRED_PASS_MAX_LEN) len = WIFI_CRED_PASS_MAX_LEN - 1;
        memcpy(m1_hotspot_pass, p, len); m1_hotspot_pass[len] = '\0';
    }

    /* Parse "wifi_primary=SSID" (SSID up to end-of-line) */
    p = strstr(buf, "wifi_primary=");
    if (p != NULL)
    {
        p += 13;  /* skip "wifi_primary=" */
        char *end = strchr(p, '\n');
        if (!end) end = p + strlen(p);
        uint16_t len = (uint16_t)(end - p);
        if (len >= WIFI_CRED_SSID_MAX_LEN) len = WIFI_CRED_SSID_MAX_LEN - 1;
        memcpy(m1_wifi_primary_ssid, p, len);
        m1_wifi_primary_ssid[len] = '\0';
    }

    /* Legacy: migrate "southpaw=1" if no orientation key found */
    if (strstr(buf, "orientation=") == NULL)
    {
        p = strstr(buf, "southpaw=");
        if (p != NULL && *(p + 9) == '1')
            m1_screen_orientation = M1_ORIENT_SOUTHPAW;
    }

#ifdef M1_APP_BADBT_ENABLE
    /* Parse "badbt_name=XYZ" */
    p = strstr(buf, "badbt_name=");
    if (p != NULL)
    {
        p += 11;  /* skip "badbt_name=" */
        char *end = strchr(p, '\n');
        if (!end) end = p + strlen(p);
        uint8_t len = end - p;
        if (len > BADBT_NAME_MAX_LEN) len = BADBT_NAME_MAX_LEN;
        if (len > 0)
        {
            memcpy(m1_badbt_name, p, len);
            m1_badbt_name[len] = '\0';
        }
    }
#endif

apply:
    /* Apply brightness */
    lp5814_backlight_on(s_brightness_values[m1_brightness_level]);

    /* Apply orientation */
    settings_apply_orientation(m1_screen_orientation);
}


/*============================================================================*/
/**
  * @brief  Ensure all module SD card directories exist
  *         Call once after SD card is mounted (e.g. after settings_load_from_sd)
  */
/*============================================================================*/
void settings_ensure_sd_folders(void)
{
    static const char * const dirs[] = {
        "0:/NFC",
        "0:/RFID",
        "0:/SUBGHZ",
        "0:/IR",
        "0:/BadUSB",
        "0:/BT",
        "0:/System",
        "0:/apps"
    };

    for (uint8_t i = 0; i < sizeof(dirs) / sizeof(dirs[0]); i++)
    {
        fs_directory_ensure(dirs[i]);
    }
}
