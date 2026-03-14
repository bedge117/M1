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
#include "m1_buzzer.h"
#include "m1_lcd.h"
#include "m1_display.h"
#include "ff.h"
#include "m1_log_debug.h"
#include "m1_fw_update_bl.h"

/*************************** D E F I N E S ************************************/

#define SETTINGS_TAG              "SETT"
#define SETTINGS_FILE_PATH        "0:/System/settings.cfg"
#define SETTINGS_FILE_MAX_SIZE    256

#define SETTING_ABOUT_CHOICES_MAX		2 //5

#define ABOUT_BOX_Y_POS_ROW_1			10
#define ABOUT_BOX_Y_POS_ROW_2			20
#define ABOUT_BOX_Y_POS_ROW_3			30
#define ABOUT_BOX_Y_POS_ROW_4			40
#define ABOUT_BOX_Y_POS_ROW_5			50

//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

void menu_settings_init(void);
void menu_settings_exit(void);
void settings_system(void);
void settings_about(void);
static void settings_about_display_choice(uint8_t choice);
static void settings_lcd_draw(void);
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



/*============================================================================*/
/**
  * @brief  Draw the LCD & Notifications settings screen
  */
/*============================================================================*/
static void settings_lcd_draw(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);

    m1_draw_text(&m1_u8g2, 2, 10, 124, "LCD Settings", TEXT_ALIGN_CENTER);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 4, 28, 60, "Southpaw:", TEXT_ALIGN_LEFT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    m1_draw_text(&m1_u8g2, 66, 28, 58, m1_southpaw_mode ? "ON" : "OFF", TEXT_ALIGN_LEFT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 4, 42, 120, "Left-hand flip", TEXT_ALIGN_CENTER);

    m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "Toggle", arrowright_8x8);
    m1_u8g2_nextpage();
}


/*============================================================================*/
/**
  * @brief  LCD & Notifications settings — southpaw toggle
  */
/*============================================================================*/
void settings_lcd_and_notifications(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;

    settings_lcd_draw();

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
        else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK ||
                 this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK ||
                 this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            /* Toggle southpaw mode */
            m1_southpaw_mode = m1_southpaw_mode ? 0 : 1;
            m1_lcd_set_southpaw(m1_southpaw_mode);
            settings_save_to_sd();
            settings_lcd_draw();
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
static void settings_system_draw(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);

    m1_draw_text(&m1_u8g2, 2, 10, 124, "System Settings", TEXT_ALIGN_CENTER);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 4, 28, 72, "ESP32 at boot:", TEXT_ALIGN_LEFT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
    m1_draw_text(&m1_u8g2, 78, 28, 46, m1_esp32_auto_init ? "ON" : "OFF", TEXT_ALIGN_LEFT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 4, 42, 120, "Init WiFi/BT on boot", TEXT_ALIGN_CENTER);

    m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "Toggle", arrowright_8x8);
    m1_u8g2_nextpage();
}

void settings_system(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;

    settings_system_draw();

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
        else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK ||
                 this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK ||
                 this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            m1_esp32_auto_init = m1_esp32_auto_init ? 0 : 1;
            settings_save_to_sd();
            settings_system_draw();
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
    char buf[64];

    /* Ensure System directory exists */
    f_mkdir("0:/System");

    fres = f_open(&fp, SETTINGS_FILE_PATH, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK)
    {
        M1_LOG_W(SETTINGS_TAG, "Save failed (err=%d)\r\n", fres);
        return;
    }

    snprintf(buf, sizeof(buf), "southpaw=%d\n", m1_southpaw_mode);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "esp32_auto_init=%d\n", m1_esp32_auto_init);
    f_write(&fp, buf, strlen(buf), &bw);

    snprintf(buf, sizeof(buf), "badbt_name=%s\n", m1_badbt_name);
    f_write(&fp, buf, strlen(buf), &bw);

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

    fres = f_open(&fp, SETTINGS_FILE_PATH, FA_READ);
    if (fres != FR_OK)
        return;  /* No settings file yet — use defaults */

    fres = f_read(&fp, buf, sizeof(buf) - 1, &br);
    f_close(&fp);

    if (fres != FR_OK || br == 0)
        return;

    buf[br] = '\0';

    /* Parse "southpaw=X" */
    char *p = strstr(buf, "southpaw=");
    if (p != NULL)
    {
        p += 9;  /* skip "southpaw=" */
        uint8_t val = (uint8_t)(*p - '0');
        if (val <= 1)
        {
            m1_southpaw_mode = val;
            m1_lcd_set_southpaw(m1_southpaw_mode);
        }
    }

    /* Parse "esp32_auto_init=X" */
    p = strstr(buf, "esp32_auto_init=");
    if (p != NULL)
    {
        p += 16;  /* skip "esp32_auto_init=" */
        uint8_t val = (uint8_t)(*p - '0');
        if (val <= 1)
        {
            m1_esp32_auto_init = val;
        }
    }

    /* Parse "badbt_name=<name>" */
    p = strstr(buf, "badbt_name=");
    if (p != NULL)
    {
        p += 11;  /* skip "badbt_name=" */
        char *end = strchr(p, '\n');
        size_t len = end ? (size_t)(end - p) : strlen(p);
        if (len > BADBT_NAME_MAX_LEN) len = BADBT_NAME_MAX_LEN;
        if (len > 0)
        {
            memcpy(m1_badbt_name, p, len);
            m1_badbt_name[len] = '\0';
        }
    }
}
