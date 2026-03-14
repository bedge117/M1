/*
*
* m1_bt.c
*
* Source for M1 bluetooth
*
* M1 Project
*
*/
/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/********************************************************************************
*
* This file is for BLE management: scan, saved devices, connect, info.
*
*********************************************************************************/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_bt.h"
#include "m1_esp32_hal.h"
#include "spi_master.h"
#include "esp_app_main.h"
#include "esp_at_list.h"
#include "m1_compile_cfg.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_virtual_kb.h"
#include "m1_settings.h"
#include "m1_system.h"

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG				"BLE"

#define M1_GUI_ROW_SPACING			1

#define M1_BLE_SCANNING_TIME		10 // seconds

#ifdef M1_APP_BT_MANAGE_ENABLE

#define BT_DEVICES_DIR				"0:/BT"
#define BT_DEVICES_FILE				"0:/BT/devices.txt"
#define BT_MAX_SAVED				20

#define BT_LIST_ITEM_HEIGHT			9
#define BT_LIST_START_Y				13
#define BT_LIST_VISIBLE				4

static const uint8_t blank_8x8[8] = {0};

#endif /* M1_APP_BT_MANAGE_ENABLE */


//************************** S T R U C T U R E S *******************************

#ifdef M1_APP_BT_MANAGE_ENABLE
typedef struct {
	char addr[BSSID_STR_SIZE];
	char name[SSID_LENGTH];
	uint8_t addr_type;
} bt_saved_device_t;
#endif

/***************************** V A R I A B L E S ******************************/

#ifdef M1_APP_BT_MANAGE_ENABLE
static bt_saved_device_t s_saved_devices[BT_MAX_SAVED];
static uint8_t s_saved_count = 0;
static bt_connection_state_t s_bt_conn_state = {0};
#endif


/********************* F U N C T I O N   P R O T O T Y P E S ******************/

void menu_bluetooth_init(void);
void bluetooth_config(void);
void bluetooth_scan(void);
void bluetooth_advertise(void);

#ifndef M1_APP_BT_MANAGE_ENABLE
static uint8_t ble_scan_list_validation(ctrl_cmd_t *app_resp);
static uint16_t ble_scan_list_print(ctrl_cmd_t *app_resp, bool up_dir);
#endif

extern void  esp32_main_init(void);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/*
 * This function initializes display for this sub-menu item.
 */
/*============================================================================*/
void menu_bluetooth_init(void)
{
	;
} // void menu_bluetooth_init(void)


#ifdef M1_APP_BT_MANAGE_ENABLE

/******************************************************************************/
/*                      S A V E D   D E V I C E S                             */
/******************************************************************************/

static void bt_load_devices(void)
{
	FIL fil;
	FRESULT res;
	char line[80];
	char *p, *p2;

	s_saved_count = 0;
	f_mkdir(BT_DEVICES_DIR);

	res = f_open(&fil, BT_DEVICES_FILE, FA_READ);
	if (res != FR_OK) return;

	while (s_saved_count < BT_MAX_SAVED)
	{
		if (f_gets(line, sizeof(line), &fil) == NULL) break;

		/* Strip trailing newline */
		p = strstr(line, "\r");
		if (p) *p = '\0';
		p = strstr(line, "\n");
		if (p) *p = '\0';

		if (strlen(line) < 5) continue; /* skip empty/short lines */

		/* Parse: MAC,Name,AddrType */
		p = strchr(line, ',');
		if (!p) continue;
		*p = '\0';

		strncpy(s_saved_devices[s_saved_count].addr, line, BSSID_STR_SIZE - 1);
		s_saved_devices[s_saved_count].addr[BSSID_STR_SIZE - 1] = '\0';

		p++; /* start of name */
		p2 = strchr(p, ',');
		if (p2)
		{
			*p2 = '\0';
			strncpy(s_saved_devices[s_saved_count].name, p, SSID_LENGTH - 1);
			s_saved_devices[s_saved_count].name[SSID_LENGTH - 1] = '\0';
			s_saved_devices[s_saved_count].addr_type = (uint8_t)strtol(p2 + 1, NULL, 10);
		}
		else
		{
			strncpy(s_saved_devices[s_saved_count].name, p, SSID_LENGTH - 1);
			s_saved_devices[s_saved_count].name[SSID_LENGTH - 1] = '\0';
			s_saved_devices[s_saved_count].addr_type = 0;
		}

		s_saved_count++;
	}

	f_close(&fil);
} // static void bt_load_devices(void)


static bool bt_save_devices(void)
{
	FIL fil;
	FRESULT res;
	char line[80];
	uint8_t i;

	f_mkdir(BT_DEVICES_DIR);
	res = f_open(&fil, BT_DEVICES_FILE, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK) return false;

	for (i = 0; i < s_saved_count; i++)
	{
		snprintf(line, sizeof(line), "%s,%s,%u\n",
			s_saved_devices[i].addr,
			s_saved_devices[i].name,
			s_saved_devices[i].addr_type);
		f_puts(line, &fil);
	}

	f_close(&fil);
	return true;
} // static bool bt_save_devices(void)


static bool bt_add_device(const char *addr, const char *name, uint8_t addr_type)
{
	uint8_t i;

	/* Check if already exists by MAC */
	for (i = 0; i < s_saved_count; i++)
	{
		if (strcmp(s_saved_devices[i].addr, addr) == 0)
		{
			/* Update name if provided */
			if (name && name[0])
				strncpy(s_saved_devices[i].name, name, SSID_LENGTH - 1);
			s_saved_devices[i].addr_type = addr_type;
			return bt_save_devices();
		}
	}

	if (s_saved_count >= BT_MAX_SAVED) return false;

	strncpy(s_saved_devices[s_saved_count].addr, addr, BSSID_STR_SIZE - 1);
	s_saved_devices[s_saved_count].addr[BSSID_STR_SIZE - 1] = '\0';
	if (name && name[0])
		strncpy(s_saved_devices[s_saved_count].name, name, SSID_LENGTH - 1);
	else
		s_saved_devices[s_saved_count].name[0] = '\0';
	s_saved_devices[s_saved_count].name[SSID_LENGTH - 1] = '\0';
	s_saved_devices[s_saved_count].addr_type = addr_type;
	s_saved_count++;

	return bt_save_devices();
} // static bool bt_add_device(...)


static bool bt_remove_device(uint8_t index)
{
	if (index >= s_saved_count) return false;

	/* Shift remaining devices down */
	for (uint8_t i = index; i < s_saved_count - 1; i++)
	{
		s_saved_devices[i] = s_saved_devices[i + 1];
	}
	s_saved_count--;

	return bt_save_devices();
} // static bool bt_remove_device(uint8_t index)


/******************************************************************************/
/*               D I S P L A Y   H E L P E R S                               */
/******************************************************************************/

static void bt_draw_title_bar(const char *title)
{
	u8g2_DrawXBMP(&m1_u8g2, 0, 0, 128, 14, m1_frame_128_14);
	u8g2_DrawStr(&m1_u8g2, 2, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, title);
}


static void bt_draw_list_item(uint8_t vis_idx, const char *text, bool selected)
{
	uint8_t y = BT_LIST_START_Y + vis_idx * BT_LIST_ITEM_HEIGHT;

	if (selected)
	{
		u8g2_SetDrawColor(&m1_u8g2, 1);
		u8g2_DrawBox(&m1_u8g2, 0, y, 128, BT_LIST_ITEM_HEIGHT);
		u8g2_SetDrawColor(&m1_u8g2, 0);
	}

	/* Truncate text to fit screen (20 chars at 6px font) */
	char buf[22];
	strncpy(buf, text, 21);
	buf[21] = '\0';
	u8g2_DrawStr(&m1_u8g2, 2, y + 8, buf);

	if (selected)
		u8g2_SetDrawColor(&m1_u8g2, 1);
}


static void bt_show_message(const char *line1, const char *line2, uint16_t delay_ms)
{
	m1_u8g2_firstpage();
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	bt_draw_title_bar("Bluetooth");
	if (line1)
		u8g2_DrawStr(&m1_u8g2, 2, 28, line1);
	if (line2)
		u8g2_DrawStr(&m1_u8g2, 2, 40, line2);
	m1_u8g2_nextpage();
	if (delay_ms)
		vTaskDelay(pdMS_TO_TICKS(delay_ms));
}


/******************************************************************************/
/*               B L E   S C A N   ( E N H A N C E D )                       */
/******************************************************************************/

static void bt_scan_detail_screen(ble_scanlist_t *dev)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	char prn_msg[25];
	bool redraw = true;

	while (1)
	{
		if (redraw)
		{
			redraw = false;
			m1_u8g2_firstpage();
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			bt_draw_title_bar("Device Info");

			uint8_t y = 24;
			/* Name */
			if (dev->name[0])
			{
				char nbuf[22];
				strncpy(nbuf, (char *)dev->name, 21);
				nbuf[21] = '\0';
				u8g2_DrawStr(&m1_u8g2, 2, y, nbuf);
			}
			else
				u8g2_DrawStr(&m1_u8g2, 2, y, "(No name)");
			y += 10;

			/* MAC */
			u8g2_DrawStr(&m1_u8g2, 2, y, (char *)dev->addr);
			y += 10;

			/* RSSI + addr_type */
			snprintf(prn_msg, sizeof(prn_msg), "RSSI:%ddBm Type:%u", dev->rssi, dev->addr_type);
			u8g2_DrawStr(&m1_u8g2, 2, y, prn_msg);

			m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "Save", arrowright_8x8);
			m1_u8g2_nextpage();
		}

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				break;
			}
			else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				/* Save device */
				bool saved = bt_add_device((char *)dev->addr, (char *)dev->name, dev->addr_type);
				bt_show_message(saved ? "Saved!" : "Save failed!", NULL, 1500);
				break;
			}
		}
	}
} // static void bt_scan_detail_screen(...)


void bluetooth_scan(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	ctrl_cmd_t app_req = CTRL_CMD_DEFAULT_REQ();
	int16_t selection = 0;
	int16_t scroll_offset = 0;
	int16_t count = 0;
	ble_scanlist_t *list = NULL;
	bool redraw = true;
	bool scan_done = false;
	char page_info[16];

	/* Init ESP32 if needed */
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	if (!m1_esp32_get_init_status())
	{
		m1_esp32_init();
	}
	if (!get_esp32_main_init_status())
	{
		m1_u8g2_firstpage();
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		bt_draw_title_bar("Bluetooth");
		u8g2_DrawStr(&m1_u8g2, 6, 28, "Initializing...");
		u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, 32, 18, 32, hourglass_18x32);
		m1_u8g2_nextpage();
		esp32_main_init();
	}

	/* Show scanning animation */
	m1_u8g2_firstpage();
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	bt_draw_title_bar("Bluetooth");
	u8g2_DrawStr(&m1_u8g2, 6, 28, "Scanning BLE...");
	u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, 32, 18, 32, hourglass_18x32);
	m1_u8g2_nextpage();

	if (get_esp32_main_init_status())
	{
		app_req.cmd_timeout_sec = M1_BLE_SCANNING_TIME;
		app_req.msg_id = CTRL_RESP_GET_BLE_SCAN_LIST;
		ret = ble_scan_list_ex(&app_req);

		if (ret == SUCCESS && app_req.msg_type == CTRL_RESP &&
			app_req.resp_event_status == SUCCESS)
		{
			count = app_req.u.ble_scan.count;
			list = app_req.u.ble_scan.out_list;
			scan_done = true;
		}
		else
		{
			bt_show_message("Scan failed!", "Press Back", 0);
		}
	}
	else
	{
		bt_show_message("ESP32 not ready!", "Press Back", 0);
	}

	if (!scan_done || count == 0)
	{
		if (scan_done)
			bt_show_message("No devices found", "Press Back", 0);

		/* Wait for BACK to exit */
		while (1)
		{
			ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
			if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
			{
				xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
					break;
			}
		}
		if (list) free(list);
		xQueueReset(main_q_hdl);
		m1_esp32_deinit();
		return;
	}

	/* Display scrollable list */
	while (1)
	{
		if (redraw)
		{
			redraw = false;
			m1_u8g2_firstpage();
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

			snprintf(page_info, sizeof(page_info), "BLE Scan (%d)", count);
			bt_draw_title_bar(page_info);

			/* Adjust scroll offset */
			if (selection < scroll_offset) scroll_offset = selection;
			if (selection >= scroll_offset + BT_LIST_VISIBLE)
				scroll_offset = selection - BT_LIST_VISIBLE + 1;

			/* Draw visible items */
			for (int i = 0; i < BT_LIST_VISIBLE && scroll_offset + i < count; i++)
			{
				int idx = scroll_offset + i;
				const char *display_name;
				if (list[idx].name[0])
					display_name = (const char *)list[idx].name;
				else
					display_name = (const char *)list[idx].addr;

				bt_draw_list_item(i, display_name, (idx == selection));
			}

			snprintf(page_info, sizeof(page_info), "%d/%d", selection + 1, count);
			m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, page_info, "Info", arrowright_8x8);
			m1_u8g2_nextpage();
		}

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				break;
			}
			else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (selection > 0) selection--;
				else selection = count - 1;
				redraw = true;
			}
			else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (selection < count - 1) selection++;
				else selection = 0;
				redraw = true;
			}
			else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				bt_scan_detail_screen(&list[selection]);
				redraw = true;
			}
		}
	}

	if (list) free(list);
	xQueueReset(main_q_hdl);
	m1_esp32_deinit();
} // void bluetooth_scan(void)



/******************************************************************************/
/*               S A V E D   D E V I C E S   S C R E E N                     */
/******************************************************************************/

static void bt_saved_detail_screen(uint8_t dev_idx)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	char prn_msg[25];
	uint8_t menu_sel = 0;  /* 0=Connect, 1=Delete */
	bool redraw = true;

	while (1)
	{
		if (redraw)
		{
			if (dev_idx >= s_saved_count) break; /* safety check */

			redraw = false;
			m1_u8g2_firstpage();
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			bt_draw_title_bar("Saved Device");

			uint8_t y = 24;
			/* Name */
			if (s_saved_devices[dev_idx].name[0])
			{
				char nbuf[22];
				strncpy(nbuf, s_saved_devices[dev_idx].name, 21);
				nbuf[21] = '\0';
				u8g2_DrawStr(&m1_u8g2, 2, y, nbuf);
			}
			else
				u8g2_DrawStr(&m1_u8g2, 2, y, "(No name)");
			y += 10;

			/* MAC */
			u8g2_DrawStr(&m1_u8g2, 2, y, s_saved_devices[dev_idx].addr);
			y += 10;

			/* Options */
			snprintf(prn_msg, sizeof(prn_msg), "%sConnect  %sDelete",
				menu_sel == 0 ? ">" : " ",
				menu_sel == 1 ? ">" : " ");
			u8g2_DrawStr(&m1_u8g2, 2, y, prn_msg);

			m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "OK", arrowright_8x8);
			m1_u8g2_nextpage();
		}

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				break;
			}
			else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK ||
					 this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
			{
				menu_sel = menu_sel ? 0 : 1;
				redraw = true;
			}
			else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (menu_sel == 1) /* Delete */
				{
					bt_remove_device(dev_idx);
					bt_show_message("Deleted!", NULL, 1000);
					break;
				}
				else /* Connect */
				{
					/* Init ESP32 if needed */
					if (!m1_esp32_get_init_status())
					{
						m1_esp32_init();
					}
					if (!get_esp32_main_init_status())
					{
						bt_show_message("Initializing...", NULL, 0);
						esp32_main_init();
					}

					if (get_esp32_main_init_status())
					{
						bt_show_message("Connecting...", s_saved_devices[dev_idx].addr, 0);
						ctrl_cmd_t conn_req = CTRL_CMD_DEFAULT_REQ();
						conn_req.cmd_timeout_sec = 15;
						uint8_t cret = ble_connect(&conn_req,
							s_saved_devices[dev_idx].addr,
							s_saved_devices[dev_idx].addr_type);

						if (cret == SUCCESS)
						{
							s_bt_conn_state.connected = true;
							strncpy(s_bt_conn_state.addr, s_saved_devices[dev_idx].addr, BSSID_STR_SIZE - 1);
							strncpy(s_bt_conn_state.name, s_saved_devices[dev_idx].name, SSID_LENGTH - 1);
							bt_show_message("Connected!", NULL, 1500);
						}
						else
						{
							bt_show_message("Connect failed!", "Not supported?", 2000);
						}
					}
					else
					{
						bt_show_message("ESP32 not ready!", NULL, 1500);
					}
					redraw = true;
				}
			}
		}
	}
} // static void bt_saved_detail_screen(...)


void bluetooth_saved_devices(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	int16_t selection = 0;
	int16_t scroll_offset = 0;
	bool redraw = true;
	char page_info[16];

	bt_load_devices();

	while (1)
	{
		if (redraw)
		{
			redraw = false;
			m1_u8g2_firstpage();
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

			if (s_saved_count == 0)
			{
				bt_draw_title_bar("Saved Devices");
				u8g2_DrawStr(&m1_u8g2, 2, 30, "No saved devices");
				m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "", blank_8x8);
				m1_u8g2_nextpage();
			}
			else
			{
				snprintf(page_info, sizeof(page_info), "Saved (%d)", s_saved_count);
				bt_draw_title_bar(page_info);

				if (selection >= s_saved_count) selection = s_saved_count - 1;
				if (selection < 0) selection = 0;
				if (selection < scroll_offset) scroll_offset = selection;
				if (selection >= scroll_offset + BT_LIST_VISIBLE)
					scroll_offset = selection - BT_LIST_VISIBLE + 1;

				for (int i = 0; i < BT_LIST_VISIBLE && scroll_offset + i < s_saved_count; i++)
				{
					int idx = scroll_offset + i;
					const char *display_name;
					if (s_saved_devices[idx].name[0])
						display_name = s_saved_devices[idx].name;
					else
						display_name = s_saved_devices[idx].addr;

					bt_draw_list_item(i, display_name, (idx == selection));
				}

				snprintf(page_info, sizeof(page_info), "%d/%d", selection + 1, s_saved_count);
				m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, page_info, "Open", arrowright_8x8);
				m1_u8g2_nextpage();
			}
		}

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				xQueueReset(main_q_hdl);
				break;
			}
			else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (s_saved_count > 0)
				{
					if (selection > 0) selection--;
					else selection = s_saved_count - 1;
					redraw = true;
				}
			}
			else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (s_saved_count > 0)
				{
					if (selection < s_saved_count - 1) selection++;
					else selection = 0;
					redraw = true;
				}
			}
			else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (s_saved_count > 0)
				{
					bt_saved_detail_screen(selection);
					bt_load_devices(); /* Reload in case device was deleted */
					if (selection >= s_saved_count && s_saved_count > 0)
						selection = s_saved_count - 1;
					redraw = true;
				}
			}
		}
	}
} // void bluetooth_saved_devices(void)



/******************************************************************************/
/*               B T   I N F O   S C R E E N                                 */
/******************************************************************************/

void bluetooth_info(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	ctrl_cmd_t app_req = CTRL_CMD_DEFAULT_REQ();
	char version_str[STATUS_LENGTH];
	uint8_t y;

	memset(version_str, 0, sizeof(version_str));

	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

	/* Init ESP32 if needed */
	if (!m1_esp32_get_init_status())
	{
		m1_esp32_init();
	}
	if (!get_esp32_main_init_status())
	{
		bt_show_message("Initializing...", NULL, 0);
		esp32_main_init();
	}

	if (get_esp32_main_init_status())
	{
		bt_show_message("Querying...", NULL, 0);
		app_req.cmd_timeout_sec = 10;
		app_req.msg_id = CTRL_RESP_GET_VERSION;
		ret = esp_get_version(&app_req);
		if (ret == SUCCESS)
		{
			strncpy(version_str, app_req.u.wifi_ap_config.status, STATUS_LENGTH - 1);
		}
		else
		{
			strncpy(version_str, "Unknown", sizeof(version_str) - 1);
		}
	}
	else
	{
		strncpy(version_str, "ESP32 offline", sizeof(version_str) - 1);
	}

	/* Draw info screen */
	m1_u8g2_firstpage();
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	bt_draw_title_bar("BT Info");

	y = 24;
	u8g2_DrawStr(&m1_u8g2, 2, y, "ESP32 AT FW:");
	y += 10;
	u8g2_DrawStr(&m1_u8g2, 4, y, version_str);
	y += 12;

	if (s_bt_conn_state.connected)
	{
		u8g2_DrawStr(&m1_u8g2, 2, y, "Connected:");
		y += 10;
		if (s_bt_conn_state.name[0])
		{
			char nbuf[22];
			strncpy(nbuf, s_bt_conn_state.name, 21);
			nbuf[21] = '\0';
			u8g2_DrawStr(&m1_u8g2, 4, y, nbuf);
		}
		else
			u8g2_DrawStr(&m1_u8g2, 4, y, s_bt_conn_state.addr);
	}
	else
	{
		u8g2_DrawStr(&m1_u8g2, 2, y, "Not connected");
	}

	m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "", blank_8x8);
	m1_u8g2_nextpage();

	/* Wait for BACK */
	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
				break;
		}
	}

	xQueueReset(main_q_hdl);
	m1_esp32_deinit();
} // void bluetooth_info(void)


bt_connection_state_t *bt_get_connection_state(void)
{
	return &s_bt_conn_state;
}


/*============================================================================*/
void bluetooth_set_badbt_name(void)
{
	char new_name[BADBT_NAME_MAX_LEN + 1] = {0};

	uint8_t len = m1_vkb_get_filename("Bad-BT Name", m1_badbt_name, new_name);
	if (len > 0)
	{
		strncpy(m1_badbt_name, new_name, BADBT_NAME_MAX_LEN);
		m1_badbt_name[BADBT_NAME_MAX_LEN] = '\0';
		settings_save_to_sd();

		bt_show_message("Name saved:", m1_badbt_name, 1500);
	}
}


#else /* !M1_APP_BT_MANAGE_ENABLE — original scan/advertise code */


/*============================================================================*/
/*
 * This function scans for devices. (Original implementation)
 */
/*============================================================================*/
void bluetooth_scan(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	ctrl_cmd_t app_req = CTRL_CMD_DEFAULT_REQ();
	uint16_t list_count;

    /* Graphic work starts here */
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	if ( !m1_esp32_get_init_status() )
	{
		m1_esp32_init();
	}
	if ( !get_esp32_main_init_status() )
	{
		m1_u8g2_firstpage();
		u8g2_DrawStr(&m1_u8g2, 6, 15, "Initializing...");
		u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32, hourglass_18x32);
		m1_u8g2_nextpage();
		esp32_main_init();
	}

	list_count = 0;

	m1_u8g2_firstpage();
	if ( get_esp32_main_init_status() )
	{
		u8g2_DrawStr(&m1_u8g2, 6, 15, "Scanning BLE...");
		u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32, hourglass_18x32);
		m1_u8g2_nextpage();

		app_req.cmd_timeout_sec = M1_BLE_SCANNING_TIME;
		app_req.msg_id = CTRL_RESP_GET_BLE_SCAN_LIST;
		ret = ble_scan_list(&app_req);
		ret = ble_scan_list_validation(&app_req);
		if ( ret )
		{
			list_count = ble_scan_list_print(&app_req, true);
		}
		else
		{
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
			u8g2_DrawBox(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32);
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 32/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 32, 32, wifi_error_32x32);
			u8g2_DrawStr(&m1_u8g2, 6, 15 + M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "Failed. Retrying...");
			m1_u8g2_nextpage();
			esp32_disable();
			m1_hard_delay(1);
			esp32_enable();
			m1_hard_delay(200);
		}
	}
	else
	{
		u8g2_DrawStr(&m1_u8g2, 6, 15 + M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "ESP32 not ready!");
		m1_u8g2_nextpage();
	}

	while (1 )
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				{
					if (app_req.u.wifi_ap_scan.out_list != NULL)
					{
						free(app_req.u.wifi_ap_scan.out_list);
					}
					ble_scan_list_print(NULL, false);

					xQueueReset(main_q_hdl);
					m1_esp32_deinit();
					break;
				}
				else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
				{
					if ( list_count )
						ble_scan_list_print(&app_req, true);
				}
				else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
				{
					if ( list_count )
						ble_scan_list_print(&app_req, false);
				}
				else if ( this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
				{
					;
				}
			}
			else
			{
				;
			}
		}
	}
} // void bluetooth_scan(void)

#endif /* M1_APP_BT_MANAGE_ENABLE */



/*============================================================================*/
/*
 * This function initializes the server mode and starts the advertising mode.
 */
/*============================================================================*/
void bluetooth_advertise(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	ctrl_cmd_t app_req = CTRL_CMD_DEFAULT_REQ();
	char *index, *start_index, *next_index;
	char prn_buffer[25], prn_cnt;
	uint8_t cp_len;

    /* Graphic work starts here */
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	if ( !m1_esp32_get_init_status() )
	{
		m1_esp32_init();
	}
	if ( !get_esp32_main_init_status() )
	{
		m1_u8g2_firstpage();
		u8g2_DrawStr(&m1_u8g2, 6, 15, "Initializing...");
		u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32, hourglass_18x32);
		m1_u8g2_nextpage();
		esp32_main_init();
	}

	m1_u8g2_firstpage();
	if ( get_esp32_main_init_status() )
	{
		u8g2_DrawStr(&m1_u8g2, 6, 15, "Advertising...");
		u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32, hourglass_18x32);
		m1_u8g2_nextpage();

		app_req.cmd_timeout_sec = M1_BLE_SCANNING_TIME;
		app_req.msg_id = CTRL_RESP_SET_BLE_RESET;
		ret = esp_dev_reset(&app_req);
		app_req.msg_id = CTRL_RESP_SET_BLE_ADVERTISE;
		ret = ble_advertise(&app_req);

		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
		u8g2_DrawBox(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32);
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

		if ( !ret )
		{
			u8g2_DrawStr(&m1_u8g2, 100, 15, "Done");
			index = ESP32C6_AT_REQ_ADV_DATA;
			if ( index )
			{
				prn_cnt = 0;
				while (true)
				{
					start_index = strstr(index, "\"");
					if ( !start_index )
						break;
					next_index = strstr(&start_index[1], "\"");
					cp_len = next_index - start_index - 1;
					strncpy(prn_buffer, &start_index[1], cp_len);
					prn_buffer[cp_len] = '\0';
					u8g2_DrawStr(&m1_u8g2, 2, 35 + prn_cnt*10, prn_buffer);
					index = &next_index[1];
					prn_cnt++;
					if ( prn_cnt >= 3)
						break;
				}
			}
			else
			{
				u8g2_DrawStr(&m1_u8g2, 6, 25, "Done");
				m1_u8g2_nextpage();
			}
			m1_u8g2_nextpage();
		}
		else
		{
			u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 32/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 32, 32, wifi_error_32x32);
			u8g2_DrawStr(&m1_u8g2, 6, 15 + M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "Failed. Retrying...");
			m1_u8g2_nextpage();
			esp32_disable();
			m1_hard_delay(1);
			esp32_enable();
			m1_hard_delay(200);
		}
	}
	else
	{
		u8g2_DrawStr(&m1_u8g2, 6, 15 + M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "ESP32 not ready!");
		m1_u8g2_nextpage();
	}

	while (1 )
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				{
					u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH - 18*2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32, hourglass_18x32);
					m1_u8g2_nextpage();

					app_req.cmd_timeout_sec = M1_BLE_SCANNING_TIME;
					app_req.msg_id = CTRL_RESP_SET_BLE_RESET;
					ret = esp_dev_reset(&app_req);

					xQueueReset(main_q_hdl);
					m1_esp32_deinit();
					break;
				}
			}
			else
			{
				;
			}
		}
	}
} // void bluetooth_advertise(void)



/*============================================================================*/
/*
 * This function placeholder for config.
 */
/*============================================================================*/
void bluetooth_config(void)
{
#ifdef M1_APP_BT_MANAGE_ENABLE
	bluetooth_saved_devices();
#else
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;

	m1_gui_let_update_fw();

	while (1 )
	{
		;
		;
		;

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				{
					;
					xQueueReset(main_q_hdl);
					break;
				}
				else
				{
					;
				}
			}
			else
			{
				;
			}
		}
	}
#endif
} // void bluetooth_config(void)



#ifndef M1_APP_BT_MANAGE_ENABLE
/*============================================================================*/
/*
 * This function validates the scan list. (Original)
 */
/*============================================================================*/
static uint8_t ble_scan_list_validation(ctrl_cmd_t *app_resp)
{
	if (!app_resp || (app_resp->msg_type != CTRL_RESP))
	{
		if (app_resp)
			M1_LOG_D(M1_LOGDB_TAG, "Msg type is not response[%u]\n\r", app_resp->msg_type);
		return false;
	}
	if (app_resp->resp_event_status != SUCCESS)
	{
		return false;
	}
	if (app_resp->msg_id != CTRL_RESP_GET_BLE_SCAN_LIST)
	{
		M1_LOG_D(M1_LOGDB_TAG, "Invalid Response[%u] to parse\n\r", app_resp->msg_id);
		return false;
	}

	return true;
} // static uint8_t ble_scan_list_validation(ctrl_cmd_t *app_resp)




/*============================================================================*/
/*
 * This function displays all scanned device list. (Original)
 * Return: number of devices found
 */
/*============================================================================*/
static uint16_t ble_scan_list_print(ctrl_cmd_t *app_resp, bool up_dir)
{
	static uint16_t i;
	static wifi_ap_scan_list_t *w_scan_p;
	static wifi_scanlist_t *list;
	static bool init_done = false;
	char prn_msg[25];
	uint8_t y_offset;

	if ( !app_resp && !up_dir )
	{
		init_done = false;
		return 0;
	}

	if ( !init_done )
	{
		init_done = true;
		w_scan_p = &app_resp->u.wifi_ap_scan;
		list = w_scan_p->out_list;

		if (!w_scan_p->count)
		{
			strcpy(prn_msg, "No device found!");
			M1_LOG_D(M1_LOGDB_TAG, "No device found\n\r");
			init_done = false;
		}
		else if (!list)
		{
			strcpy(prn_msg, "Try again!");
			M1_LOG_D(M1_LOGDB_TAG, "Failed to get scanned device list\n\r");
			init_done = false;
		}
		else
		{
			M1_LOG_D(M1_LOGDB_TAG, "Number of available devices is %d\n\r", w_scan_p->count);
		}

		if ( !init_done )
		{
			u8g2_DrawStr(&m1_u8g2, 6, 25 + M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, prn_msg);
			m1_u8g2_nextpage();
			return 0;
		}
		i = 1;
		up_dir = true;
	}

	if ( up_dir )
	{
		if ( i )
			i--;
		else
			i = w_scan_p->count-1;
	}
	else
	{
		i++;
		if ( i >= w_scan_p->count )
			i = 0;
	}

	m1_u8g2_firstpage();
	u8g2_DrawXBMP(&m1_u8g2, 0, 0, 128, 14, m1_frame_128_14);
	u8g2_DrawStr(&m1_u8g2, 2, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "Total Dev:");

	sprintf(prn_msg, "%d", w_scan_p->count);
	u8g2_DrawStr(&m1_u8g2, 2 + strlen("Total Dev: ")*M1_GUI_FONT_WIDTH + 2, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, prn_msg);

	sprintf(prn_msg, "%d/%d", i + 1, w_scan_p->count);
	u8g2_DrawStr(&m1_u8g2, M1_LCD_DISPLAY_WIDTH - 6*M1_GUI_FONT_WIDTH, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, prn_msg);

	y_offset = 14 + M1_GUI_FONT_HEIGHT - 1;
	u8g2_DrawStr(&m1_u8g2, 2, y_offset, list[i].bssid);
	y_offset += M1_GUI_FONT_HEIGHT + M1_GUI_ROW_SPACING;
	sprintf(prn_msg, "RSSI: %ddBm", list[i].rssi);
	u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);
	y_offset += M1_GUI_FONT_HEIGHT;
	sprintf(prn_msg, "Address type: %d", list[i].encryption_mode);
	u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);

	m1_u8g2_nextpage();

	M1_LOG_D(M1_LOGDB_TAG, "%d) bssid \"%s\" rssi \"%d\" address type \"%d\" \n\r",\
						i, list[i].bssid, list[i].rssi, list[i].encryption_mode);

	return w_scan_p->count;
} // static uint16_t ble_scan_list_print(ctrl_cmd_t *app_resp, bool up_dir)

#endif /* !M1_APP_BT_MANAGE_ENABLE */
