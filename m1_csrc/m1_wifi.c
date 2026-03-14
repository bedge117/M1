/* See COPYING.txt for license details. */

/*
*
* m1_wifi.c
*
* Library for M1 Wifi — scan, connect, saved networks, status
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_wifi.h"
#include "m1_esp32_hal.h"
//#include "control.h"
#include "ctrl_api.h"
#include "esp_app_main.h"
#include "m1_compile_cfg.h"

#ifdef M1_APP_WIFI_CONNECT_ENABLE
#include "m1_wifi_cred.h"
#include "m1_virtual_kb.h"
#endif

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG	"Wifi"

#define M1_WIFI_AP_SCANNING_TIME	30 // seconds

#define M1_GUI_ROW_SPACING			1

//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/

#ifdef M1_APP_WIFI_CONNECT_ENABLE
/* Track current AP index in scan list for connect-from-scan */
static uint16_t s_current_ap_index = 0;
static bool s_wifi_connected = false;
static char s_connected_ssid[SSID_LENGTH];
#endif

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

void menu_wifi_init(void);
void menu_wifi_exit(void);

void wifi_scan_ap(void);
void wifi_config(void);

static uint16_t wifi_ap_list_print(ctrl_cmd_t *app_resp, bool up_dir);
static uint8_t wifi_ap_list_validation(ctrl_cmd_t *app_resp);

#ifdef M1_APP_WIFI_CONNECT_ENABLE
void wifi_saved_networks(void);
void wifi_show_status(void);
void wifi_disconnect(void);
static uint16_t wifi_ap_list_get_index(void);
static bool wifi_do_connect(const char *ssid, const char *password);
static void wifi_display_msg(const char *line1, const char *line2);
#endif

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
  * @brief  Initialize ESP32 module if not already initialized
  * @retval true if ready, false if failed
  */
/*============================================================================*/
static bool wifi_ensure_esp32_ready(void)
{
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
	return get_esp32_main_init_status();
}


/*============================================================================*/
void menu_wifi_init(void)
{
	;
} // void menu_wifi_init(void)


/*============================================================================*/
void  menu_wifi_exit(void)
{
	;
} // void  menu_wifi_exit(void)



/*============================================================================*/
/**
  * @brief Scans for wifi access point list and allows connecting
  * @param
  * @retval
  */
/*============================================================================*/
void wifi_scan_ap(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	ctrl_cmd_t app_req = CTRL_CMD_DEFAULT_REQ();
	uint16_t list_count;
#ifdef M1_APP_WIFI_CONNECT_ENABLE
	wifi_scanlist_t *list;
	wifi_credential_t cred;
	char password[WIFI_CRED_PASS_MAX_LEN];
	bool do_connect;
#endif

    /* Graphic work starts here */
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	if ( !wifi_ensure_esp32_ready() )
	{
		u8g2_DrawStr(&m1_u8g2, 6, 15 + M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "ESP32 not ready!");
		m1_u8g2_nextpage();
		/* Fall through to event loop so user can press BACK */
	}

	list_count = 0;

	m1_u8g2_firstpage();
	if ( get_esp32_main_init_status() )
	{
		u8g2_DrawStr(&m1_u8g2, 6, 15, "Scanning AP...");
		u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32, hourglass_18x32);
		m1_u8g2_nextpage();

		// implemented synchronous
		app_req.cmd_timeout_sec = M1_WIFI_AP_SCANNING_TIME; //DEFAULT_CTRL_RESP_TIMEOUT //30 sec
		app_req.msg_id = CTRL_RESP_GET_AP_SCAN_LIST;
		ret = wifi_ap_scan_list(&app_req);
		ret = wifi_ap_list_validation(&app_req);
		if ( ret )
		{
			list_count = wifi_ap_list_print(&app_req, true);
		} // if ( ret )
		else
		{
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
			u8g2_DrawBox(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32); // Clear old image
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 32/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 32, 32, wifi_error_32x32);
			u8g2_DrawStr(&m1_u8g2, 6, 15 + M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "Failed. Retrying...");
			m1_u8g2_nextpage();
			// Reset the ESP module
			esp32_disable();
			m1_hard_delay(1);
			esp32_enable();
			/* stop spi transactions short time to avoid slave sync issues */
			m1_hard_delay(200);
		}
	} // if ( get_esp32_main_init_status() )

	while (1 ) // Main loop of this task
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // user wants to exit?
				{
					if (app_req.u.wifi_ap_scan.out_list != NULL)
					{
						free(app_req.u.wifi_ap_scan.out_list);
					}
					wifi_ap_list_print(NULL, false);

					xQueueReset(main_q_hdl); // Reset main q before return
					m1_esp32_deinit();
					break; // Exit
				}
				else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
				{
					if ( list_count )
						wifi_ap_list_print(&app_req, true);
				}
				else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
				{
					if ( list_count )
						wifi_ap_list_print(&app_req, false);
				}
				else if ( this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
				{
#ifdef M1_APP_WIFI_CONNECT_ENABLE
					if ( !list_count )
						continue;

					/* Get the currently displayed AP */
					list = app_req.u.wifi_ap_scan.out_list;
					s_current_ap_index = wifi_ap_list_get_index();

					if ( s_current_ap_index >= app_req.u.wifi_ap_scan.count )
						continue;

					do_connect = false;
					password[0] = '\0';

					/* Check if we have saved credentials */
					if ( wifi_cred_find((const char *)list[s_current_ap_index].ssid, &cred) )
					{
						/* Use saved password */
						strncpy(password, cred.password, WIFI_CRED_PASS_MAX_LEN - 1);
						password[WIFI_CRED_PASS_MAX_LEN - 1] = '\0';
						do_connect = true;
					}
					else if ( list[s_current_ap_index].encryption_mode == 0 )
					{
						/* Open network - no password needed */
						do_connect = true;
					}
					else
					{
						/* Prompt for password using virtual keyboard */
						memset(password, 0, sizeof(password));
						uint8_t pw_len = m1_vkb_get_filename("Password:",
							"", password);
						if ( pw_len > 0 )
						{
							do_connect = true;
						}
						else
						{
							/* User cancelled - redraw AP list */
							u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
							wifi_ap_list_print(NULL, false); /* reset state */
							list_count = wifi_ap_list_print(&app_req, true);
						}
					}

					if ( do_connect )
					{
						bool ok = wifi_do_connect(
							(const char *)list[s_current_ap_index].ssid,
							password);

						if ( ok )
						{
							/* Offer to save credentials if not already saved */
							if ( !wifi_cred_find((const char *)list[s_current_ap_index].ssid, &cred)
								&& password[0] != '\0' )
							{
								wifi_cred_save(
									(const char *)list[s_current_ap_index].ssid,
									password);
								wifi_display_msg("Credentials", "saved!");
								vTaskDelay(pdMS_TO_TICKS(1500));
							}
						}

						/* Redraw the scan list */
						u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
						wifi_ap_list_print(NULL, false); /* reset state */
						list_count = wifi_ap_list_print(&app_req, true);
					}
#endif /* M1_APP_WIFI_CONNECT_ENABLE */
				}
			} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		} // if (ret==pdTRUE)
	} // while (1 ) // Main loop of this task

} // void wifi_scan_ap(void)



/*============================================================================*/
/**
  * @brief Displays all scanned AP list.
  * @param
  * @retval
  */
/*============================================================================*/
static uint16_t wifi_ap_list_print(ctrl_cmd_t *app_resp, bool up_dir)
{
	static uint16_t i;
	static wifi_ap_scan_list_t *w_scan_p;
	static wifi_scanlist_t *list;
	static bool init_done = false;
	char prn_msg[25];
	uint8_t y_offset;

	if ( !app_resp && !up_dir ) // reset condition?
	{
		init_done = false;
		return 0;
	} // if ( !app_resp && !up_dir )

	if ( !init_done )
	{
		init_done = true;
		w_scan_p = &app_resp->u.wifi_ap_scan;
		list = w_scan_p->out_list;

		if (!w_scan_p->count)
		{
			strcpy(prn_msg, "No AP found!");
			M1_LOG_I(M1_LOGDB_TAG, "No AP found\n\r");
			init_done = false;
		}
		else if (!list)
		{
			strcpy(prn_msg, "Try again!");
			M1_LOG_I(M1_LOGDB_TAG, "Failed to get scanned AP list\n\r");
			init_done = false;
		}
		else
		{
			M1_LOG_I(M1_LOGDB_TAG, "Number of available APs is %d\n\r", w_scan_p->count);
		}

		if ( !init_done )
		{
			u8g2_DrawStr(&m1_u8g2, 6, 25 + M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, prn_msg);
			m1_u8g2_nextpage(); // Update display RAM
			return 0;
		}
		// Display first AP in the list
		i = 1;
		up_dir = true; // Overwrite the up_dir for the AP to be displayed for the first time
	} // if ( !init_done )

	if ( up_dir )
	{
		if ( i )
			i--;
		else
			i = w_scan_p->count-1; // roll over
	}
	else
	{
		i++;
		if ( i >= w_scan_p->count )
			i = 0; // roll over
	}

#ifdef M1_APP_WIFI_CONNECT_ENABLE
	s_current_ap_index = i;
#endif

	m1_u8g2_firstpage();
	u8g2_DrawXBMP(&m1_u8g2, 0, 0, 128, 14, m1_frame_128_14);
	u8g2_DrawStr(&m1_u8g2, 2, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "Total AP:");

	sprintf(prn_msg, "%d", w_scan_p->count);
	u8g2_DrawStr(&m1_u8g2, 2 + strlen("Total AP: ")*M1_GUI_FONT_WIDTH + 2, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, prn_msg);

	sprintf(prn_msg, "%d/%d", i + 1, w_scan_p->count); // Current AP
	u8g2_DrawStr(&m1_u8g2, M1_LCD_DISPLAY_WIDTH - 6*M1_GUI_FONT_WIDTH, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, prn_msg);

	y_offset = 14 + M1_GUI_FONT_HEIGHT - 1;
	// Draw text
	if ( list[i].ssid[0]==0x00 ) // Hidden SSID?
		strcpy(prn_msg, "*hidden*");
	else
		strncpy(prn_msg, (char *)list[i].ssid, M1_LCD_DISPLAY_WIDTH/M1_GUI_FONT_WIDTH);
	prn_msg[M1_LCD_DISPLAY_WIDTH/M1_GUI_FONT_WIDTH] = '\0';
	u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);
	y_offset += M1_GUI_FONT_HEIGHT;
	u8g2_DrawStr(&m1_u8g2, 2, y_offset, (char *)list[i].bssid);
	y_offset += M1_GUI_FONT_HEIGHT + M1_GUI_ROW_SPACING;
	sprintf(prn_msg, "RSSI: %ddBm", list[i].rssi);
	u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);
	y_offset += M1_GUI_FONT_HEIGHT;
	sprintf(prn_msg, "Channel: %d", list[i].channel);
	u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);
	y_offset += M1_GUI_FONT_HEIGHT;
	sprintf(prn_msg, "Auth mode: %d", list[i].encryption_mode);
	u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);

	m1_u8g2_nextpage(); // Update display RAM

	M1_LOG_D(M1_LOGDB_TAG, "%d) ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" auth mode \"%d\" \n\r",\
						i, list[i].ssid, list[i].bssid, list[i].rssi,
						list[i].channel, list[i].encryption_mode);

	return w_scan_p->count;
} // static uint16_t wifi_ap_list_print(ctrl_cmd_t *app_resp, bool up_dir)



/*============================================================================*/
/**
  * @brief Validates the AP list.
  */
/*============================================================================*/
static uint8_t wifi_ap_list_validation(ctrl_cmd_t *app_resp)
{
	if (!app_resp || (app_resp->msg_type != CTRL_RESP))
	{
		if (app_resp)
			M1_LOG_I(M1_LOGDB_TAG, "Msg type is not response[%u]\n\r", app_resp->msg_type);
		return false;
	}
	if (app_resp->resp_event_status != SUCCESS)
	{
		//process_failed_responses(app_resp);
		return false;
	}
	if (app_resp->msg_id != CTRL_RESP_GET_AP_SCAN_LIST)
	{
		M1_LOG_I(M1_LOGDB_TAG, "Invalid Response[%u] to parse\n\r", app_resp->msg_id);
		return false;
	}

	return true;
} // static uint8_t wifi_ap_list_validation(ctrl_cmd_t *app_resp)


#ifdef M1_APP_WIFI_CONNECT_ENABLE

/*============================================================================*/
/**
  * @brief Returns the current AP index from the list display
  */
/*============================================================================*/
static uint16_t wifi_ap_list_get_index(void)
{
	return s_current_ap_index;
}


/*============================================================================*/
/**
  * @brief Display a two-line message centered on screen
  */
/*============================================================================*/
static void wifi_display_msg(const char *line1, const char *line2)
{
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	m1_u8g2_firstpage();
	if ( line1 )
		u8g2_DrawStr(&m1_u8g2, 6, 25, line1);
	if ( line2 )
		u8g2_DrawStr(&m1_u8g2, 6, 25 + M1_GUI_FONT_HEIGHT + 2, line2);
	m1_u8g2_nextpage();
}


/*============================================================================*/
/**
  * @brief Display a message with hourglass icon
  */
/*============================================================================*/
static void wifi_display_busy(const char *msg)
{
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	m1_u8g2_firstpage();
	u8g2_DrawStr(&m1_u8g2, 6, 15, msg);
	u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32, hourglass_18x32);
	m1_u8g2_nextpage();
}


/*============================================================================*/
/**
  * @brief Execute WiFi connect sequence
  * @param ssid  - network SSID
  * @param password - network password (empty string for open networks)
  * @retval true on success
  */
/*============================================================================*/
static bool wifi_do_connect(const char *ssid, const char *password)
{
	ctrl_cmd_t conn_req = CTRL_CMD_DEFAULT_REQ();
	uint8_t ret;

	wifi_display_busy("Connecting...");

	/* Populate connect request */
	strncpy((char *)conn_req.u.wifi_ap_config.ssid, ssid, SSID_LENGTH - 1);
	conn_req.u.wifi_ap_config.ssid[SSID_LENGTH - 1] = '\0';
	strncpy((char *)conn_req.u.wifi_ap_config.pwd, password, PASSWORD_LENGTH - 1);
	conn_req.u.wifi_ap_config.pwd[PASSWORD_LENGTH - 1] = '\0';
	conn_req.cmd_timeout_sec = DEFAULT_CTRL_RESP_CONNECT_AP_TIMEOUT;
	conn_req.msg_id = CTRL_RESP_CONNECT_AP;

	ret = wifi_connect_ap(&conn_req);

	if ( ret == SUCCESS && conn_req.resp_event_status == SUCCESS )
	{
		s_wifi_connected = true;
		strncpy(s_connected_ssid, ssid, SSID_LENGTH - 1);
		s_connected_ssid[SSID_LENGTH - 1] = '\0';

		/* Get IP address to display */
		ctrl_cmd_t ip_req = CTRL_CMD_DEFAULT_REQ();
		ip_req.cmd_timeout_sec = 10;
		wifi_get_ip(&ip_req);

		m1_u8g2_firstpage();
		u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 6, 15, "Connected!");
		u8g2_DrawStr(&m1_u8g2, 6, 15 + M1_GUI_FONT_HEIGHT + 2, ssid);
		if ( ip_req.u.wifi_ap_config.status[0] )
		{
			char ip_msg[25];
			snprintf(ip_msg, sizeof(ip_msg), "IP: %s", ip_req.u.wifi_ap_config.status);
			u8g2_DrawStr(&m1_u8g2, 6, 15 + 2*(M1_GUI_FONT_HEIGHT + 2), ip_msg);
		}
		m1_u8g2_nextpage();
		M1_LOG_I(M1_LOGDB_TAG, "Connected to %s, IP: %s\n\r", ssid, ip_req.u.wifi_ap_config.status);
		vTaskDelay(pdMS_TO_TICKS(2500));
		return true;
	}
	else
	{
		const char *err_msg = "Connect failed!";
		if ( conn_req.resp_event_status == 2 )
			err_msg = "Wrong password!";
		else if ( conn_req.resp_event_status == 3 )
			err_msg = "AP not found!";
		else if ( conn_req.resp_event_status == 1 )
			err_msg = "Timeout!";

		wifi_display_msg("WiFi Error:", err_msg);
		M1_LOG_E(M1_LOGDB_TAG, "Connect failed: %s (code %ld)\n\r", err_msg, conn_req.resp_event_status);
		vTaskDelay(pdMS_TO_TICKS(2500));
		return false;
	}
}


/*============================================================================*/
/**
  * @brief WiFi config menu - now replaced by Saved Networks
  *        Kept for backward compat with old menu structure
  */
/*============================================================================*/
void wifi_config(void)
{
	/* Redirect to saved networks manager */
	wifi_saved_networks();
}


/*============================================================================*/
/**
  * @brief Saved networks management screen
  *        Shows list of saved WiFi credentials, allows connect/delete
  */
/*============================================================================*/
void wifi_saved_networks(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	wifi_credential_t creds[WIFI_CRED_MAX_STORED];
	uint8_t cred_count;
	uint8_t sel_idx = 0;
	char prn_msg[25];
	uint8_t y_offset;

	/* Load saved credentials */
	cred_count = wifi_cred_load_all(creds, WIFI_CRED_MAX_STORED);

	/* Display the list */
	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

	if ( cred_count == 0 )
	{
		wifi_display_msg("No saved", "networks");
		/* Wait for BACK button */
		while (1)
		{
			ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
			if (ret==pdTRUE && q_item.q_evt_type==Q_EVENT_KEYPAD)
			{
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				{
					xQueueReset(main_q_hdl);
					return;
				}
			}
		}
	}

	/* Draw credential list */
	while (1)
	{
		m1_u8g2_firstpage();
		u8g2_DrawXBMP(&m1_u8g2, 0, 0, 128, 14, m1_frame_128_14);
		u8g2_DrawStr(&m1_u8g2, 2, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "Saved Networks");

		sprintf(prn_msg, "%d/%d", sel_idx + 1, cred_count);
		u8g2_DrawStr(&m1_u8g2, M1_LCD_DISPLAY_WIDTH - 6*M1_GUI_FONT_WIDTH,
			M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, prn_msg);

		y_offset = 14 + M1_GUI_FONT_HEIGHT;

		/* Show selected network SSID */
		strncpy(prn_msg, creds[sel_idx].ssid, 20);
		prn_msg[20] = '\0';
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);
		y_offset += M1_GUI_FONT_HEIGHT + 2;

		/* Instructions */
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, "OK: Connect");
		y_offset += M1_GUI_FONT_HEIGHT;
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, "RIGHT: Delete");

		m1_u8g2_nextpage();

		/* Wait for button input */
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE && q_item.q_evt_type==Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
			{
				xQueueReset(main_q_hdl);
				return;
			}
			else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
			{
				if ( sel_idx > 0 )
					sel_idx--;
				else
					sel_idx = cred_count - 1;
			}
			else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
			{
				sel_idx++;
				if ( sel_idx >= cred_count )
					sel_idx = 0;
			}
			else if ( this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
			{
				/* Connect using saved credentials */
				u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
				if ( !wifi_ensure_esp32_ready() )
				{
					wifi_display_msg("ESP32", "not ready!");
					vTaskDelay(pdMS_TO_TICKS(2000));
				}
				else
				{
					wifi_do_connect(creds[sel_idx].ssid, creds[sel_idx].password);
				}
				/* Redraw list */
				u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			}
			else if ( this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK )
			{
				/* Delete credential */
				wifi_display_msg("Deleting...", creds[sel_idx].ssid);
				wifi_cred_delete(creds[sel_idx].ssid);
				vTaskDelay(pdMS_TO_TICKS(1000));

				/* Reload list */
				cred_count = wifi_cred_load_all(creds, WIFI_CRED_MAX_STORED);
				if ( cred_count == 0 )
				{
					wifi_display_msg("No saved", "networks");
					vTaskDelay(pdMS_TO_TICKS(1500));
					xQueueReset(main_q_hdl);
					return;
				}
				if ( sel_idx >= cred_count )
					sel_idx = cred_count - 1;
				u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			}
		}
	} // while(1)
} // void wifi_saved_networks(void)



/*============================================================================*/
/**
  * @brief Show WiFi connection status - IP, SSID, RSSI, MAC
  */
/*============================================================================*/
void wifi_show_status(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	ctrl_cmd_t ip_req;
	char prn_msg[25];
	uint8_t y_offset;

	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

	if ( !wifi_ensure_esp32_ready() )
	{
		wifi_display_msg("ESP32", "not ready!");
		/* Wait for BACK */
		while (1)
		{
			ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
			if (ret==pdTRUE && q_item.q_evt_type==Q_EVENT_KEYPAD)
			{
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				{
					xQueueReset(main_q_hdl);
					m1_esp32_deinit();
					return;
				}
			}
		}
	}

	/* Query IP/MAC from ESP32 */
	wifi_display_busy("Getting status...");

	memset(&ip_req, 0, sizeof(ip_req));
	ip_req.msg_type = CTRL_REQ;
	ip_req.cmd_timeout_sec = 10;
	wifi_get_ip(&ip_req);

	/* Display status */
	m1_u8g2_firstpage();
	u8g2_DrawXBMP(&m1_u8g2, 0, 0, 128, 14, m1_frame_128_14);
	u8g2_DrawStr(&m1_u8g2, 2, M1_GUI_ROW_SPACING + M1_GUI_FONT_HEIGHT, "WiFi Status");

	y_offset = 14 + M1_GUI_FONT_HEIGHT;

	if ( s_wifi_connected && s_connected_ssid[0] )
	{
		strncpy(prn_msg, s_connected_ssid, 20);
		prn_msg[20] = '\0';
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);
	}
	else
	{
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, "Not connected");
	}
	y_offset += M1_GUI_FONT_HEIGHT;

	if ( ip_req.u.wifi_ap_config.status[0]
		&& strcmp(ip_req.u.wifi_ap_config.status, "0.0.0.0") != 0 )
	{
		snprintf(prn_msg, sizeof(prn_msg), "IP:%s", ip_req.u.wifi_ap_config.status);
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, prn_msg);
		s_wifi_connected = true;
	}
	else
	{
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, "IP: N/A");
		s_wifi_connected = false;
	}
	y_offset += M1_GUI_FONT_HEIGHT;

	if ( ip_req.u.wifi_ap_config.out_mac[0] )
	{
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, ip_req.u.wifi_ap_config.out_mac);
	}
	y_offset += M1_GUI_FONT_HEIGHT;

	u8g2_DrawStr(&m1_u8g2, 2, y_offset, "BACK to exit");
	m1_u8g2_nextpage();

	/* Wait for BACK button */
	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE && q_item.q_evt_type==Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
			{
				xQueueReset(main_q_hdl);
				m1_esp32_deinit();
				return;
			}
		}
	}
} // void wifi_show_status(void)



/*============================================================================*/
/**
  * @brief Disconnect from current WiFi network
  */
/*============================================================================*/
void wifi_disconnect(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;

	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);

	if ( !wifi_ensure_esp32_ready() )
	{
		wifi_display_msg("ESP32", "not ready!");
		vTaskDelay(pdMS_TO_TICKS(2000));
		xQueueReset(main_q_hdl);
		return;
	}

	if ( !s_wifi_connected )
	{
		wifi_display_msg("WiFi", "Not connected");
		vTaskDelay(pdMS_TO_TICKS(2000));
		m1_esp32_deinit();
		xQueueReset(main_q_hdl);
		return;
	}

	wifi_display_busy("Disconnecting...");

	ctrl_cmd_t disc_req = CTRL_CMD_DEFAULT_REQ();
	disc_req.cmd_timeout_sec = 10;
	uint8_t result = wifi_disconnect_ap(&disc_req);

	if ( result == SUCCESS )
	{
		s_wifi_connected = false;
		s_connected_ssid[0] = '\0';
		wifi_display_msg("WiFi", "Disconnected");
		M1_LOG_I(M1_LOGDB_TAG, "WiFi disconnected\n\r");
	}
	else
	{
		wifi_display_msg("Disconnect", "failed!");
		M1_LOG_E(M1_LOGDB_TAG, "WiFi disconnect failed\n\r");
	}

	vTaskDelay(pdMS_TO_TICKS(2000));
	m1_esp32_deinit();

	/* Wait for any pending button press then return */
	xQueueReset(main_q_hdl);

	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE && q_item.q_evt_type==Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
			{
				xQueueReset(main_q_hdl);
				return;
			}
		}
	}
} // void wifi_disconnect(void)

#else /* M1_APP_WIFI_CONNECT_ENABLE not defined */

/*============================================================================*/
/**
  * @brief WiFi config - stub when connect feature not enabled
  */
/*============================================================================*/
void wifi_config(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;

	m1_gui_let_update_fw();

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
					xQueueReset(main_q_hdl);
					break;
				}
			}
		}
	}
} // void wifi_config(void)

#endif /* M1_APP_WIFI_CONNECT_ENABLE */
