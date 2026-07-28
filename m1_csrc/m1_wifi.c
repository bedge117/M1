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
#include "m1_esp_client.h"
#include "m1_esp_rpc_cfg.h"   /* M1_USE_ESP_RPC */
#include "ff.h"
//#include "control.h"
#include "ctrl_api.h"
#include "esp_app_main.h"
#include "m1_compile_cfg.h"

#ifdef M1_APP_WIFI_CONNECT_ENABLE
#include "m1_wifi_cred.h"
#include "m1_virtual_kb.h"
#if defined(M1_APP_RPC_ENABLE)
#include "m1_wifi_rpc.h"
#endif
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
static void wifi_display_busy(const char *msg);
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
/*
 * Deauth target flow (Stage 2): given a selected AP, scan its stations, let the
 * user choose a target (item 0 = "All / broadcast", 1..N = a client MAC), then
 * run deauth on that target until BACK. Self-contained event loop; returns to
 * the caller's AP list on BACK from the target list.
 */
/*============================================================================*/
static void deauth_ap_flow(const wifi_scanlist_t *ap)
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	uint8_t bssid[6];
	unsigned b[6];
	uint8_t stas[32][6];
	int8_t  sta_rssi[32];
	int     sta_count = 0;
	int     sel = 0;                 /* 0 = All(broadcast), 1..sta_count = client */
	bool    running = false;

	if ( sscanf((char *)ap->bssid, "%x:%x:%x:%x:%x:%x",
	            &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6 )
	{
		wifi_display_msg("Bad BSSID", "");
		return;
	}
	for ( int i = 0; i < 6; i++ ) bssid[i] = (uint8_t)b[i];

	/* Scan stations on this AP's channel */
	wifi_display_busy("Scanning clients");
	static uint8_t sbuf[512];
	int n = m1_esp_client_sta_scan(bssid, (uint8_t)ap->channel, 4, sbuf, sizeof(sbuf));
	if ( n >= 2 )
	{
		int total = sbuf[0] | (sbuf[1] << 8);
		int off = 2;
		for ( int i = 0; i < total && sta_count < 32 && off + 7 <= n; i++ )
		{
			memcpy(stas[sta_count], &sbuf[off], 6); off += 6;
			sta_rssi[sta_count] = (int8_t)sbuf[off++];
			sta_count++;
		}
	}

	for (;;)
	{
		if ( !running )
		{
			/* Draw the target list (windowed around sel) */
			char line[24];
			m1_u8g2_firstpage();
			u8g2_DrawStr(&m1_u8g2, 2, 12, "Deauth target:");
			int total_items = sta_count + 1;
			int top = sel < 3 ? 0 : sel - 2;
			for ( int row = 0; row < 4 && (top + row) < total_items; row++ )
			{
				int item = top + row;
				int y = 26 + row * 12;
				if ( item == sel ) u8g2_DrawStr(&m1_u8g2, 2, y, ">");
				if ( item == 0 )
					snprintf(line, sizeof(line), "All (broadcast)");
				else
					snprintf(line, sizeof(line), "%02X:%02X:%02X:%02X:%02X:%02X",
					         stas[item-1][0], stas[item-1][1], stas[item-1][2],
					         stas[item-1][3], stas[item-1][4], stas[item-1][5]);
				u8g2_DrawStr(&m1_u8g2, 10, y, line);
			}
			m1_u8g2_nextpage();
		}

		if ( xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY) != pdTRUE ||
		     q_item.q_evt_type != Q_EVENT_KEYPAD )
			continue;
		xQueueReceive(button_events_q_hdl, &btn, 0);

		if ( btn.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( running ) { m1_esp_client_deauth_stop(); running = false; continue; }
			return;                                   /* back to AP list */
		}
		if ( running ) continue;
		if ( btn.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( sel > 0 ) sel--;
		}
		else if ( btn.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( sel < sta_count ) sel++;
		}
		else if ( btn.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			uint8_t target[6];
			if ( sel == 0 ) memset(target, 0xFF, 6);        /* broadcast */
			else            memcpy(target, stas[sel-1], 6); /* specific client */

			if ( m1_esp_client_deauth_start(bssid, (uint8_t)ap->channel, target, 0, 0) )
			{
				running = true;
				m1_u8g2_firstpage();
				u8g2_DrawStr(&m1_u8g2, 4, 14, "Deauth running");
				u8g2_DrawStr(&m1_u8g2, 4, 30, (char *)ap->ssid);
				if ( sel == 0 )
					u8g2_DrawStr(&m1_u8g2, 4, 44, "target: ALL");
				else
				{
					char l[24];
					snprintf(l, sizeof(l), "%02X:%02X:%02X:%02X:%02X:%02X",
					         target[0], target[1], target[2], target[3], target[4], target[5]);
					u8g2_DrawStr(&m1_u8g2, 4, 44, l);
				}
				u8g2_DrawStr(&m1_u8g2, 4, 60, "BACK to stop");
				m1_u8g2_nextpage();
			}
			else
			{
				wifi_display_msg("Deauth", "failed");
			}
		}
	}
}

/*============================================================================*/
/*
 * WiFi Deauth menu: scan APs, pick one, then choose a client (or broadcast)
 * and deauth it. Uses the m1_link transport (deauth runs autonomously on the
 * ESP32). BACK unwinds: running -> target list -> AP list -> exit.
 */
/*============================================================================*/
void wifi_deauth_menu(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	ctrl_cmd_t app_req = CTRL_CMD_DEFAULT_REQ();
	uint16_t list_count = 0;
	wifi_scanlist_t *list;

	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	if ( !wifi_ensure_esp32_ready() )
	{
		wifi_display_msg("ESP32", "not ready!");
	}
	else
	{
		m1_u8g2_firstpage();
		u8g2_DrawStr(&m1_u8g2, 6, 15, "Scanning AP...");
		u8g2_DrawXBMP(&m1_u8g2, M1_LCD_DISPLAY_WIDTH/2 - 18/2, M1_LCD_DISPLAY_HEIGHT/2 - 2, 18, 32, hourglass_18x32);
		m1_u8g2_nextpage();

		app_req.cmd_timeout_sec = M1_WIFI_AP_SCANNING_TIME;
		app_req.msg_id = CTRL_RESP_GET_AP_SCAN_LIST;
		wifi_ap_scan_list(&app_req);
		if ( wifi_ap_list_validation(&app_req) )
			list_count = wifi_ap_list_print(&app_req, true);
		else
			wifi_display_msg("Scan failed", "BACK to exit");
	}

	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if ( ret != pdTRUE || q_item.q_evt_type != Q_EVENT_KEYPAD )
			continue;
		xQueueReceive(button_events_q_hdl, &this_button_status, 0);

		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( app_req.u.wifi_ap_scan.out_list != NULL )
				free(app_req.u.wifi_ap_scan.out_list);
			wifi_ap_list_print(NULL, false);
			xQueueReset(main_q_hdl);
			m1_esp32_deinit();
			break;                                           /* exit menu */
		}
		else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( list_count ) wifi_ap_list_print(&app_req, true);
		}
		else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( list_count ) wifi_ap_list_print(&app_req, false);
		}
		else if ( this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( !list_count ) continue;
			list = app_req.u.wifi_ap_scan.out_list;
			uint16_t idx = wifi_ap_list_get_index();
			if ( idx >= app_req.u.wifi_ap_scan.count ) continue;

			/* Scan clients on this AP and deauth a chosen target. */
			deauth_ap_flow(&list[idx]);

			/* Redraw the AP list on return. */
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			wifi_ap_list_print(NULL, false);
			list_count = wifi_ap_list_print(&app_req, true);
		}
	} // while (1)
} // void wifi_deauth_menu(void)


/*============================================================================*/
/* Stage 3: Handshake (EAPOL) capture. Save the captured PCAP to SD in chunks. */
/*============================================================================*/
static int hs_save_to_sd(const char *ssid, uint32_t total_len)
{
	f_mkdir("0:/wifi");
	char safe[24];
	int j = 0;
	for ( int i = 0; ssid[i] && j < (int)sizeof(safe) - 1; i++ )
	{
		char c = ssid[i];
		safe[j++] = ((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')) ? c : '_';
	}
	safe[j] = 0;
	if ( !j ) strcpy(safe, "cap");

	char path[48];
	snprintf(path, sizeof(path), "0:/wifi/hs_%s.pcap", safe);
	FIL fh;
	if ( f_open(&fh, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK )
		return -1;

	uint8_t buf[502];
	uint32_t off = 0;
	UINT bw;
	while ( off < total_len )
	{
		uint16_t want = (total_len - off) > sizeof(buf) ? sizeof(buf) : (uint16_t)(total_len - off);
		int n = m1_esp_client_hs_read(off, buf, want);
		if ( n <= 0 ) break;
		if ( f_write(&fh, buf, (UINT)n, &bw) != FR_OK ) break;
		off += n;
	}
	f_close(&fh);
	return (int)off;
}

static void handshake_capture_flow(const wifi_scanlist_t *ap)
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	uint8_t bssid[6];
	unsigned b[6];

	if ( sscanf((char *)ap->bssid, "%x:%x:%x:%x:%x:%x",
	            &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6 )
	{
		wifi_display_msg("Bad BSSID", "");
		return;
	}
	for ( int i = 0; i < 6; i++ ) bssid[i] = (uint8_t)b[i];

	/* Start capture + a burst of deauths to force a reconnect handshake. */
	if ( !m1_esp_client_hs_start(bssid, (uint8_t)ap->channel, 32) )
	{
		wifi_display_msg("HS start", "failed");
		return;
	}

	uint8_t state = 0;
	uint32_t total = 0;
	int secs = 0;
	bool complete = false;
	while ( secs < 30 )
	{
		if ( xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(700)) == pdTRUE
		     && q_item.q_evt_type == Q_EVENT_KEYPAD )
		{
			xQueueReceive(button_events_q_hdl, &btn, 0);
			if ( btn.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
			{
				m1_esp_client_hs_stop();
				return;
			}
		}
		m1_esp_client_hs_status(&state, &total);
		secs++;

		char l[24];
		m1_u8g2_firstpage();
		u8g2_DrawStr(&m1_u8g2, 4, 14, "Handshake capture");
		u8g2_DrawStr(&m1_u8g2, 4, 30, (char *)ap->ssid);
		snprintf(l, sizeof(l), "wait %ds  %lub", secs, (unsigned long)total);
		u8g2_DrawStr(&m1_u8g2, 4, 44, l);
		u8g2_DrawStr(&m1_u8g2, 4, 60, "BACK to cancel");
		m1_u8g2_nextpage();

		if ( state == 2 ) { complete = true; break; }
	}
	m1_esp_client_hs_stop();

	if ( complete )
	{
		int saved = hs_save_to_sd((const char *)ap->ssid, total);
		char l[24];
		if ( saved > 0 ) snprintf(l, sizeof(l), "Saved %d bytes", saved);
		else             snprintf(l, sizeof(l), "SD save failed");
		wifi_display_msg("Handshake!", l);
	}
	else
	{
		wifi_display_msg("No handshake", "timed out");
	}
	/* brief pause so the result is readable */
	xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(2500));
}

void wifi_handshake_menu(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	ctrl_cmd_t app_req = CTRL_CMD_DEFAULT_REQ();
	uint16_t list_count = 0;
	wifi_scanlist_t *list;

	u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
	if ( !wifi_ensure_esp32_ready() )
	{
		wifi_display_msg("ESP32", "not ready!");
	}
	else
	{
		wifi_display_busy("Scanning AP");
		app_req.cmd_timeout_sec = M1_WIFI_AP_SCANNING_TIME;
		app_req.msg_id = CTRL_RESP_GET_AP_SCAN_LIST;
		wifi_ap_scan_list(&app_req);
		if ( wifi_ap_list_validation(&app_req) )
			list_count = wifi_ap_list_print(&app_req, true);
		else
			wifi_display_msg("Scan failed", "BACK to exit");
	}

	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if ( ret != pdTRUE || q_item.q_evt_type != Q_EVENT_KEYPAD )
			continue;
		xQueueReceive(button_events_q_hdl, &this_button_status, 0);

		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( app_req.u.wifi_ap_scan.out_list != NULL )
				free(app_req.u.wifi_ap_scan.out_list);
			wifi_ap_list_print(NULL, false);
			xQueueReset(main_q_hdl);
			m1_esp32_deinit();
			break;
		}
		else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( list_count ) wifi_ap_list_print(&app_req, true);
		}
		else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( list_count ) wifi_ap_list_print(&app_req, false);
		}
		else if ( this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( !list_count ) continue;
			list = app_req.u.wifi_ap_scan.out_list;
			uint16_t idx = wifi_ap_list_get_index();
			if ( idx >= app_req.u.wifi_ap_scan.count ) continue;

			handshake_capture_flow(&list[idx]);

			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			wifi_ap_list_print(NULL, false);
			list_count = wifi_ap_list_print(&app_req, true);
		}
	}
} // void wifi_handshake_menu(void)


/*============================================================================*/
/* Stage 4: Beacon spam — fake APs from a typed SSID or 0:/wifi/ap_lists.txt.  */
/*============================================================================*/
static int beacon_read_ap_list(char ssids[][33], int max)
{
	FIL fh;
	UINT br;
	static char fbuf[512];
	if ( f_open(&fh, "0:/wifi/ap_lists.txt", FA_READ | FA_OPEN_EXISTING) != FR_OK )
		return 0;
	if ( f_read(&fh, fbuf, sizeof(fbuf) - 1, &br) != FR_OK ) { f_close(&fh); return 0; }
	f_close(&fh);

	int n = 0, col = 0;
	for ( UINT i = 0; i <= br && n < max; i++ )
	{
		char c = (i < br) ? fbuf[i] : '\n';
		if ( c == '\n' || c == '\r' )
		{
			if ( col > 0 ) { ssids[n][col] = 0; n++; col = 0; }
		}
		else if ( col < 32 )
		{
			ssids[n][col++] = c;
		}
	}
	return n;
}

void wifi_beacon_menu(void)
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	static char ssids[16][33];
	int count = 0;
	int sel = 0;                 /* 0 = User SSID, 1 = From SD list */
	bool running = false;

	if ( !wifi_ensure_esp32_ready() )
		wifi_display_msg("ESP32", "not ready!");

	for (;;)
	{
		if ( !running )
		{
			m1_u8g2_firstpage();
			u8g2_DrawStr(&m1_u8g2, 2, 12, "Beacon spam:");
			u8g2_DrawStr(&m1_u8g2, 10, 30, "User SSID");
			u8g2_DrawStr(&m1_u8g2, 10, 44, "From SD (ap_lists)");
			u8g2_DrawStr(&m1_u8g2, 2, sel == 0 ? 30 : 44, ">");
			m1_u8g2_nextpage();
		}

		if ( xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY) != pdTRUE
		     || q_item.q_evt_type != Q_EVENT_KEYPAD )
			continue;
		xQueueReceive(button_events_q_hdl, &btn, 0);

		if ( btn.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( running ) { m1_esp_client_beacon_stop(); running = false; continue; }
			xQueueReset(main_q_hdl);
			m1_esp32_deinit();
			return;
		}
		if ( running ) continue;

		if ( btn.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )      { if ( sel > 0 ) sel--; }
		else if ( btn.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK ) { if ( sel < 1 ) sel++; }
		else if ( btn.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			count = 0;
			if ( sel == 0 )
			{
				char one[33] = {0};
				if ( m1_vkb_get_filename("Beacon SSID:", "", one) > 0 && one[0] )
				{
					strncpy(ssids[0], one, 32); ssids[0][32] = 0;
					count = 1;
				}
			}
			else
			{
				count = beacon_read_ap_list(ssids, 11);
				if ( count == 0 ) { wifi_display_msg("ap_lists.txt", "not found"); continue; }
			}

			if ( count > 0 && m1_esp_client_beacon_start(ssids, (uint8_t)count) )
			{
				running = true;
				char l[24];
				snprintf(l, sizeof(l), "%d fake APs", count);
				m1_u8g2_firstpage();
				u8g2_DrawStr(&m1_u8g2, 4, 20, "Beacon running");
				u8g2_DrawStr(&m1_u8g2, 4, 38, l);
				u8g2_DrawStr(&m1_u8g2, 4, 58, "BACK to stop");
				m1_u8g2_nextpage();
			}
			else if ( count > 0 )
			{
				wifi_display_msg("Beacon", "failed");
			}
		}
	}
} // void wifi_beacon_menu(void)


/*============================================================================*/
/* BLE Spam — proximity-pair popup flooder (runs autonomously on the ESP).
 * Lives here (not m1_bt.c) because it uses the same m1_link RPC + ESP-ready
 * plumbing as the other offensive ESP features. Exposed in the Bluetooth menu. */
/*============================================================================*/
void ble_spam_menu(void)
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	static const char *modes[5] = { "Apple", "Google", "Samsung", "Windows", "All / Random" };
	int sel = 0;
	bool running = false;

	if ( !wifi_ensure_esp32_ready() )
		wifi_display_msg("ESP32", "not ready!");

	for (;;)
	{
		if ( !running )
		{
			m1_u8g2_firstpage();
			u8g2_DrawStr(&m1_u8g2, 2, 12, "BLE Spam:");
			for ( int i = 0; i < 5; i++ )
				u8g2_DrawStr(&m1_u8g2, 12, 24 + i * 9, modes[i]);
			u8g2_DrawStr(&m1_u8g2, 3, 24 + sel * 9, ">");
			m1_u8g2_nextpage();
		}

		if ( xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY) != pdTRUE
		     || q_item.q_evt_type != Q_EVENT_KEYPAD )
			continue;
		xQueueReceive(button_events_q_hdl, &btn, 0);

		if ( btn.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( running ) { m1_esp_client_ble_spam_stop(); running = false; continue; }
			xQueueReset(main_q_hdl);
			m1_esp32_deinit();
			return;
		}
		if ( running ) continue;

		if ( btn.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )        { if ( sel > 0 ) sel--; }
		else if ( btn.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK ) { if ( sel < 4 ) sel++; }
		else if ( btn.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( m1_esp_client_ble_spam_start((uint8_t)sel) )
			{
				running = true;
				m1_u8g2_firstpage();
				u8g2_DrawStr(&m1_u8g2, 4, 20, "BLE Spam running");
				u8g2_DrawStr(&m1_u8g2, 4, 38, modes[sel]);
				u8g2_DrawStr(&m1_u8g2, 4, 58, "BACK to stop");
				m1_u8g2_nextpage();
			}
			else
			{
				wifi_display_msg("BLE Spam", "failed");
			}
		}
	}
} // void ble_spam_menu(void)


/*============================================================================*/
/* Probe Flood + Karma — channel-picker driven ESP offensive features.
 * Shared helper: pick a channel, OK starts (via start_fn), BACK stops/exits. */
/*============================================================================*/
static bool probe_start_bcast(uint8_t ch)   /* wildcard broadcast probe flood */
{
	return m1_esp_client_probe_start(ch, NULL, 0);
}

static void wifi_channel_attack_menu(const char *title,
                                     bool (*start_fn)(uint8_t),
                                     bool (*stop_fn)(void))
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	static const struct { const char *label; uint8_t ch; } chans[] = {
		{ "Channel 1", 1 }, { "Channel 6", 6 }, { "Channel 11", 11 },
	};
	const int nchan = 3;
	int sel = 0;
	bool running = false, redraw = true;

	if ( !wifi_ensure_esp32_ready() )
		wifi_display_msg("ESP32", "not ready!");

	for (;;)
	{
		if ( redraw && !running )
		{
			redraw = false;
			m1_u8g2_firstpage();
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			u8g2_DrawStr(&m1_u8g2, 2, 12, title);
			for ( int i = 0; i < nchan; i++ )
				u8g2_DrawStr(&m1_u8g2, 12, 26 + i * 12, chans[i].label);
			u8g2_DrawStr(&m1_u8g2, 3, 26 + sel * 12, ">");
			m1_u8g2_nextpage();
		}

		if ( xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY) != pdTRUE
		     || q_item.q_evt_type != Q_EVENT_KEYPAD )
			continue;
		xQueueReceive(button_events_q_hdl, &btn, 0);

		if ( btn.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( running ) { stop_fn(); running = false; redraw = true; continue; }
			stop_fn();                       /* belt-and-braces */
			xQueueReset(main_q_hdl);
			m1_esp32_deinit();
			return;
		}
		if ( running )
			continue;

		if ( btn.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )        { sel = (sel > 0) ? sel - 1 : nchan - 1; redraw = true; }
		else if ( btn.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK ) { sel = (sel < nchan - 1) ? sel + 1 : 0; redraw = true; }
		else if ( btn.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( start_fn(chans[sel].ch) )
			{
				running = true;
				m1_u8g2_firstpage();
				u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
				u8g2_DrawStr(&m1_u8g2, 4, 20, title);
				u8g2_DrawStr(&m1_u8g2, 4, 38, chans[sel].label);
				u8g2_DrawStr(&m1_u8g2, 4, 58, "BACK to stop");
				m1_u8g2_nextpage();
			}
			else
			{
				wifi_display_msg(title, "failed");
			}
		}
	}
}

void wifi_probe_flood_menu(void)
{
	wifi_channel_attack_menu("Probe Flood", probe_start_bcast, m1_esp_client_probe_stop);
}

void wifi_karma_menu(void)
{
	wifi_channel_attack_menu("Karma", m1_esp_client_karma_start, m1_esp_client_karma_stop);
}


/*============================================================================*/
/* Packet Monitor — promiscuous 802.11 sniffer (runs on the ESP; the M1 polls
 * live capture stats). UP/DOWN pick the channel (0 = hop 1-13), OK starts, BACK
 * stops then exits.                                                            */
/*============================================================================*/
void wifi_monitor_menu(void)
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	int  ch = 6;            /* 0 = hop across 1-13 */
	bool running = false;
	bool redraw = true;

	if ( !wifi_ensure_esp32_ready() )
		wifi_display_msg("ESP32", "not ready!");

	for (;;)
	{
		if ( redraw && !running )
		{
			char l[24];
			m1_u8g2_firstpage();
			u8g2_DrawStr(&m1_u8g2, 2, 12, "Packet Monitor");
			if ( ch == 0 ) snprintf(l, sizeof(l), "Channel: HOP");
			else           snprintf(l, sizeof(l), "Channel: %d", ch);
			u8g2_DrawStr(&m1_u8g2, 2, 30, l);
			u8g2_DrawStr(&m1_u8g2, 2, 46, "UP/DN chan  OK start");
			u8g2_DrawStr(&m1_u8g2, 2, 60, "BACK exit");
			m1_u8g2_nextpage();
			redraw = false;
		}

		BaseType_t got = xQueueReceive(main_q_hdl, &q_item,
		                               running ? pdMS_TO_TICKS(500) : portMAX_DELAY);

		if ( running && got != pdTRUE )
		{
			uint32_t total = 0, dropped = 0; uint8_t buffered = 0;
			m1_esp_client_monitor_stats(&total, &dropped, &buffered);
			char a[24], b[24];
			snprintf(a, sizeof(a), "packets: %lu", (unsigned long)total);
			snprintf(b, sizeof(b), "drop:%lu buf:%u", (unsigned long)dropped, buffered);
			m1_u8g2_firstpage();
			u8g2_DrawStr(&m1_u8g2, 2, 12, ch == 0 ? "Monitor: HOP" : "Monitor running");
			u8g2_DrawStr(&m1_u8g2, 2, 32, a);
			u8g2_DrawStr(&m1_u8g2, 2, 46, b);
			u8g2_DrawStr(&m1_u8g2, 2, 60, "BACK to stop");
			m1_u8g2_nextpage();
			continue;
		}
		if ( got != pdTRUE || q_item.q_evt_type != Q_EVENT_KEYPAD ) continue;
		xQueueReceive(button_events_q_hdl, &btn, 0);

		if ( btn.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( running ) { m1_esp_client_monitor_stop(); running = false; redraw = true; continue; }
			xQueueReset(main_q_hdl);
			m1_esp32_deinit();
			return;
		}
		if ( running ) continue;

		if ( btn.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )        { ch = (ch >= 13) ? 0 : ch + 1; redraw = true; }
		else if ( btn.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK ) { ch = (ch <= 0)  ? 13 : ch - 1; redraw = true; }
		else if ( btn.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			int e = m1_esp_client_monitor_start((uint8_t)ch);
			if ( e == 0 ) running = true;
			else { char m[20]; snprintf(m, sizeof(m), "err %d", e); wifi_display_msg("Monitor", m); redraw = true; }
		}
	}
} // void wifi_monitor_menu(void)


/* Append captured creds to 0:/wifi/captive_creds.txt (created if missing, so
 * nothing is lost across sessions). Returns bytes written, or -1 on error. */
static int captive_save_creds(const char *ssid, char us[][48], char ps[][48], int cnt)
{
	FIL fh; UINT bw; char line[128];
	f_mkdir("0:/wifi");   /* FR_EXIST is fine */
	if ( f_open(&fh, "0:/wifi/captive_creds.txt", FA_WRITE | FA_OPEN_ALWAYS) != FR_OK )
		return -1;
	f_lseek(&fh, f_size(&fh));    /* append */
	int ln = snprintf(line, sizeof(line), "--- SSID: %.32s ---\r\n", ssid ? ssid : "");
	f_write(&fh, line, (UINT)ln, &bw);
	for ( int i = 0; i < cnt; i++ )
	{
		ln = snprintf(line, sizeof(line), "%s:%s\r\n", us[i], ps[i]);
		f_write(&fh, line, (UINT)ln, &bw);
	}
	f_close(&fh);
	return cnt;
}

/* Show captured credentials — username/password on their own lines so full
 * values are visible. UP/DOWN pages, OK saves to SD, BACK exits. */
static void captive_show_creds(const char *ssid)
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	static char us[16][48], ps[16][48];
	uint8_t buf[512];
	int n = m1_esp_client_captive_creds(buf, sizeof(buf));
	uint8_t cnt = (n >= 1) ? buf[0] : 0;

	/* Parse all creds we received into arrays. */
	int parsed = 0, o = 1;
	for ( uint8_t i = 0; i < cnt && parsed < 16; i++ )
	{
		if ( o + 6 > n ) break;
		o += 4;                                       /* skip timestamp */
		uint8_t ul = buf[o++]; uint8_t uc = ul < 47 ? ul : 47;
		memcpy(us[parsed], &buf[o], uc); us[parsed][uc] = 0; o += ul;
		uint8_t pl = buf[o++]; uint8_t pc = pl < 47 ? pl : 47;
		memcpy(ps[parsed], &buf[o], pc); ps[parsed][pc] = 0; o += pl;
		parsed++;
	}

	int page = parsed > 0 ? parsed - 1 : 0;   /* start on the most recent */
	for (;;)
	{
		m1_u8g2_firstpage();
		char hdr[24];
		snprintf(hdr, sizeof(hdr), "Creds %d/%d", parsed ? page + 1 : 0, parsed);
		u8g2_DrawStr(&m1_u8g2, 2, 10, hdr);
		if ( parsed == 0 )
		{
			u8g2_DrawStr(&m1_u8g2, 2, 30, "(none captured yet)");
		}
		else
		{
			char l1[28], l2[28];
			snprintf(l1, sizeof(l1), "%.21s", us[page]);
			snprintf(l2, sizeof(l2), "%.21s", ps[page]);
			u8g2_DrawStr(&m1_u8g2, 2, 24, "user:");
			u8g2_DrawStr(&m1_u8g2, 2, 34, l1);
			u8g2_DrawStr(&m1_u8g2, 2, 48, "pass:");
			u8g2_DrawStr(&m1_u8g2, 2, 58, l2);
		}
		u8g2_DrawStr(&m1_u8g2, 78, 10, "OK=save");
		m1_u8g2_nextpage();

		if ( xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY) != pdTRUE
		     || q_item.q_evt_type != Q_EVENT_KEYPAD ) continue;
		xQueueReceive(button_events_q_hdl, &btn, 0);

		if ( btn.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
			return;
		if ( btn.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK && parsed > 0 )
		{
			int r = captive_save_creds(ssid, us, ps, parsed);
			m1_u8g2_firstpage();
			if ( r >= 0 )
			{
				char m[26]; snprintf(m, sizeof(m), "Saved %d creds", r);
				u8g2_DrawStr(&m1_u8g2, 2, 26, m);
				u8g2_DrawStr(&m1_u8g2, 2, 42, "wifi/captive_creds.txt");
			}
			else
			{
				u8g2_DrawStr(&m1_u8g2, 2, 32, "Save FAILED (SD?)");
			}
			m1_u8g2_nextpage();
			vTaskDelay(pdMS_TO_TICKS(1400));
		}
		else if ( btn.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )   { if ( page > 0 ) page--; }
		else if ( btn.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK ) { if ( page < parsed - 1 ) page++; }
	}
}

/*============================================================================*/
/* Captive Portal — rogue open AP + credential capture (runs on the ESP). Pick
 * "Start" or edit the SSID; while running, live client/cred counts, OK views
 * captured creds, BACK stops then exits.                                       */
/*============================================================================*/
void wifi_captive_menu(void)
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	char ssid[33] = "Free WiFi";
	int  sel = 0;           /* 0 = Start, 1 = Edit SSID */
	bool running = false;
	bool redraw = true;

	if ( !wifi_ensure_esp32_ready() )
		wifi_display_msg("ESP32", "not ready!");

	for (;;)
	{
		if ( redraw && !running )
		{
			char l[28];
			m1_u8g2_firstpage();
			u8g2_DrawStr(&m1_u8g2, 2, 12, "Captive Portal");
			snprintf(l, sizeof(l), "SSID: %.18s", ssid);
			u8g2_DrawStr(&m1_u8g2, 2, 26, l);
			u8g2_DrawStr(&m1_u8g2, 12, 42, "Start portal");
			u8g2_DrawStr(&m1_u8g2, 12, 56, "Edit SSID");
			u8g2_DrawStr(&m1_u8g2, 2, sel == 0 ? 42 : 56, ">");
			m1_u8g2_nextpage();
			redraw = false;
		}

		BaseType_t got = xQueueReceive(main_q_hdl, &q_item,
		                               running ? pdMS_TO_TICKS(700) : portMAX_DELAY);

		if ( running && got != pdTRUE )
		{
			uint32_t dns_q = 0, http = 0; uint8_t clients = 0;
			char lp[80] = {0};
			m1_esp_client_captive_diag(&dns_q, &http, &clients, lp, sizeof(lp));
			uint8_t cbuf[64];
			int cn = m1_esp_client_captive_creds(cbuf, sizeof(cbuf));
			uint8_t creds = (cn >= 1) ? cbuf[0] : 0;
			char a[26], b[26];
			snprintf(a, sizeof(a), "clients:%u creds:%u", clients, creds);
			snprintf(b, sizeof(b), "dns:%lu http:%lu", (unsigned long)dns_q, (unsigned long)http);
			m1_u8g2_firstpage();
			u8g2_DrawStr(&m1_u8g2, 2, 11, "Portal running");
			u8g2_DrawStr(&m1_u8g2, 2, 24, ssid);
			u8g2_DrawStr(&m1_u8g2, 2, 38, a);
			u8g2_DrawStr(&m1_u8g2, 2, 50, b);
			u8g2_DrawStr(&m1_u8g2, 2, 62, "OK view creds  BACK stop");
			m1_u8g2_nextpage();
			continue;
		}
		if ( got != pdTRUE || q_item.q_evt_type != Q_EVENT_KEYPAD ) continue;
		xQueueReceive(button_events_q_hdl, &btn, 0);

		if ( btn.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( running ) { m1_esp_client_captive_stop(); running = false; redraw = true; continue; }
			xQueueReset(main_q_hdl);
			m1_esp32_deinit();
			return;
		}

		if ( running )
		{
			if ( btn.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
				captive_show_creds(ssid);
			continue;
		}

		if ( btn.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )        { if ( sel > 0 ) { sel--; redraw = true; } }
		else if ( btn.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK ) { if ( sel < 1 ) { sel++; redraw = true; } }
		else if ( btn.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if ( sel == 1 )
			{
				char one[33] = {0};
				if ( m1_vkb_get_filename("Portal SSID:", ssid, one) > 0 && one[0] )
				{ strncpy(ssid, one, 32); ssid[32] = 0; }
				redraw = true;
			}
			else
			{
				int e = m1_esp_client_captive_start(ssid, ssid, 1);
				if ( e == 0 ) running = true;
				else { char m[20]; snprintf(m, sizeof(m), "err %d", e); wifi_display_msg("Portal", m); redraw = true; }
			}
		}
	}
} // void wifi_captive_menu(void)



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
#if defined(M1_APP_RPC_ENABLE)
		wifi_rpc_notify_wifi_connected();
#endif
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

	/* Refresh cached connection state once on entry: a single status poll so a
	 * silently-dropped link isn't drawn as "connected". If the ESP reports it is
	 * not associated, drop the stale SSID so the OK label falls back to "Connect". */
	{
		uint8_t st_connected = 0;
		uint8_t st_ip[4] = { 0 };
		if ( m1_esp_client_wifi_status(&st_connected, st_ip) && !st_connected )
			s_connected_ssid[0] = '\0';
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

		/* Instructions — OK action is dynamic vs the current association state:
		 * connected to the selected SSID -> Disconnect, connected elsewhere ->
		 * Switch, otherwise Connect. */
		const char *ok_label = "OK: Connect";
		if ( m1_esp_client_wifi_is_connected() && s_connected_ssid[0] )
		{
			if ( strncmp(s_connected_ssid, creds[sel_idx].ssid, SSID_LENGTH - 1) == 0 )
				ok_label = "OK: Disconnect";
			else
				ok_label = "OK: Switch";
		}
		u8g2_DrawStr(&m1_u8g2, 2, y_offset, ok_label);
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
				/* Decide the action from the current association vs the selected
				 * SSID. NEVER re-issue a raw connect to the SSID we're already on:
				 * that tears down and re-associates the link on the ESP side and
				 * wedges WiFi until reboot. */
				bool sel_is_current = ( m1_esp_client_wifi_is_connected()
					&& s_connected_ssid[0]
					&& strncmp(s_connected_ssid, creds[sel_idx].ssid, SSID_LENGTH - 1) == 0 );

				u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
				if ( sel_is_current )
				{
					/* Already on this network -> OK means Disconnect */
					wifi_disconnect();
				}
				else if ( !wifi_ensure_esp32_ready() )
				{
					wifi_display_msg("ESP32", "not ready!");
					vTaskDelay(pdMS_TO_TICKS(2000));
				}
				else
				{
					/* Connect, or Switch from a different SSID: the client layer
					 * disconnect-then-connects when already associated elsewhere. */
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


/*============================================================================*/
/**
  * @brief Sync system RTC with WiFi NTP
  * @retval 1 on success, 0 on failure
  */
/*============================================================================*/
uint8_t wifi_sync_rtc(void)
{
	m1_time_t dt;

	if ( !wifi_ensure_esp32_ready() ) return 0;

	/* Option C: native ESP SNTP over m1_link (the AT+CIPSNTP path below would
	 * crash on the uninitialized AT transport). */
	{
		m1esp_time_t et;
		if ( m1_esp_client_sntp_sync(&et) )
		{
			dt.year    = et.year;
			dt.month   = et.month;
			dt.day     = et.day;
			dt.hour    = et.hour;
			dt.minute  = et.minute;
			dt.second  = et.second;
			/* ESP weekday 0=Sun..6=Sat -> M1 convention Mon=1..Sun=7 */
			dt.weekday = (et.weekday == 0) ? 7 : et.weekday;
			m1_set_datetime(&dt);
			return 1;
		}
		return 0;
	}
}


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
