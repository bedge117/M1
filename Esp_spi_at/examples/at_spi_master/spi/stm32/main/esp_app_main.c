/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stream_buffer.h"

//#include "esp_system.h"
#include "esp_err.h"
#include "m1_log_debug.h"
#include "ctrl_api.h"        /* ctrl_cmd_t + wifi/ble scanlist types (was pulled via spi_master.h) */
#include "m1_esp_rpc_cfg.h"
#ifdef M1_USE_ESP_RPC
#include "m1_esp_client.h"
#include "m1_esp_rpc.h"
#include "m1_link.h"          /* m1_link_bc() bring-up breadcrumbs */
#include <stdlib.h>
#endif
#include "m1_tasks.h"
#include "m1_compile_cfg.h"
#include "m1_esp32_hal.h"
#include "m1_io_defs.h"

#define TAG						"SPI_AT_Master"

/* Legacy AT-transport handshake ISR (ESP32_GPIO_EXTI_Callback in m1_int_hdl.c)
 * still references this queue handle. Under Option C (M1_USE_ESP_RPC) it is
 * never created, so it stays NULL and that ISR path is a guarded no-op. Kept
 * only to satisfy the external reference. */
QueueHandle_t esp_spi_msg_queue = NULL;

static bool esp32_main_init_done = false;


bool get_esp32_main_init_status(void)
{
	return esp32_main_init_done;
} // bool get_esp32_main_init_status(void)


/**
  * @brief Delay without context switch
  * @param  x in ms approximately
  * @retval None
  */
void hard_delay(uint32_t x)
{
    volatile uint32_t idx;

    for (idx=0; idx<6000*x; idx++) // 100
    {
    	;
    }
}


/**
  * @brief  Reset slave to initialize
  * @param  None
  * @retval None
  */
static void reset_slave(void)
{
	esp32_disable();
	hard_delay(1); // 50
	esp32_enable();
	/* Brief delay for ESP32-C6 SPI slave to initialize after reset.
	 * Stock firmware used hard_delay(200) here (~200ms busy-loop).
	 * The handshake pin check in esp32_main_init() catches missed events. */
	HAL_Delay(200);
}



void esp32_main_init(void)
{
	if ( esp32_main_init_done )
		return;

	/* Option C: the m1_link full-duplex transport owns the ESP link. SPI3 +
	 * HANDSHAKE/DATAREADY EXTI + m1_esp_client_init() are already set up in
	 * m1_esp32_init(). BUT the ESP still needs a real RESET pulse to (re)boot
	 * into its native m1_link slave firmware and start raising HANDSHAKE —
	 * m1_esp32_init() only drives EN high (esp32_enable), it never pulses reset.
	 * reset_slave() (EN low -> settle -> EN high -> 200ms) provides that boot.
	 * We deliberately DO NOT run the legacy AT bring-up that used to follow it
	 * (init_master_hd()'s ~10s poll of an AT slave + spi_trans_control_task,
	 * which clocks the bus with AT framing that collides with m1_link). */
	reset_slave();
	/* reset_slave() only settles 200ms; give the native ESP-IDF image the rest
	 * of its boot (bootloader + app_main + spi_slave arm + first HANDSHAKE raise)
	 * before the first m1_link request, so the very first scan isn't a timing miss. */
	HAL_Delay(600);
	esp32_main_init_done = true;
} // void esp32_main_init(void)


static void esp_free_mem( char **buf_ptr)
{
	if ( *buf_ptr != NULL )
	{
		free (*buf_ptr);
		*buf_ptr = NULL;
	}
} // static void esp_free_mem( char **buf_ptr)


uint8_t wifi_ap_scan_list(ctrl_cmd_t *app_req)
{
	{
		/* Re-pointed onto the m1_link transport. NOTE: one 512B frame today
		 * (no fragmentation yet) => up to ~14 APs; larger needs M1ESP_FRAG. */
		static uint8_t scanbuf[M1L_MTU];
		m1_link_bc(8, 0xA0);                 /* breadcrumb: entering scan, before m1_link */
		int n = m1_esp_client_wifi_scan(scanbuf, sizeof(scanbuf));
		m1_link_bc(8, 0xA1);                 /* breadcrumb: m1_link returned */
		m1_link_bc(13, (uint32_t)n);         /* BKP13R = scan return n */
		if ( n < 2 ) { M1_LOG_E(TAG, "rpc scan fail (%d)\r\n", n); return ERROR; }

		uint16_t total = (uint16_t)scanbuf[0] | ((uint16_t)scanbuf[1] << 8);
		uint16_t off = 2;
		wifi_scanlist_t *list = NULL;
		m1_link_bc(8, 0xA2);                 /* breadcrumb: about to calloc(total) */
		m1_link_bc(9, (uint32_t)total);      /* BKP9R = total AP count from ESP */
		if ( total )
		{
			list = (wifi_scanlist_t *)calloc(total, sizeof(wifi_scanlist_t));
			if ( !list ) return ERROR;
		}
		m1_link_bc(8, 0xA3);                 /* breadcrumb: calloc done, parsing entries */
		uint16_t filled = 0;
		for ( uint16_t i = 0; i < total; i++ )
		{
			if ( off + sizeof(m1esp_scan_entry_t) > (uint16_t)n ) break;
			m1esp_scan_entry_t e;
			memcpy(&e, &scanbuf[off], sizeof(e));
			off += sizeof(e);
			if ( off + e.ssid_len > (uint16_t)n ) break;
			uint8_t cl = e.ssid_len < (SSID_LENGTH - 1) ? e.ssid_len : (SSID_LENGTH - 1);
			memcpy(list[filled].ssid, &scanbuf[off], cl);
			list[filled].ssid[cl] = 0;
			off += e.ssid_len;
			snprintf((char *)list[filled].bssid, BSSID_STR_SIZE,
			         "%02X:%02X:%02X:%02X:%02X:%02X",
			         e.bssid[0], e.bssid[1], e.bssid[2], e.bssid[3], e.bssid[4], e.bssid[5]);
			list[filled].rssi = (int)e.rssi;
			list[filled].channel = (int)e.channel;
			list[filled].encryption_mode = (int)e.authmode;
			filled++;
		}
		m1_link_bc(8, 0xA4);                 /* breadcrumb: parse loop done, returning OK */
		app_req->u.wifi_ap_scan.count = filled;
		app_req->u.wifi_ap_scan.out_list = list;
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
		return SUCCESS;
	}
} // uint8_t wifi_ap_scan_list(ctrl_cmd_t *app_req)



#ifdef M1_USE_ESP_RPC
/* Shared BLE-scan re-point: scan via m1_link, map into ble_scanlist_t. */
static uint8_t m1esp_ble_scan_fill(ctrl_cmd_t *app_req)
{
	static uint8_t sbuf[M1L_MTU];
	int n = m1_esp_client_ble_scan(sbuf, sizeof(sbuf), 3);
	if ( n < 2 )
	{
		app_req->u.ble_scan.count = 0;
		app_req->u.ble_scan.out_list = NULL;
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = (n < 0) ? ERROR : SUCCESS;
		return (n < 0) ? ERROR : SUCCESS;
	}
	uint16_t total = (uint16_t)sbuf[0] | ((uint16_t)sbuf[1] << 8);
	uint16_t off = 2;
	ble_scanlist_t *list = total ? (ble_scanlist_t *)calloc(total, sizeof(ble_scanlist_t)) : NULL;
	if ( total && !list ) return ERROR;
	uint16_t filled = 0;
	for ( uint16_t i = 0; i < total; i++ )
	{
		if ( off + 6 + 1 + 1 + 1 > (uint16_t)n ) break;
		uint8_t a[6];
		memcpy(a, &sbuf[off], 6); off += 6;
		uint8_t at = sbuf[off++];
		int8_t rs = (int8_t)sbuf[off++];
		uint8_t nl = sbuf[off++];
		if ( off + nl > (uint16_t)n ) break;
		uint8_t cl = nl < (SSID_LENGTH - 1) ? nl : (SSID_LENGTH - 1);
		memcpy(list[filled].name, &sbuf[off], cl);
		list[filled].name[cl] = 0;
		off += nl;
		snprintf((char *)list[filled].addr, BSSID_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
		         a[0], a[1], a[2], a[3], a[4], a[5]);
		list[filled].rssi = (int)rs;
		list[filled].addr_type = at;
		filled++;
	}
	app_req->u.ble_scan.count = filled;
	app_req->u.ble_scan.out_list = list;
	app_req->msg_type = CTRL_RESP;
	app_req->resp_event_status = SUCCESS;
	return SUCCESS;
}
#endif /* M1_USE_ESP_RPC */

uint8_t ble_scan_list(ctrl_cmd_t *app_req)
{
	return m1esp_ble_scan_fill(app_req);
} // uint8_t ble_scan_list(ctrl_cmd_t *app_req)



#ifdef M1_APP_BT_MANAGE_ENABLE

uint8_t ble_scan_list_ex(ctrl_cmd_t *app_req)
{
	return m1esp_ble_scan_fill(app_req);
} // uint8_t ble_scan_list_ex(ctrl_cmd_t *app_req)



uint8_t esp_get_version(ctrl_cmd_t *app_req)
{
	/* Option C: native ESP firmware version via m1_link (SYS_GET_FW_VERSION).
	 * The legacy AT path below (AT+GMR over the uninitialized AT transport)
	 * would crash — this is a menu-reachable landmine (BT -> Info). */
	memset(app_req->u.wifi_ap_config.status, 0, STATUS_LENGTH);
	if ( m1_esp_client_fw_version((char *)app_req->u.wifi_ap_config.status, STATUS_LENGTH) )
	{
		app_req->resp_event_status = SUCCESS;
		return SUCCESS;
	}
	app_req->resp_event_status = ERROR;
	return ERROR;
} // uint8_t esp_get_version(ctrl_cmd_t *app_req)



uint8_t ble_connect(ctrl_cmd_t *app_req, const char *addr, uint8_t addr_type)
{
	{
		uint8_t ok = m1_esp_client_ble_connect(addr, addr_type) ? SUCCESS : ERROR;
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = ok;
		return ok;
	}
} // uint8_t ble_connect(ctrl_cmd_t *app_req, const char *addr, uint8_t addr_type)



uint8_t ble_disconnect(ctrl_cmd_t *app_req)
{
	/* Native BLE stack owns its own lifecycle; the legacy AT disconnect path
	 * has been removed. No current callers. */
	if ( app_req )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	}
	return SUCCESS;
} // uint8_t ble_disconnect(ctrl_cmd_t *app_req)

#endif /* M1_APP_BT_MANAGE_ENABLE */


uint8_t ble_advertise(ctrl_cmd_t *app_req)
{
	{
		uint8_t ok = m1_esp_client_ble_advertise("M1-BLE") ? SUCCESS : ERROR;
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = ok;
		return ok;
	}
} // uint8_t ble_advertise(ctrl_cmd_t *app_req)




#ifdef M1_APP_BADBT_ENABLE

uint8_t ble_hid_init(ctrl_cmd_t *app_req, const char *device_name)
{
	return m1_esp_client_ble_hid_init(device_name) ? SUCCESS : ERROR;
}


uint8_t ble_hid_deinit(ctrl_cmd_t *app_req)
{
	m1_esp_client_ble_hid_deinit();
	return SUCCESS;
}


uint8_t ble_hid_send_kb(ctrl_cmd_t *app_req, uint8_t modifier, uint8_t key1)
{
	return m1_esp_client_ble_hid_key(modifier, key1) ? SUCCESS : ERROR;
}


// Wait for BLE HID connection + security handshake to complete.
// Handles: +BLECONN: → +BLESECREQ: → AT+BLEENC → +BLEAUTHCMPL:
// Also handles +BLESECNTFYNUM: (numeric comparison) → AT+BLECONFREPLY
// Returns SUCCESS when connection + pairing are both done.
uint8_t ble_hid_wait_connect(ctrl_cmd_t *app_req, uint8_t timeout_sec)
{
	{
		uint32_t t0 = HAL_GetTick();
		while ( (HAL_GetTick() - t0) < (uint32_t)timeout_sec * 1000u )
		{
			uint8_t connected = 0;
			if ( m1_esp_client_ble_hid_status(&connected) && connected )
				return SUCCESS;
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		return ERROR;
	}
}

#endif /* M1_APP_BADBT_ENABLE */


uint8_t esp_dev_reset(ctrl_cmd_t *app_req)
{
	/* Option C: the native BLE stack owns its own lifecycle (ble_adv_start/stop,
	 * ble_hid_deinit) and needs no AT-era "AT+RST + wait ready" cleanup. Routing
	 * this to SYS_RESET would reboot the ESP and tear down m1_link — wrong. The
	 * legacy AT path below would also crash (uninitialized AT transport). This
	 * is a menu-reachable landmine (BT -> Advertise, on entry and exit). No-op. */
	if ( app_req ) app_req->resp_event_status = SUCCESS;
	return SUCCESS;
} // uint8_t esp_dev_reset(ctrl_cmd_t *app_req)


#ifdef M1_APP_WIFI_CONNECT_ENABLE

uint8_t wifi_connect_ap(ctrl_cmd_t *app_req)
{
	{
		uint8_t ip[4] = { 0 };
		bool ok = m1_esp_client_wifi_connect((const char *)app_req->u.wifi_ap_config.ssid,
		                                     (const char *)app_req->u.wifi_ap_config.pwd, ip);
		memset(app_req->u.wifi_ap_config.status, 0, STATUS_LENGTH);
		if ( ok )
			snprintf(app_req->u.wifi_ap_config.status, STATUS_LENGTH, "%u.%u.%u.%u",
			         ip[0], ip[1], ip[2], ip[3]);
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = ok ? SUCCESS : ERROR;
		return ok ? SUCCESS : ERROR;
	}
} // uint8_t wifi_connect_ap(ctrl_cmd_t *app_req)



uint8_t wifi_disconnect_ap(ctrl_cmd_t *app_req)
{
	{
		bool ok = m1_esp_client_wifi_disconnect();
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = ok ? SUCCESS : ERROR;
		return ok ? SUCCESS : ERROR;
	}
} // uint8_t wifi_disconnect_ap(ctrl_cmd_t *app_req)



uint8_t wifi_get_ip(ctrl_cmd_t *app_req)
{
	{
		uint8_t connected = 0, ip[4] = { 0 };
		bool ok = m1_esp_client_wifi_status(&connected, ip);
		memset(app_req->u.wifi_ap_config.status, 0, STATUS_LENGTH);
		memset(app_req->u.wifi_ap_config.out_mac, 0, MAX_MAC_STR_SIZE);
		if ( ok && connected )
			snprintf(app_req->u.wifi_ap_config.status, STATUS_LENGTH, "%u.%u.%u.%u",
			         ip[0], ip[1], ip[2], ip[3]);
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
		return SUCCESS;
	}
} // uint8_t wifi_get_ip(ctrl_cmd_t *app_req)

#endif /* M1_APP_WIFI_CONNECT_ENABLE */
