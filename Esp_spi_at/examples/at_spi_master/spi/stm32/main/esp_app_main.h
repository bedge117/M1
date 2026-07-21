/* See COPYING.txt for license details. */

/*
*
* esp_app_main.h
*
* Header for esp app
*
* M1 Project
*
*/

#ifndef ESP_APP_MAIN_H_
#define ESP_APP_MAIN_H_

#include <stdbool.h>
#include "ctrl_api.h"

bool get_esp32_main_init_status(void);
void esp32_main_init(void);
uint8_t spi_AT_send_recv(const char *at_cmd, char *out_buf, int out_buf_size, int timeout_sec);
uint8_t spi_AT_app_send_command(ctrl_cmd_t *app_req);
uint8_t *spi_AT_app_get_response(int *read_len, uint32_t *uid, int timeout_sec);
uint8_t wifi_ap_scan_list(ctrl_cmd_t *app_req);
uint8_t ble_scan_list(ctrl_cmd_t *app_req);
uint8_t ble_advertise(ctrl_cmd_t *app_req);
uint8_t esp_dev_reset(ctrl_cmd_t *app_req);

#ifdef M1_APP_WIFI_CONNECT_ENABLE
uint8_t wifi_connect_ap(ctrl_cmd_t *app_req);
uint8_t wifi_disconnect_ap(ctrl_cmd_t *app_req);
uint8_t wifi_get_ip(ctrl_cmd_t *app_req);
#endif

#ifdef M1_APP_BADBT_ENABLE
uint8_t ble_hid_init(ctrl_cmd_t *app_req, const char *device_name);
uint8_t ble_hid_deinit(ctrl_cmd_t *app_req);
uint8_t ble_hid_send_kb(ctrl_cmd_t *app_req, uint8_t modifier, uint8_t key1);
uint8_t ble_hid_wait_connect(ctrl_cmd_t *app_req, uint8_t timeout_sec);
#endif

#ifdef M1_APP_BT_MANAGE_ENABLE
uint8_t ble_scan_list_ex(ctrl_cmd_t *app_req);
uint8_t esp_get_version(ctrl_cmd_t *app_req);
uint8_t ble_connect(ctrl_cmd_t *app_req, const char *addr, uint8_t addr_type);
uint8_t ble_disconnect(ctrl_cmd_t *app_req);
#endif

/* NOTE: the old "binary protocol" layer (M1_USE_BINARY_PROTOCOL -> *_bin
 * functions in m1_esp32_binary.c, over the legacy esp-hosted AT transport) has
 * been removed. Under Option C every ESP operation goes through m1_esp_client
 * (SPI / m1_link); the ctrl-API entry points below are implemented directly on
 * that path in esp_app_main.c. */

#endif /* ESP_APP_MAIN_H_ */
