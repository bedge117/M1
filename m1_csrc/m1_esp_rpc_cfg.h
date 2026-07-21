/* See COPYING.txt for license details. */
/*
 * m1_esp_rpc_cfg.h — master toggle for routing ESP32 operations through the
 * new reliable m1_link transport + m1_esp_rpc binary protocol instead of the
 * legacy ESP-AT text path.
 *
 * When M1_USE_ESP_RPC is defined, the ctrl-API functions (wifi_ap_scan_list,
 * wifi_connect_ap, ...) use m1_esp_client under the hood; the ctrl_cmd_t
 * interface the UI reads is unchanged. Undefine to fall back to the AT path
 * (both implementations are kept side-by-side for reversibility).
 */
#ifndef M1_ESP_RPC_CFG_H_
#define M1_ESP_RPC_CFG_H_

#define M1_USE_ESP_RPC   1

#endif /* M1_ESP_RPC_CFG_H_ */
