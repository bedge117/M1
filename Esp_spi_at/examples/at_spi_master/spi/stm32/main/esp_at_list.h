/* See COPYING.txt for license details. */

/*
*
* esp_at_list.h
*
* AT commands list
*
* M1 Project
*
*/

#ifndef ESP32C6_AT_LIST_H_
#define ESP32C6_AT_LIST_H_

#define CONCAT_CMD_PARAM(cmd, param)		cmd param ESP32C6_AT_REQ_CRLF

#define ESP32C6_WIFI_MODE_NULL		"0" // Null mode. Wi-Fi RF will be disabled.
#define ESP32C6_WIFI_MODE_STA		"1" // Station mode.
#define ESP32C6_WIFI_MODE_AP		"2" // SoftAP mode.
#define ESP32C6_WIFI_MODE_APSTA		"3" // SoftAP+Station mode.

#define ESP32C6_BLE_MODE_NULL		"0" // deinit Bluetooth LE
#define ESP32C6_BLE_MODE_CLI		"1" // client role
#define ESP32C6_BLE_MODE_SER		"2" // server role

// Syntax: AT+CWMODE=<mode>[,<auto_connect>]
// Response; OK
#define ESP32C6_AT_REQ_WIFI_MODE		"AT+CWMODE="

#define ESP32C6_AT_REQ_LIST_AP			"AT+CWLAP"
#define ESP32C6_AT_RES_LIST_AP_KEY		"+CWLAP:"

#define ESP32C6_AT_REQ_CRLF				"\r\n"

#define ESP32C6_AT_RES_OK				"OK"
#define ESP32C6_AT_RES_READY			"ready"

#define ESP32C6_AT_REQ_BLE_MODE			"AT+BLEINIT="
#define ESP32C6_AT_REQ_BLE_SCAN			"AT+BLESCAN=1," // AT+BLESCAN=<enable>[,<duration>][,<filter_type>,<filter_param>]
#define ESP32C6_AT_RES_BLE_SCAN_KEY		"+BLESCAN:"

#define ESP32C6_AT_REQ_ADVERTISE		"AT+BLEADVDATAEX=" // AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power>
#define ESP32C6_AT_REQ_ADV_DATA			"\"MONSTATEK-M1\",\"A000\",\"1A2B3C4D5E\",1"

#define ESP32C6_AT_RESET				"AT+RST"

// WiFi Connect/Disconnect/Status
// Syntax: AT+CWJAP="<ssid>","<pwd>"[,<bssid>]
// Success response: WIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK
// Error response: +CWJAP:<error_code>\r\n\r\nFAIL
//   error_code: 1=timeout, 2=wrong password, 3=AP not found, 4=connect fail
#define ESP32C6_AT_REQ_CONNECT_AP		"AT+CWJAP="
#define ESP32C6_AT_RES_CONNECT_AP_KEY	"+CWJAP:"
#define ESP32C6_AT_RES_WIFI_CONNECTED	"WIFI CONNECTED"
#define ESP32C6_AT_RES_WIFI_GOT_IP		"WIFI GOT IP"
#define ESP32C6_AT_RES_FAIL				"FAIL"

// Syntax: AT+CWQAP
// Response: OK
#define ESP32C6_AT_REQ_DISCONNECT_AP	"AT+CWQAP"

// Syntax: AT+CIFSR
// Response: +CIFSR:STAIP,"<ip>"\r\n+CIFSR:STAMAC,"<mac>"
#define ESP32C6_AT_REQ_GET_IP			"AT+CIFSR"
#define ESP32C6_AT_RES_GET_IP_KEY		"+CIFSR:"
#define ESP32C6_AT_RES_STAIP_KEY		"+CIFSR:STAIP,"
#define ESP32C6_AT_RES_STAMAC_KEY		"+CIFSR:STAMAC,"

#endif /* ESP32C6_AT_LIST_H_ */
