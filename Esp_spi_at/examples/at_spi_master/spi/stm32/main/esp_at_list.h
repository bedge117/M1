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

// Firmware version query
#define ESP32C6_AT_REQ_GET_VERSION		"AT+GMR"
#define ESP32C6_AT_RES_VERSION_KEY		"AT version:"

// BLE GATT Server
#define ESP32C6_AT_REQ_BLE_GATTS_CRE	"AT+BLEGATTSSRVCRE"
#define ESP32C6_AT_REQ_BLE_GATTS_START	"AT+BLEGATTSSRVSTART"

// BLE Security: AT+BLESECPARAM=<auth_req>,<iocap>,<enc_key_size>,<init_key_dist>,<rsp_key_dist>
// 1=bonding, 3=NoInputNoOutput, 16=max key size, 3=init_key, 3=rsp_key ("Just Works" pairing)
#define ESP32C6_AT_REQ_BLE_SEC_PARAM	"AT+BLESECPARAM=1,3,16,3,3"

// BLE Advertising control
// Force legacy advertising parameters: min=32(20ms), max=64(40ms), type=0(ADV_IND legacy),
// addr_type=0(public), channel_map=7(all), filter=0(none)
// Required because CONFIG_BT_NIMBLE_EXT_ADV=y in ESP32 build causes AT firmware
// to default to extended advertising which some hosts can't see
#define ESP32C6_AT_REQ_BLE_ADV_PARAM	"AT+BLEADVPARAM=32,64,0,0,7,0"
#define ESP32C6_AT_REQ_BLE_ADV_DATA		"AT+BLEADVDATA="
#define ESP32C6_AT_REQ_BLE_ADV_START	"AT+BLEADVSTART"
#define ESP32C6_AT_REQ_BLE_ADV_STOP		"AT+BLEADVSTOP"

// BLE Encryption: AT+BLEENC=<conn_index>,<sec_act>
// sec_act: 0=no change, 1=encrypt no MITM, 2=encrypt MITM, 3=CSRK
#define ESP32C6_AT_REQ_BLE_ENC			"AT+BLEENC="

// BLE Security events (unsolicited from ESP32)
#define ESP32C6_AT_RES_BLE_SEC_REQ		"+BLESECREQ:"
#define ESP32C6_AT_RES_BLE_AUTH_CMPL	"+BLEAUTHCMPL:"
#define ESP32C6_AT_RES_BLE_SEC_NTFY		"+BLESECNTFYNUM:"

// BLE Confirm reply for numeric comparison: AT+BLECONFREPLY=<conn_index>,<confirm>
#define ESP32C6_AT_REQ_BLE_CONF_REPLY	"AT+BLECONFREPLY="

// BLE device name
#define ESP32C6_AT_REQ_BLE_NAME		"AT+BLENAME="

// BLE HID (keyboard emulation)
// AT+HIDKBINIT=<enable>  1=configure HID appearance/security
#define ESP32C6_AT_REQ_BLE_HID_INIT	"AT+HIDKBINIT="
// AT+HIDKBSEND=<modifier>,<key1>,<key2>,<key3>,<key4>,<key5>,<key6>
#define ESP32C6_AT_REQ_BLE_HID_KB		"AT+HIDKBSEND="

// BLE Connect/Disconnect
// AT+BLECONN=<conn_index>,<remote_BLE_address>[,<addr_type>][,<timeout>]
#define ESP32C6_AT_REQ_BLE_CONNECT		"AT+BLECONN="
#define ESP32C6_AT_RES_BLE_CONNECT_KEY	"+BLECONN:"

// AT+BLEDISCONN=<conn_index>
#define ESP32C6_AT_REQ_BLE_DISCONNECT	"AT+BLEDISCONN="
#define ESP32C6_AT_RES_BLE_DISCONNECTED	"+BLEDISCONN:"

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
