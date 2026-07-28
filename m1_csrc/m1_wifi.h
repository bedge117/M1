/* See COPYING.txt for license details. */

/*
*
* m1_wifi.h
*
* Library for M1 Wifi
*
* M1 Project
*
*/


#ifndef M1_WIFI_H_
#define M1_WIFI_H_

#include "m1_compile_cfg.h"
#include <stdint.h>

/* WiFi auto-connect on boot + the chosen "primary" network SSID. Persisted in
 * settings.cfg. Declared unconditionally so the settings save/load and the
 * System Settings menu always link regardless of the WiFi-connect feature gate. */
extern uint8_t m1_wifi_boot_connect;        /* 0 = off, 1 = connect at boot */
extern char    m1_wifi_primary_ssid[];      /* empty = no primary set */
extern uint8_t m1_ble_direct;               /* 0 = off, 1 = ESP advertises NUS RPC over BLE */
extern uint8_t m1_hotspot_on;               /* 0 = off, 1 = SoftAP hotspot running */
extern char    m1_hotspot_ssid[];           /* hotspot SSID (editable) */
extern char    m1_hotspot_pass[];           /* hotspot WPA2 password (>=8 chars, else open) */

/* WiFi Hotspot (SoftAP) menu — enable/disable, edit SSID/password, show status. */
void wifi_hotspot_menu(void);

/* Connect to the primary network at boot when m1_wifi_boot_connect is set.
 * No-op if the toggle is off, no primary is set, or its credential is missing. */
void wifi_boot_autoconnect(void);

void menu_wifi_init(void);
void menu_wifi_exit(void);

void wifi_scan_ap(void);
void wifi_deauth_menu(void);
void wifi_handshake_menu(void);
void wifi_beacon_menu(void);
void ble_spam_menu(void);
void wifi_probe_flood_menu(void);
void wifi_karma_menu(void);
void wifi_monitor_menu(void);
void wifi_captive_menu(void);
void wifi_config(void);

#ifdef M1_APP_WIFI_CONNECT_ENABLE
void wifi_saved_networks(void);
void wifi_show_status(void);
void wifi_disconnect(void);
uint8_t wifi_sync_rtc(void);
#endif

#endif /* M1_WIFI_H_ */
