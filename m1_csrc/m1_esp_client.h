/* See COPYING.txt for license details. */
/*
 * m1_esp_client.h — M1 -> ESP32-C6 operation API over the new reliable
 * transport (m1_link) + binary protocol (m1_esp_rpc, magic 0x4D31).
 *
 * This is the m1_rpc-native replacement for the AT-transport client in
 * m1_esp32_binary.c. Each call builds a REQ frame, drives it over m1_link,
 * and returns the parsed response. Operations are serialized (one at a time).
 */
#ifndef M1_ESP_CLIENT_H_
#define M1_ESP_CLIENT_H_

#include <stdint.h>
#include <stdbool.h>
#include "m1_esp_rpc.h"

/* Initialize the transport (call once after the ESP SPI is up). */
void m1_esp_client_init(void);

/* Liveness: SYS_PING round-trip. true if the ESP echoed our cookie. */
bool m1_esp_client_ping(void);

/* Capability/identity probe: fills *out (proto_ver, cap_bitmap, fw_name).
 * true on success. Lets one host firmware adapt to any ESP build. */
bool m1_esp_client_get_status(m1esp_devstatus_t *out);

/* Native ESP firmware version formatted into `out` (cap bytes). */
bool m1_esp_client_fw_version(char *out, uint16_t cap);

/* NTP time sync via the ESP (WiFi must be up). Fills `out` with UTC time. */
bool m1_esp_client_sntp_sync(m1esp_time_t *out);

/* 802.15.4 (Zigbee/Thread) device discovery. channel 0 = hop all 11-26. */
bool m1_esp_client_zb_sniff_start(uint8_t channel);
bool m1_esp_client_zb_sniff_stop(void);
int  m1_esp_client_zb_sniff_get(uint8_t *buf, uint16_t cap);

/* WiFi scan. Copies the raw response payload ([count:2][entries...]) into buf.
 * Returns payload length (>0), 0 on timeout, <0 on error. */
int  m1_esp_client_wifi_scan(uint8_t *buf, uint16_t buf_size);

/* WiFi connect. Returns true on success; writes the 4-byte IP into ip_out. */
bool m1_esp_client_wifi_connect(const char *ssid, const char *pwd, uint8_t ip_out[4]);
bool m1_esp_client_wifi_disconnect(void);

/* True while the ESP is associated to a WiFi AP (tracked via connect/disconnect/
 * status). Used by the qMonstatek-over-WiFi relay to avoid polling with no WiFi. */
bool m1_esp_client_wifi_is_connected(void);

/* WiFi status: sets *connected (0/1) and the current IP (0.0.0.0 if none).
 * Returns true if the ESP answered. */
bool m1_esp_client_wifi_status(uint8_t *connected, uint8_t ip_out[4]);

/* Offensive: start/stop a deauth run on the ESP (runs autonomously there). */
bool m1_esp_client_deauth_start(const uint8_t bssid[6], uint8_t channel,
                                const uint8_t station[6], uint16_t count,
                                uint16_t interval_ms);
bool m1_esp_client_deauth_stop(void);

/* Scan for stations (clients) on a target AP for duration_sec. Copies the raw
 * payload ([count:2][per sta: mac:6, rssi:i8]) into buf. Returns length, 0/neg on error. */
int  m1_esp_client_sta_scan(const uint8_t bssid[6], uint8_t channel, uint8_t duration_sec,
                            uint8_t *buf, uint16_t buf_size);

/* Handshake (EAPOL) capture. hs_start optionally deauths (deauth_count>0) to
 * force reconnect. hs_status returns state (0 idle/1 capturing/2 complete) and
 * total captured PCAP length. hs_read fetches a chunk at offset (<=502). */
bool m1_esp_client_hs_start(const uint8_t bssid[6], uint8_t channel, uint16_t deauth_count);
bool m1_esp_client_hs_status(uint8_t *state, uint32_t *total_len);
int  m1_esp_client_hs_read(uint32_t offset, uint8_t *buf, uint16_t want);
bool m1_esp_client_hs_stop(void);

/* Beacon spam: broadcast fake APs for the given SSID list. */
bool m1_esp_client_beacon_start(const char ssids[][33], uint8_t count);
bool m1_esp_client_beacon_stop(void);

/* Packet monitor: promiscuous 802.11 sniffer on the ESP. channel 0 = hop 1-13.
 * start returns the ESP esp_err (0 = OK); negative = SPI/link failure.
 * The ESP buffers captured frames; the M1 pulls stats and frames on demand. */
int      m1_esp_client_monitor_start(uint8_t channel);
bool     m1_esp_client_monitor_stop(void);
bool     m1_esp_client_monitor_stats(uint32_t *total, uint32_t *dropped, uint8_t *buffered);
uint16_t m1_esp_client_monitor_read(uint8_t *out, uint16_t out_cap,
                                    int8_t *rssi, uint8_t *channel);

/* Captive portal: rogue open AP (SSID) + DNS hijack + credential-capture page,
 * all autonomous on the ESP. creds returns raw bytes:
 * [count:1] then per cred [ts:4][ulen:1][user][plen:1][pass]. */
int  m1_esp_client_captive_start(const char *ssid, const char *title, uint8_t channel);
bool m1_esp_client_captive_stop(void);
int  m1_esp_client_captive_creds(uint8_t *buf, uint16_t buf_size);
bool m1_esp_client_captive_diag(uint32_t *dns_q, uint32_t *http_hits, uint8_t *clients,
                                char *last_post, uint16_t last_post_cap);

/* BLE scan: scan for duration_sec, copy the raw result payload
 * ([count:2][per dev: addr[6], addr_type, rssi, name_len, name]) into buf.
 * Returns payload length (>0), 0 on timeout, <0 on error. */
int m1_esp_client_ble_scan(uint8_t *buf, uint16_t buf_size, uint8_t duration_sec);

/* BLE HID (BadBT): start advertising as `name`, tear down, query connection,
 * and send one key report (modifier + HID usage; send a 0,0 report to release). */
bool m1_esp_client_ble_hid_init(const char *name);
bool m1_esp_client_ble_hid_deinit(void);
bool m1_esp_client_ble_hid_status(uint8_t *connected);
bool m1_esp_client_ble_hid_key(uint8_t modifier, uint8_t keycode);

/* BLE connect to a device by MAC string ("AA:BB:CC:DD:EE:FF") + addr_type,
 * and generic (non-HID) advertising as `name`. */
bool m1_esp_client_ble_connect(const char *addr_str, uint8_t addr_type);
bool m1_esp_client_ble_advertise(const char *name);

/* Push a framebuffer to the ESP (for qMonstatek forwarding). */
bool m1_esp_client_screen_push(const uint8_t *fb, uint16_t len);

/* ---- ESP-NOW peer-to-peer M1<->M1 link ---- */
typedef struct {
    uint8_t mac[6];
    int8_t  rssi;
    char    name[24];
} m1_now_peer_t;

/* Start the ESP-NOW peer link on `channel` (1-13), announcing as `name`.
 * Returns this unit's MAC in mac_out (6 bytes). */
bool m1_esp_client_now_start(uint8_t channel, const char *name, uint8_t mac_out[6]);
/* Diagnostic: raw result of the last now_start (esp_call length / first byte /
 * the esp_err_t the ESP reported on failure). */
extern volatile int      m1_esp_client_now_last_n;
extern volatile int      m1_esp_client_now_last_b0;
extern volatile uint32_t m1_esp_client_now_last_err;
bool m1_esp_client_now_stop(void);
bool m1_esp_client_now_announce(void);                 /* broadcast presence */
/* Fill out[] with up to `max` discovered peers; returns count (<0 on error). */
int  m1_esp_client_now_get_peers(m1_now_peer_t *out, int max);
/* Unicast to mac (6 bytes; all-FF = broadcast). len <= 240. */
bool m1_esp_client_now_send(const uint8_t mac[6], const uint8_t *data, uint16_t len);
/* Drain received messages into buf as [count:1] then [mac:6][len:2 LE][data]
 * per message. Returns bytes written (<0 on error). */
int  m1_esp_client_now_recv(uint8_t *buf, uint16_t cap);

/* qMonstatek-over-WiFi relay: is a desktop TCP client connected to the ESP?
 * Poll the next queued desktop command frame into buf (returns len, 0 if none).
 * Forward an M1-produced qMonstatek response frame to that client. */
bool m1_esp_client_rpc_connected(uint8_t *connected);
int  m1_esp_client_qmon_poll(uint8_t *buf, uint16_t buf_size);
bool m1_esp_client_qmon_resp(const uint8_t *frame, uint16_t len);

#endif /* M1_ESP_CLIENT_H_ */
