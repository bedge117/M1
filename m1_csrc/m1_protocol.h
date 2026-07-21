/*
 * M1 SPI Protocol — Binary command protocol between M1 and ESP32-C6
 *
 * Architecture:
 *   M1 (STM32H573) = SPI master, device controller (screen, buttons, menus)
 *   ESP32-C6 = SPI slave, network coprocessor (WiFi, BLE, Zigbee, attacks)
 *
 * M1 sends commands, ESP32 executes and returns results.
 * Replaces AT text commands with fast binary protocol.
 * ESP32 runs operations autonomously (e.g. entire Bad-BT scripts,
 * deauth loops, captive portals) — M1 just orchestrates.
 *
 * SPI: half-duplex via spi_slave_hd, handshake GPIO for flow control.
 * Same pin wiring and handshake protocol as the AT firmware so M1's
 * existing SPI master driver works unchanged.
 *
 * Frame format (over SPI stream):
 * [SYNC:2][CMD:1][FLAGS:1][SEQ:2][LEN:2][PAYLOAD:0..4088][CRC16:2]
 *
 * Max frame: 4096 bytes. Max payload: 4088 bytes.
 */

#ifndef M1_PROTOCOL_H
#define M1_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Frame constants ---------- */

#define M1P_SYNC_WORD           0xAA55
#define M1P_HEADER_SIZE         8       /* SYNC(2) + CMD(1) + FLAGS(1) + SEQ(2) + LEN(2) */
#define M1P_CRC_SIZE            2
#define M1P_MAX_FRAME_SIZE      4096
#define M1P_MAX_PAYLOAD_SIZE    (M1P_MAX_FRAME_SIZE - M1P_HEADER_SIZE - M1P_CRC_SIZE)

/* ---------- Frame header ---------- */

typedef struct __attribute__((packed)) {
    uint16_t sync;      /* M1P_SYNC_WORD (0xAA55) */
    uint8_t  cmd;       /* Command ID (m1p_cmd_t) */
    uint8_t  flags;     /* m1p_flags_t */
    uint16_t seq;       /* Sequence number — ESP32 sets, M1 echoes */
    uint16_t len;       /* Payload length (0..M1P_MAX_PAYLOAD_SIZE) */
} m1p_header_t;

_Static_assert(sizeof(m1p_header_t) == M1P_HEADER_SIZE, "header size mismatch");

/* ---------- Flags ---------- */

typedef enum {
    M1P_FLAG_RESPONSE   = (1 << 0),  /* 0=request (M1→ESP32), 1=response (ESP32→M1) */
    M1P_FLAG_MORE       = (1 << 1),  /* More frames follow (multi-frame response) */
    M1P_FLAG_ERROR      = (1 << 2),  /* Error response — payload is error code */
    M1P_FLAG_NOTIFY     = (1 << 3),  /* Unsolicited notification from ESP32 */
} m1p_flags_t;

/* ---------- Command IDs ---------- */

typedef enum {
    /* System (0x01–0x0F) — M1 sends, ESP32 responds */
    M1P_CMD_PING            = 0x01,
    M1P_CMD_PONG            = 0x02,
    M1P_CMD_RESET           = 0x03,   /* M1 requests ESP32 soft reset */
    M1P_CMD_ESP_INFO        = 0x04,   /* M1 queries ESP32 firmware version */
    M1P_CMD_ESP_INFO_RESP   = 0x05,

    /* WiFi (0x10–0x1F) — M1 sends command, ESP32 executes */
    M1P_CMD_WIFI_SCAN       = 0x10,   /* Start scan, returns AP list */
    M1P_CMD_WIFI_SCAN_RESP  = 0x11,   /* Binary array of AP records */
    M1P_CMD_WIFI_CONNECT    = 0x12,   /* Payload: [ssid_len:1][ssid:N][pass_len:1][pass:N] */
    M1P_CMD_WIFI_CONNECT_RESP = 0x13, /* Payload: [status:1][ip:4] */
    M1P_CMD_WIFI_DISCONNECT = 0x14,
    M1P_CMD_WIFI_STATUS     = 0x15,   /* Query connection status */
    M1P_CMD_WIFI_STATUS_RESP = 0x16,

    /* WiFi Attacks (0x18–0x1F) — ESP32 runs autonomously */
    M1P_CMD_DEAUTH_START    = 0x18,   /* Payload: [target:6][ap:6][channel:1][count:2] */
    M1P_CMD_DEAUTH_STOP     = 0x19,
    M1P_CMD_DEAUTH_STATUS   = 0x1A,   /* ESP32 reports frames sent */
    M1P_CMD_CAPTIVE_START   = 0x1B,   /* Payload: [ssid_len:1][ssid:N] */
    M1P_CMD_CAPTIVE_STOP    = 0x1C,
    M1P_CMD_CAPTIVE_CREDS   = 0x1D,   /* ESP32 returns captured credentials */
    M1P_CMD_BEACON_START    = 0x1E,   /* Payload: [count:1][ssid_len:1][ssid:N]... */
    M1P_CMD_BEACON_STOP     = 0x1F,

    /* BLE (0x20–0x2F) — ESP32 handles BLE stack */
    M1P_CMD_BLE_SCAN        = 0x20,   /* Start BLE scan */
    M1P_CMD_BLE_SCAN_RESP   = 0x21,   /* Binary array of BLE devices */
    M1P_CMD_BLE_HID_START   = 0x22,   /* Start HID keyboard, payload: [device_name:N] */
    M1P_CMD_BLE_HID_STOP    = 0x23,
    M1P_CMD_BLE_HID_KEY     = 0x24,   /* Single key: [modifier:1][keycode:1] */
    M1P_CMD_BLE_HID_STRING  = 0x25,   /* Type string: [str:N] — ESP32 fires all keys autonomously */
    M1P_CMD_BLE_HID_SCRIPT  = 0x26,   /* Bad-BT script: bulk payload, ESP32 executes entire thing */
    M1P_CMD_BLE_HID_STATUS  = 0x27,   /* Query: connected? progress? */
    M1P_CMD_BLE_HID_STATUS_RESP = 0x28,

    /* Zigbee/Thread (0x30–0x3F) */
    M1P_CMD_ZIGBEE_SCAN     = 0x30,
    M1P_CMD_ZIGBEE_SCAN_RESP = 0x31,
    M1P_CMD_THREAD_SCAN     = 0x32,
    M1P_CMD_THREAD_SCAN_RESP = 0x33,

    /* WiFi Monitor / Packet Capture (0x38–0x3F) */
    M1P_CMD_MONITOR_START   = 0x38,   /* Payload: [channel:1] */
    M1P_CMD_MONITOR_STOP    = 0x39,
    M1P_CMD_MONITOR_HOP     = 0x3A,   /* Channel hop: [dwell_ms:2] */
    M1P_CMD_MONITOR_PACKET  = 0x3B,   /* ESP32→M1: captured packet data */

    /* Screen streaming to qMonstatek (0x40–0x4F) */
    M1P_CMD_SCREEN_PUSH     = 0x40,   /* M1 pushes framebuffer to ESP32 for TCP forwarding */
    M1P_CMD_SCREEN_PUSH_ACK = 0x41,
    M1P_CMD_RPC_STATUS      = 0x42,   /* M1 asks: is qMonstatek connected? */
    M1P_CMD_RPC_STATUS_RESP = 0x43,

    /* Generic ACK/NACK */
    M1P_CMD_ACK             = 0xF0,
    M1P_CMD_NACK            = 0xF1,
} m1p_cmd_t;

/* ---------- Error codes (in error response payload) ---------- */

typedef enum {
    M1P_ERR_NONE           = 0x00,
    M1P_ERR_UNKNOWN_CMD    = 0x01,
    M1P_ERR_BAD_CRC        = 0x02,
    M1P_ERR_BAD_LENGTH     = 0x03,
    M1P_ERR_BUSY           = 0x04,
    M1P_ERR_NOT_READY      = 0x05,
    M1P_ERR_FILE_NOT_FOUND = 0x06,
    M1P_ERR_SD_ERROR       = 0x07,
    M1P_ERR_FW_ERROR       = 0x08,
} m1p_error_t;

/* ---------- Payload structs ---------- */

/* Device info response (CMD_GET_DEVICE_INFO response) */
typedef struct __attribute__((packed)) {
    uint8_t  hw_revision;
    uint8_t  battery_pct;       /* 0–100 */
    uint16_t battery_mv;        /* millivolts */
    uint8_t  sd_present;        /* 0=no, 1=yes */
    uint32_t sd_total_kb;
    uint32_t sd_free_kb;
    uint8_t  charging;          /* 0=no, 1=yes */
    uint8_t  screen_width;      /* pixels */
    uint8_t  screen_height;     /* pixels */
    uint8_t  screen_bpp;        /* bits per pixel (1=mono) */
    char     device_name[32];   /* null-terminated */
} m1p_device_info_t;

/* Firmware info response (CMD_GET_FW_INFO response) */
typedef struct __attribute__((packed)) {
    uint8_t  major;
    uint8_t  minor;
    uint8_t  patch;
    uint32_t build_number;
    char     git_hash[12];      /* short hash, null-terminated */
    char     build_date[20];    /* "YYYY-MM-DD HH:MM:SS" */
} m1p_fw_info_t;

/* Button press (CMD_BUTTON_PRESS payload) */
typedef struct __attribute__((packed)) {
    uint8_t button_id;          /* See m1p_button_t */
    uint8_t action;             /* 0=press, 1=release, 2=long_press */
} m1p_button_press_t;

typedef enum {
    M1P_BTN_UP     = 0,
    M1P_BTN_DOWN   = 1,
    M1P_BTN_LEFT   = 2,
    M1P_BTN_RIGHT  = 3,
    M1P_BTN_SELECT = 4,
    M1P_BTN_BACK   = 5,
} m1p_button_t;

/* Screen stream control (CMD_SCREEN_STREAM payload) */
typedef struct __attribute__((packed)) {
    uint8_t  enable;            /* 0=stop, 1=start */
    uint16_t interval_ms;       /* min interval between frames (0=fastest) */
} m1p_screen_stream_t;

/* File list request (CMD_FILE_LIST payload) */
typedef struct __attribute__((packed)) {
    char path[256];             /* null-terminated directory path */
} m1p_file_list_req_t;

/* File list entry (one per entry in CMD_FILE_LIST_DATA) */
typedef struct __attribute__((packed)) {
    uint8_t  is_dir;
    uint32_t size;
    char     name[128];         /* null-terminated */
} m1p_file_entry_t;

/* File read request (CMD_FILE_READ payload) */
typedef struct __attribute__((packed)) {
    uint32_t offset;
    uint16_t max_len;           /* max bytes to return */
    char     path[256];         /* null-terminated */
} m1p_file_read_req_t;

/* FW update begin (CMD_FW_UPDATE_BEGIN payload) */
typedef struct __attribute__((packed)) {
    uint32_t total_size;
    uint32_t crc32;             /* CRC of entire firmware image */
} m1p_fw_update_begin_t;

/* FW update data (CMD_FW_UPDATE_DATA payload) */
typedef struct __attribute__((packed)) {
    uint32_t offset;
    /* followed by raw firmware bytes */
} m1p_fw_update_data_t;

/* ---------- GPIO signals ---------- */

/*
 * DATA_READY: M1 → ESP32 (active high)
 *   M1 asserts when it has unsolicited data (logs, NFC events, etc.)
 *   ESP32 sees rising edge → sends CMD_GET_LOG or appropriate poll
 *
 * Current SPI pin assignments (ESP32 GPIO numbers):
 *   SCLK = GPIO7   (ESP32 drives as master)
 *   MOSI = GPIO12  (ESP32 out → M1 in)
 *   MISO = GPIO13  (M1 out → ESP32 in)
 *   CS   = GPIO15  (ESP32 drives)
 *   DATA_READY = GPIO14 (M1 drives, ESP32 reads — was handshake pin)
 */

#define M1P_SPI_PIN_SCLK       7
#define M1P_SPI_PIN_MOSI       12
#define M1P_SPI_PIN_MISO       13
#define M1P_SPI_PIN_CS         15
#define M1P_SPI_PIN_DATA_READY 14

#define M1P_SPI_CLOCK_HZ       (4687500)  /* ~4.7 MHz — matches M1 SPI3 (75MHz/16) */

/* ---------- Protocol helpers ---------- */

/* CRC-16/CCITT over buffer */
uint16_t m1p_crc16(const uint8_t *data, size_t len);

/* Build a frame into buf. Returns total frame size, or 0 on error. */
size_t m1p_build_frame(uint8_t *buf, size_t buf_size,
                       uint8_t cmd, uint8_t flags, uint16_t seq,
                       const uint8_t *payload, uint16_t payload_len);

/* Parse a received frame. Returns 0 on success, -1 on error (bad sync/crc/length). */
int m1p_parse_frame(const uint8_t *buf, size_t buf_len,
                    m1p_header_t *hdr_out,
                    const uint8_t **payload_out,
                    uint16_t *payload_len_out);

#ifdef __cplusplus
}
#endif

#endif /* M1_PROTOCOL_H */
