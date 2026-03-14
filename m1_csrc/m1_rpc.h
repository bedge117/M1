/* See COPYING.txt for license details. */

/*
 * m1_rpc.h
 *
 * M1 Remote Procedure Call (RPC) Protocol Handler
 *
 * Binary protocol for communication with the qMonstatek desktop app.
 * Runs over USB CDC, coexisting with the existing CLI by detecting
 * the sync byte (0xAA) as the first byte of each packet.
 *
 * Frame format:
 *   [0xAA] [CMD:1] [SEQ:1] [LEN:2 LE] [PAYLOAD:0-1200] [CRC16:2]
 *
 * M1 Project
 */

#ifndef M1_RPC_H_
#define M1_RPC_H_

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>

/*************************** D E F I N E S ************************************/

/* Frame constants */
#define RPC_SYNC_BYTE           0xAA
#define RPC_HEADER_SIZE         5       /* SYNC + CMD + SEQ + LEN(2) */
#define RPC_CRC_SIZE            2
#define RPC_MAX_PAYLOAD         8192    /* Increased for ESP32 boot capture */
#define RPC_MAX_FRAME_SIZE      (RPC_HEADER_SIZE + RPC_MAX_PAYLOAD + RPC_CRC_SIZE)
#define RPC_SCREEN_FB_SIZE      1024    /* 128x64 / 8 */

/* ── System Commands (0x00–0x0F) ── */
#define RPC_CMD_PING            0x01
#define RPC_CMD_PONG            0x02
#define RPC_CMD_GET_DEVICE_INFO 0x03
#define RPC_CMD_DEVICE_INFO_RESP 0x04
#define RPC_CMD_REBOOT          0x05
#define RPC_CMD_ACK             0x06
#define RPC_CMD_NACK            0x07
#define RPC_CMD_POWER_OFF       0x08

/* ── Screen Commands (0x10–0x1F) ── */
#define RPC_CMD_SCREEN_START    0x10
#define RPC_CMD_SCREEN_STOP     0x11
#define RPC_CMD_SCREEN_FRAME    0x12
#define RPC_CMD_SCREEN_CAPTURE  0x13

/* ── Input Commands (0x20–0x2F) ── */
#define RPC_CMD_BUTTON_PRESS    0x20
#define RPC_CMD_BUTTON_RELEASE  0x21
#define RPC_CMD_BUTTON_CLICK    0x22

/* ── File Commands (0x30–0x3F) ── */
#define RPC_CMD_FILE_LIST       0x30
#define RPC_CMD_FILE_LIST_RESP  0x31
#define RPC_CMD_FILE_READ       0x32
#define RPC_CMD_FILE_READ_DATA  0x33
#define RPC_CMD_FILE_WRITE_START 0x34
#define RPC_CMD_FILE_WRITE_DATA 0x35
#define RPC_CMD_FILE_WRITE_FINISH 0x36
#define RPC_CMD_FILE_DELETE     0x37
#define RPC_CMD_FILE_MKDIR      0x38

/* ── Firmware Commands (0x40–0x4F) ── */
#define RPC_CMD_FW_INFO         0x40
#define RPC_CMD_FW_INFO_RESP    0x41
#define RPC_CMD_FW_UPDATE_START 0x42
#define RPC_CMD_FW_UPDATE_DATA  0x43
#define RPC_CMD_FW_UPDATE_FINISH 0x44
#define RPC_CMD_FW_BANK_SWAP    0x45
#define RPC_CMD_FW_DFU_ENTER    0x46

/* ── ESP32 Commands (0x50–0x5F) ── */
#define RPC_CMD_ESP_INFO        0x50
#define RPC_CMD_ESP_INFO_RESP   0x51
#define RPC_CMD_ESP_UPDATE_START 0x52
#define RPC_CMD_ESP_UPDATE_DATA 0x53
#define RPC_CMD_ESP_UPDATE_FINISH 0x54

/* ── Debug / CLI Commands (0x60–0x6F) ── */
#define RPC_CMD_CLI_EXEC        0x60
#define RPC_CMD_CLI_RESP        0x61
#define RPC_CMD_ESP_UART_SNOOP  0x62
#define RPC_CMD_ESP_UART_SNOOP_RESP 0x63

/* ── Button IDs (match m1_system.h) ── */
#define RPC_BUTTON_OK           0
#define RPC_BUTTON_UP           1
#define RPC_BUTTON_LEFT         2
#define RPC_BUTTON_RIGHT        3
#define RPC_BUTTON_DOWN         4
#define RPC_BUTTON_BACK         5

/* ── NACK Error Codes ── */
#define RPC_ERR_UNKNOWN_CMD     0x01
#define RPC_ERR_INVALID_PAYLOAD 0x02
#define RPC_ERR_BUSY            0x03
#define RPC_ERR_SD_NOT_READY    0x04
#define RPC_ERR_FILE_NOT_FOUND  0x05
#define RPC_ERR_FLASH_ERROR     0x07
#define RPC_ERR_CRC_MISMATCH   0x08
#define RPC_ERR_SIZE_TOO_LARGE  0x09
#define RPC_ERR_BANK_EMPTY      0x0A
#define RPC_ERR_ESP_FLASH       0x0B

/* ESP32 flash sub-error codes (sent as 2nd NACK payload byte) */
#define RPC_ESP_SUB_CONNECT     0x01   /* connect_to_target() failed */
#define RPC_ESP_SUB_ERASE       0x02   /* esp_loader_flash_start() / erase failed */
#define RPC_ESP_SUB_WRITE       0x03   /* esp_loader_flash_write() failed */
#define RPC_ESP_SUB_VERIFY      0x04   /* esp_loader_flash_verify() / MD5 failed */

/* ── Device Info Magic ── */
#define RPC_DEVICE_INFO_MAGIC   0x4D314649  /* "M1FI" */

//************************** S T R U C T U R E S *******************************

/* Parser state machine */
typedef enum {
    RPC_STATE_IDLE = 0,         /* Waiting for sync byte */
    RPC_STATE_HEADER,           /* Reading CMD + SEQ + LEN */
    RPC_STATE_PAYLOAD,          /* Reading payload */
    RPC_STATE_CRC               /* Reading CRC-16 */
} E_RPC_ParseState;

/* Parsed frame */
typedef struct {
    uint8_t  cmd;
    uint8_t  seq;
    uint16_t len;
    uint8_t  payload[RPC_MAX_PAYLOAD];
} S_RPC_Frame;

/* Screen streaming state */
typedef struct {
    bool     active;
    uint8_t  fps;               /* Requested FPS (1-15) */
    uint32_t interval_ms;       /* Computed from FPS */
    uint32_t last_send_tick;
} S_RPC_ScreenStream;

/* Device info response payload (packed, sent over wire) */
typedef struct __attribute__((packed)) {
    uint32_t magic;             /* RPC_DEVICE_INFO_MAGIC */
    uint8_t  fw_version_major;
    uint8_t  fw_version_minor;
    uint8_t  fw_version_build;
    uint8_t  fw_version_rc;
    uint16_t active_bank;
    uint8_t  battery_level;     /* 0-100 */
    uint8_t  battery_charging;  /* 0=no, 1=yes */
    uint8_t  sd_card_present;   /* 0=no, 1=yes */
    uint32_t sd_total_kb;
    uint32_t sd_free_kb;
    uint8_t  esp32_ready;
    char     esp32_version[32]; /* null-terminated */
    uint8_t  ism_band_region;
    uint8_t  op_mode;
    uint8_t  southpaw_mode;
    uint8_t  c3_revision;       /* C3 fork revision (0 = stock Monstatek) */

    /* Extended battery detail (appended for backward compat) */
    uint16_t batt_voltage_mv;   /* BQ27421 voltage in millivolts */
    int16_t  batt_current_ma;   /* BQ27421 current in mA (negative = discharging) */
    uint8_t  batt_temp_c;       /* BQ27421 temperature in degrees C */
    uint8_t  batt_health;       /* BQ27421 state-of-health 0-100% */
    uint8_t  charge_state;      /* BQ25896 stat: 0=off, 1=pre, 2=fast, 3=done */
    uint8_t  charge_fault;      /* BQ25896 fault code */
} S_RPC_DeviceInfo;

/* Firmware bank info (packed) */
typedef struct __attribute__((packed)) {
    uint8_t  fw_version_major;
    uint8_t  fw_version_minor;
    uint8_t  fw_version_build;
    uint8_t  fw_version_rc;
    uint8_t  crc_valid;
    uint32_t crc32;
    uint32_t image_size;
    uint8_t  c3_revision;        /* C3 fork revision (0 = stock or unknown) */
    char     build_date[20];     /* "MMM DD YYYY HH:MM:SS" null-terminated */
} S_RPC_BankInfo;

typedef struct __attribute__((packed)) {
    uint16_t       active_bank;
    S_RPC_BankInfo bank1;
    S_RPC_BankInfo bank2;
} S_RPC_FwInfo;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

/**
 * @brief  Initialize the RPC subsystem.
 *         Call from system init before task scheduler starts.
 */
void m1_rpc_init(void);

/**
 * @brief  Feed received bytes from USB CDC into the RPC parser.
 *         Called from the USB-to-serial task when sync byte detected.
 * @param  data   Pointer to received data
 * @param  len    Number of bytes
 */
void m1_rpc_feed(const uint8_t *data, uint16_t len);

/**
 * @brief  Check if the given data starts with an RPC sync byte.
 * @param  data   Pointer to data buffer
 * @param  len    Buffer length
 * @return true if first byte is 0xAA (RPC frame)
 */
bool m1_rpc_is_sync(const uint8_t *data, uint16_t len);

/**
 * @brief  RPC periodic task — handles screen streaming, timeouts.
 *         Register as FreeRTOS task.
 * @param  param  Not used
 */
void m1_rpc_task(void *param);

/**
 * @brief  Notify the RPC task that a new screen frame is ready.
 *         Call from m1_u8g2_nextpage() when streaming is active.
 */
void m1_rpc_notify_screen_update(void);

/**
 * @brief  Check if screen streaming is currently active.
 */
bool m1_rpc_screen_streaming_active(void);

/**
 * @brief  Send an RPC frame over USB CDC.
 */
void m1_rpc_send_frame(uint8_t cmd, uint8_t seq,
                       const uint8_t *payload, uint16_t len);

/**
 * @brief  Send ACK response.
 */
void m1_rpc_send_ack(uint8_t seq);

/**
 * @brief  Send NACK response with error code.
 */
void m1_rpc_send_nack(uint8_t seq, uint8_t error_code);

/*************************** E X T E R N S ************************************/

extern S_RPC_ScreenStream rpc_screen_stream;

/**
 * @brief  RPC mode flag — set true when first valid RPC frame is received.
 *         When true, debug printf output is routed to UART only (not USB CDC)
 *         to prevent corruption of RPC binary frames.
 */
extern volatile bool m1_rpc_active;

#endif /* M1_RPC_H_ */
