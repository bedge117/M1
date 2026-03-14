/* See COPYING.txt for license details. */

/*
 * m1_rpc.c
 *
 * M1 Remote Procedure Call (RPC) Protocol Handler
 *
 * Binary protocol for communication with the qMonstatek desktop app.
 * Runs over USB CDC, coexisting with the existing CLI by detecting
 * the sync byte (0xAA) as the first byte of each packet.
 *
 * M1 Project
 */

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "app_freertos.h"
#include "semphr.h"
#include "main.h"
#include "m1_compile_cfg.h"

#ifdef M1_APP_RPC_ENABLE

#include "m1_rpc.h"
#include "m1_usb_cdc_msc.h"
#include "m1_log_debug.h"
#include "m1_system.h"
#include "m1_tasks.h"
#include "m1_sdcard.h"
#include "m1_file_browser.h"
#include "m1_fw_update_bl.h"
#include "m1_lcd.h"
#include "m1_watchdog.h"
#include "u8g2.h"
#include "usbd_cdc_if.h"
#include "battery.h"
#include "m1_esp32_hal.h"
#include "esp_loader.h"
#include "stm32_port.h"
#include "app_common.h"
#include "ff.h"
#include "FreeRTOS_CLI.h"
#include "m1_power_ctl.h"
#include "m1_sys_init.h"

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG            "RPC"

/* CRC-16/CCITT polynomial */
#define CRC16_POLY              0x1021
#define CRC16_INIT              0xFFFF

/* Screen streaming defaults */
#define RPC_SCREEN_FPS_MIN      1
#define RPC_SCREEN_FPS_MAX      15
#define RPC_SCREEN_FPS_DEFAULT  10

/* USB CDC TX retry */
#define RPC_USB_TX_RETRIES      10
#define RPC_USB_TX_RETRY_MS     2

/* File operation buffer size */
#define RPC_FILE_CHUNK_SIZE     1024

/* FW update write destination offset in inactive bank */
#define RPC_FW_CHUNK_SIZE       FW_IMAGE_CHUNK_SIZE  /* 1024 bytes */

//************************** S T R U C T U R E S *******************************

/* Firmware update state */
typedef struct {
    bool     active;
    uint32_t total_size;
    uint32_t expected_crc32;
    uint32_t bytes_written;
    uint8_t *flash_addr;         /* Next write address in inactive bank */
} S_RPC_FwUpdateState;

/* ESP32 update state — direct flash via esp-serial-flasher */
typedef struct {
    bool     active;
    uint32_t total_size;
    uint32_t start_addr;
    uint32_t bytes_written;
    bool     screen_was_streaming;  /* restore screen stream after flash */
} S_RPC_EspUpdateState;

/* File write state */
typedef struct {
    bool     active;
    FIL      file;
    uint32_t total_size;
    uint32_t bytes_written;
} S_RPC_FileWriteState;

/***************************** V A R I A B L E S ******************************/

/* Parser state */
static E_RPC_ParseState s_parse_state = RPC_STATE_IDLE;
static S_RPC_Frame      s_rx_frame;
static uint8_t           s_header_buf[4];    /* CMD + SEQ + LEN(2) */
static uint16_t          s_header_idx;
static uint16_t          s_payload_idx;
static uint8_t           s_crc_buf[2];
static uint8_t           s_crc_idx;

/* TX frame buffer */
static uint8_t           s_tx_buf[RPC_MAX_FRAME_SIZE];
static SemaphoreHandle_t s_tx_mutex;

/* Screen streaming */
S_RPC_ScreenStream rpc_screen_stream;
static uint8_t     s_screen_fb_copy[RPC_SCREEN_FB_SIZE];

/* RPC mode flag — when true, debug output goes to UART only (not USB CDC) */
volatile bool m1_rpc_active = false;

/* RPC task */
static TaskHandle_t s_rpc_task_hdl;

/* FW update state */
static S_RPC_FwUpdateState  s_fw_update;

/* ESP32 update state */
static S_RPC_EspUpdateState s_esp_update;

/* File write state */
static S_RPC_FileWriteState s_file_write;

/* Deferred command processing — slow SD-card operations (delete, mkdir, etc.)
 * are copied here and processed in rpc_task instead of inline in the CDC task,
 * so the USB endpoint stays responsive during long FatFs operations. */
static S_RPC_Frame  s_deferred_frame;
static volatile bool s_deferred_pending = false;

/* CRC-16 lookup table */
static const uint16_t s_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x4864, 0x5845, 0x6826, 0x7807, 0x08E0, 0x18C1, 0x28A2, 0x38C3,
    0xC92C, 0xD90D, 0xE96E, 0xF94F, 0x89A8, 0x9989, 0xA9EA, 0xB9CB,
    0x5A15, 0x4A34, 0x7A57, 0x6A76, 0x1A91, 0x0AB0, 0x3AD3, 0x2AF2,
    0xDB1D, 0xCB3C, 0xFB5F, 0xEB7E, 0x9B99, 0x8BB8, 0xBBDB, 0xABFA,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBBBA,
    0x4A55, 0x5A74, 0x6A17, 0x7A36, 0x0AD1, 0x1AF0, 0x2A93, 0x3AB2,
    0xFD0E, 0xED2F, 0xDD4C, 0xCD6D, 0xBD8A, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

/* CRC */
static uint16_t rpc_crc16(const uint8_t *data, uint16_t len);
static uint16_t rpc_crc16_continue(uint16_t crc, const uint8_t *data, uint16_t len);

/* Frame handling */
static void rpc_parse_byte(uint8_t byte);
static void rpc_dispatch_frame(const S_RPC_Frame *frame);

/* USB CDC TX helper */
static void rpc_usb_transmit(const uint8_t *data, uint16_t len);

/* Command handlers */
static void rpc_handle_ping(const S_RPC_Frame *f);
static void rpc_handle_get_device_info(const S_RPC_Frame *f);
static void rpc_handle_reboot(const S_RPC_Frame *f);
static void rpc_handle_power_off(const S_RPC_Frame *f);
static void rpc_handle_screen_start(const S_RPC_Frame *f);
static void rpc_handle_screen_stop(const S_RPC_Frame *f);
static void rpc_handle_screen_capture(const S_RPC_Frame *f);
static void rpc_handle_button_press(const S_RPC_Frame *f);
static void rpc_handle_button_release(const S_RPC_Frame *f);
static void rpc_handle_button_click(const S_RPC_Frame *f);
static void rpc_handle_file_list(const S_RPC_Frame *f);
static void rpc_handle_file_read(const S_RPC_Frame *f);
static void rpc_handle_file_write_start(const S_RPC_Frame *f);
static void rpc_handle_file_write_data(const S_RPC_Frame *f);
static void rpc_handle_file_write_finish(const S_RPC_Frame *f);
static void rpc_handle_file_delete(const S_RPC_Frame *f);
static void rpc_handle_file_mkdir(const S_RPC_Frame *f);
static void rpc_handle_fw_info(const S_RPC_Frame *f);
static void rpc_handle_fw_update_start(const S_RPC_Frame *f);
static void rpc_handle_fw_update_data(const S_RPC_Frame *f);
static void rpc_handle_fw_update_finish(const S_RPC_Frame *f);
static void rpc_handle_fw_bank_swap(const S_RPC_Frame *f);
static void rpc_handle_fw_dfu_enter(const S_RPC_Frame *f);
static void rpc_handle_esp_info(const S_RPC_Frame *f);
static void rpc_handle_esp_update_start(const S_RPC_Frame *f);
static void rpc_handle_esp_update_data(const S_RPC_Frame *f);
static void rpc_handle_esp_update_finish(const S_RPC_Frame *f);
static void rpc_handle_cli_exec(const S_RPC_Frame *f);
static void rpc_handle_esp_uart_snoop(const S_RPC_Frame *f);

/* Screen streaming helpers */
static void rpc_send_screen_frame(uint8_t seq);
static void rpc_flip_buffer_180(uint8_t *buf, uint16_t size);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*                           C R C - 1 6                                      */
/*============================================================================*/

/**
 * @brief  Compute CRC-16/CCITT over a byte buffer.
 *         Polynomial: 0x1021, Initial value: 0xFFFF.
 */
static uint16_t rpc_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = CRC16_INIT;

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t idx = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (crc << 8) ^ s_crc16_table[idx];
    }

    return crc;
}


/**
 * @brief  Continue CRC-16/CCITT computation with a running CRC value.
 *         Used to compute CRC over non-contiguous buffers incrementally.
 */
static uint16_t rpc_crc16_continue(uint16_t crc, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t idx = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (crc << 8) ^ s_crc16_table[idx];
    }

    return crc;
}


/*============================================================================*/
/*                     U S B   C D C   T X   H E L P E R                      */
/*============================================================================*/

/**
 * @brief  Transmit data over USB CDC with retry logic.
 *         Must be called from task context (not ISR).
 */
static void rpc_usb_transmit(const uint8_t *data, uint16_t len)
{
    if (m1_USB_CDC_ready != 0)
    {
        M1_LOG_I(M1_LOGDB_TAG, "TX blocked: CDC_ready=%d\r\n", m1_USB_CDC_ready);
        return;
    }

    for (int retry = 0; retry < RPC_USB_TX_RETRIES; retry++)
    {
        uint8_t result = CDC_Transmit_FS((uint8_t *)data, len);
        if (result == USBD_OK)
        {
            M1_LOG_I(M1_LOGDB_TAG, "TX OK %u bytes cmd=0x%02X\r\n", len, data[1]);
            return;
        }
        M1_LOG_I(M1_LOGDB_TAG, "TX retry %d result=%d\r\n", retry, result);
        osDelay(RPC_USB_TX_RETRY_MS);
    }
    M1_LOG_I(M1_LOGDB_TAG, "TX FAILED after %d retries\r\n", RPC_USB_TX_RETRIES);
}


/*============================================================================*/
/*                     F R A M E   B U I L D E R                              */
/*============================================================================*/

/**
 * @brief  Build and send an RPC frame over USB CDC.
 *         Thread-safe via mutex.
 *
 * Frame: [0xAA] [CMD] [SEQ] [LEN_LO] [LEN_HI] [PAYLOAD...] [CRC_LO] [CRC_HI]
 *   CRC computed over CMD through end of PAYLOAD.
 */
void m1_rpc_send_frame(uint8_t cmd, uint8_t seq,
                       const uint8_t *payload, uint16_t len)
{
    if (len > RPC_MAX_PAYLOAD)
        return;

    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return;

    uint16_t frame_size = RPC_HEADER_SIZE + len + RPC_CRC_SIZE;
    uint16_t idx = 0;

    /* Sync byte */
    s_tx_buf[idx++] = RPC_SYNC_BYTE;

    /* Command */
    s_tx_buf[idx++] = cmd;

    /* Sequence number */
    s_tx_buf[idx++] = seq;

    /* Payload length (little-endian) */
    s_tx_buf[idx++] = (uint8_t)(len & 0xFF);
    s_tx_buf[idx++] = (uint8_t)(len >> 8);

    /* Payload */
    if (payload != NULL && len > 0)
    {
        memcpy(&s_tx_buf[idx], payload, len);
        idx += len;
    }

    /* CRC-16 over CMD + SEQ + LEN + PAYLOAD (bytes 1 through idx-1) */
    uint16_t crc = rpc_crc16(&s_tx_buf[1], idx - 1);
    s_tx_buf[idx++] = (uint8_t)(crc & 0xFF);
    s_tx_buf[idx++] = (uint8_t)(crc >> 8);

    /* Transmit */
    rpc_usb_transmit(s_tx_buf, frame_size);

    xSemaphoreGive(s_tx_mutex);
}


/**
 * @brief  Send an ACK response.
 */
void m1_rpc_send_ack(uint8_t seq)
{
    m1_rpc_send_frame(RPC_CMD_ACK, seq, NULL, 0);
}


/**
 * @brief  Send a NACK response with error code.
 */
void m1_rpc_send_nack(uint8_t seq, uint8_t error_code)
{
    m1_rpc_send_frame(RPC_CMD_NACK, seq, &error_code, 1);
}

/**
 * @brief  Send NACK with a sub-error byte for detailed diagnostics.
 *         Payload: [error_code] [sub_error]
 */
static void rpc_send_nack_sub(uint8_t seq, uint8_t error_code, uint8_t sub_error)
{
    uint8_t buf[2] = { error_code, sub_error };
    m1_rpc_send_frame(RPC_CMD_NACK, seq, buf, 2);
}


/*============================================================================*/
/*                     F R A M E   P A R S E R                                */
/*============================================================================*/

/**
 * @brief  Feed received bytes from USB CDC into the RPC parser.
 */
void m1_rpc_feed(const uint8_t *data, uint16_t len)
{
    M1_LOG_I(M1_LOGDB_TAG, "FEED %u bytes [%02X %02X %02X ...]\r\n",
             len,
             (len > 0) ? data[0] : 0,
             (len > 1) ? data[1] : 0,
             (len > 2) ? data[2] : 0);
    for (uint16_t i = 0; i < len; i++)
    {
        rpc_parse_byte(data[i]);
    }
}


/**
 * @brief  Check if the given data starts with an RPC sync byte.
 */
bool m1_rpc_is_sync(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return false;
    return (data[0] == RPC_SYNC_BYTE);
}


/**
 * @brief  Parse a single byte through the RPC frame state machine.
 *
 * States:
 *   IDLE    → Wait for 0xAA sync byte
 *   HEADER  → Accumulate CMD(1) + SEQ(1) + LEN(2) = 4 bytes
 *   PAYLOAD → Accumulate LEN payload bytes (may be 0)
 *   CRC     → Accumulate 2 CRC bytes, validate, dispatch
 */
static void rpc_parse_byte(uint8_t byte)
{
    switch (s_parse_state)
    {
    case RPC_STATE_IDLE:
        if (byte == RPC_SYNC_BYTE)
        {
            s_header_idx  = 0;
            s_payload_idx = 0;
            s_crc_idx     = 0;
            s_parse_state = RPC_STATE_HEADER;
        }
        break;

    case RPC_STATE_HEADER:
        s_header_buf[s_header_idx++] = byte;
        if (s_header_idx >= 4)
        {
            s_rx_frame.cmd = s_header_buf[0];
            s_rx_frame.seq = s_header_buf[1];
            s_rx_frame.len = (uint16_t)s_header_buf[2] |
                             ((uint16_t)s_header_buf[3] << 8);

            if (s_rx_frame.len > RPC_MAX_PAYLOAD)
            {
                /* Invalid length — reset */
                s_parse_state = RPC_STATE_IDLE;
            }
            else if (s_rx_frame.len == 0)
            {
                /* No payload — go straight to CRC */
                s_parse_state = RPC_STATE_CRC;
            }
            else
            {
                s_parse_state = RPC_STATE_PAYLOAD;
            }
        }
        break;

    case RPC_STATE_PAYLOAD:
        s_rx_frame.payload[s_payload_idx++] = byte;
        if (s_payload_idx >= s_rx_frame.len)
        {
            s_parse_state = RPC_STATE_CRC;
        }
        break;

    case RPC_STATE_CRC:
        s_crc_buf[s_crc_idx++] = byte;
        if (s_crc_idx >= 2)
        {
            /* Reconstruct received CRC (little-endian) */
            uint16_t rx_crc = (uint16_t)s_crc_buf[0] |
                              ((uint16_t)s_crc_buf[1] << 8);

            /* Compute CRC over header(4) + payload incrementally
             * without copying to a temp buffer */
            uint16_t computed_crc = rpc_crc16(s_header_buf, 4);
            if (s_rx_frame.len > 0)
            {
                computed_crc = rpc_crc16_continue(computed_crc,
                                                   s_rx_frame.payload,
                                                   s_rx_frame.len);
            }

            if (rx_crc == computed_crc)
            {
                rpc_dispatch_frame(&s_rx_frame);
            }
            else
            {
                M1_LOG_I(M1_LOGDB_TAG, "CRC MISMATCH cmd=0x%02X rx=0x%04X calc=0x%04X\r\n",
                         s_rx_frame.cmd, rx_crc, computed_crc);
            }

            s_parse_state = RPC_STATE_IDLE;
        }
        break;

    default:
        s_parse_state = RPC_STATE_IDLE;
        break;
    }
}


/*============================================================================*/
/*                  C O M M A N D   D I S P A T C H E R                       */
/*============================================================================*/

/**
 * @brief  Route a validated frame to the appropriate handler.
 */
static void rpc_dispatch_frame(const S_RPC_Frame *frame)
{
    M1_LOG_I(M1_LOGDB_TAG, "DISPATCH cmd=0x%02X seq=%u len=%u\r\n",
             frame->cmd, frame->seq, frame->len);

    /* First valid RPC frame received — suppress debug CDC output */
    if (!m1_rpc_active)
    {
        m1_rpc_active = true;
    }

    switch (frame->cmd)
    {
    /* System */
    case RPC_CMD_PING:            rpc_handle_ping(frame);            break;
    case RPC_CMD_GET_DEVICE_INFO: rpc_handle_get_device_info(frame); break;
    case RPC_CMD_REBOOT:          rpc_handle_reboot(frame);          break;
    case RPC_CMD_POWER_OFF:       rpc_handle_power_off(frame);       break;

    /* Screen */
    case RPC_CMD_SCREEN_START:    rpc_handle_screen_start(frame);    break;
    case RPC_CMD_SCREEN_STOP:     rpc_handle_screen_stop(frame);     break;
    case RPC_CMD_SCREEN_CAPTURE:  rpc_handle_screen_capture(frame);  break;

    /* Input */
    case RPC_CMD_BUTTON_PRESS:    rpc_handle_button_press(frame);    break;
    case RPC_CMD_BUTTON_RELEASE:  rpc_handle_button_release(frame);  break;
    case RPC_CMD_BUTTON_CLICK:    rpc_handle_button_click(frame);    break;

    /* File — read-only operations stay inline */
    case RPC_CMD_FILE_LIST:        rpc_handle_file_list(frame);        break;
    case RPC_CMD_FILE_READ:        rpc_handle_file_read(frame);        break;

    /* File — write operations deferred to rpc_task.
     * The Usb2SerTask has limited stack (8KB) and SD card writes can
     * block long enough to cause USB CDC timeouts or watchdog resets.
     * The rpc_task has 16KB stack and doesn't block the USB endpoint. */
    case RPC_CMD_FILE_WRITE_START:
    case RPC_CMD_FILE_WRITE_DATA:
    case RPC_CMD_FILE_WRITE_FINISH:
    case RPC_CMD_FILE_DELETE:
    case RPC_CMD_FILE_MKDIR:
        if (s_deferred_pending)
        {
            m1_rpc_send_nack(frame->seq, RPC_ERR_BUSY);
        }
        else
        {
            memcpy(&s_deferred_frame, frame, sizeof(S_RPC_Frame));
            s_deferred_pending = true;
            xTaskNotifyGive(s_rpc_task_hdl);
        }
        break;

    /* Firmware */
    case RPC_CMD_FW_INFO:          rpc_handle_fw_info(frame);          break;
    case RPC_CMD_FW_UPDATE_START:  rpc_handle_fw_update_start(frame);  break;
    case RPC_CMD_FW_UPDATE_DATA:   rpc_handle_fw_update_data(frame);   break;
    case RPC_CMD_FW_UPDATE_FINISH: rpc_handle_fw_update_finish(frame); break;
    case RPC_CMD_FW_BANK_SWAP:     rpc_handle_fw_bank_swap(frame);     break;
    case RPC_CMD_FW_DFU_ENTER:     rpc_handle_fw_dfu_enter(frame);     break;

    /* ESP32 */
    case RPC_CMD_ESP_INFO:          rpc_handle_esp_info(frame);          break;
    case RPC_CMD_ESP_UPDATE_DATA:   rpc_handle_esp_update_data(frame);   break;

    /* ESP32 flash start/finish — deferred to rpc_task because
     * connect_to_target(), flash erase, and flash verify are slow
     * (seconds) and would stall the USB CDC endpoint. */
    case RPC_CMD_ESP_UPDATE_START:
    case RPC_CMD_ESP_UPDATE_FINISH:
        if (s_deferred_pending)
        {
            m1_rpc_send_nack(frame->seq, RPC_ERR_BUSY);
        }
        else
        {
            memcpy(&s_deferred_frame, frame, sizeof(S_RPC_Frame));
            s_deferred_pending = true;
            xTaskNotifyGive(s_rpc_task_hdl);
        }
        break;

    /* Debug / CLI — deferred to rpc_task (16KB stack).
     * mtest subcommands may trigger SD/I2C/SPI operations that
     * overflow the 8KB Usb2SerTask stack. */
    case RPC_CMD_CLI_EXEC:
        if (s_deferred_pending)
        {
            m1_rpc_send_nack(frame->seq, RPC_ERR_BUSY);
        }
        else
        {
            memcpy(&s_deferred_frame, frame, sizeof(S_RPC_Frame));
            s_deferred_pending = true;
            xTaskNotifyGive(s_rpc_task_hdl);
        }
        break;
    case RPC_CMD_ESP_UART_SNOOP:    rpc_handle_esp_uart_snoop(frame);    break;

    default:
        m1_rpc_send_nack(frame->seq, RPC_ERR_UNKNOWN_CMD);
        break;
    }
}


/*============================================================================*/
/*                  S Y S T E M   C O M M A N D S                             */
/*============================================================================*/

/**
 * @brief  Handle PING — respond with PONG.
 */
static void rpc_handle_ping(const S_RPC_Frame *f)
{
    m1_rpc_send_frame(RPC_CMD_PONG, f->seq, NULL, 0);
}


/**
 * @brief  Handle GET_DEVICE_INFO — gather all device state and respond.
 */
static void rpc_handle_get_device_info(const S_RPC_Frame *f)
{
    S_RPC_DeviceInfo info;
    memset(&info, 0, sizeof(info));

    /* Magic */
    info.magic = RPC_DEVICE_INFO_MAGIC;

    /* Firmware version from flash config */
    info.fw_version_major = m1_fw_config.fw_version_major;
    info.fw_version_minor = m1_fw_config.fw_version_minor;
    info.fw_version_build = m1_fw_config.fw_version_build;
    info.fw_version_rc    = m1_fw_config.fw_version_rc;

    /* Active bank */
    info.active_bank = bl_get_active_bank();

    /* Battery */
    S_M1_Power_Status_t pwr;
    battery_power_status_get(&pwr);
    info.battery_level    = pwr.battery_level;
    info.battery_charging = (pwr.stat != 0) ? 1 : 0;
    info.batt_voltage_mv  = (uint16_t)(pwr.battery_voltage * 1000.0f);
    info.batt_current_ma  = (int16_t)pwr.consumption_current;
    info.batt_temp_c      = pwr.battery_temp;
    info.batt_health      = pwr.battery_health;
    info.charge_state     = pwr.stat;
    info.charge_fault     = pwr.fault;

    /* SD card */
    info.sd_card_present = m1_sd_detected();
    if (info.sd_card_present)
    {
        S_M1_SDCard_Info *sd_info = m1_sdcard_get_info();
        if (sd_info != NULL)
        {
            info.sd_total_kb = sd_info->total_cap_kb;
            info.sd_free_kb  = sd_info->free_cap_kb;
        }
    }

    /* ESP32 */
    info.esp32_ready = m1_esp32_get_init_status();
    /* ESP32 version string: not readily available from HAL.
     * Set to build-time constant for now. */
    strncpy(info.esp32_version,
            info.esp32_ready ? "AT ready" : "offline",
            sizeof(info.esp32_version) - 1);

    /* Operation mode & settings */
    info.ism_band_region = m1_fw_config.ism_band_region;
    info.op_mode         = (uint8_t)m1_device_stat.op_mode;
    info.southpaw_mode   = m1_southpaw_mode;
    info.c3_revision     = M1_C3_REVISION;

    m1_rpc_send_frame(RPC_CMD_DEVICE_INFO_RESP, f->seq,
                      (const uint8_t *)&info, sizeof(info));
}


/**
 * @brief  Handle REBOOT — clean reset, matching the M1 device menu reboot.
 *
 * The M1's own Power → Reboot path writes DEV_OP_STATUS_REBOOT to the
 * backup register, calls m1_pre_power_down() (which shuts down peripherals
 * and drives ESP32_EN LOW via m1_system_GPIO_init()), then NVIC_SystemReset().
 * A raw NVIC_SystemReset() skips all of that, leaving the ESP32 and other
 * peripherals in indeterminate states during the brief GPIO-floating window
 * before MX_GPIO_Init() runs on the next boot.  This caused ESP32 flashing
 * to fail after a qMonstatek-initiated reboot.
 */
static void rpc_handle_reboot(const S_RPC_Frame *f)
{
    m1_rpc_send_ack(f->seq);
    osDelay(100);  /* Allow ACK to be transmitted */
    startup_config_write(BK_REGS_SELECT_DEV_OP_STAT, DEV_OP_STATUS_REBOOT);
    m1_pre_power_down();
    NVIC_SystemReset();
}


/**
 * @brief  Handle POWER_OFF — shutdown the device directly.
 *
 * Cannot call power_off() here — it enters the UI view loop which
 * requires the main task context.  m1_power_down() performs the actual
 * hardware shutdown (LEDs off, display off, BQ ship mode, standby).
 * The user already confirmed in qMonstatek, so no on-device
 * confirmation screen is needed.
 */
static void rpc_handle_power_off(const S_RPC_Frame *f)
{
    m1_rpc_send_ack(f->seq);
    osDelay(100);  /* Allow ACK to be transmitted */
    m1_power_down();
}


/*============================================================================*/
/*                  S C R E E N   C O M M A N D S                             */
/*============================================================================*/

/**
 * @brief  Handle SCREEN_START — begin streaming at requested FPS.
 *         Payload: fps(1 byte), 1-15.
 */
static void rpc_handle_screen_start(const S_RPC_Frame *f)
{
    uint8_t fps = RPC_SCREEN_FPS_DEFAULT;

    if (f->len >= 1)
    {
        fps = f->payload[0];
        if (fps < RPC_SCREEN_FPS_MIN) fps = RPC_SCREEN_FPS_MIN;
        if (fps > RPC_SCREEN_FPS_MAX) fps = RPC_SCREEN_FPS_MAX;
    }

    rpc_screen_stream.fps          = fps;
    rpc_screen_stream.interval_ms  = 1000 / fps;
    rpc_screen_stream.last_send_tick = xTaskGetTickCount();
    rpc_screen_stream.active       = true;

    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle SCREEN_STOP — stop streaming.
 */
static void rpc_handle_screen_stop(const S_RPC_Frame *f)
{
    rpc_screen_stream.active = false;
    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle SCREEN_CAPTURE — send a single frame immediately.
 */
static void rpc_handle_screen_capture(const S_RPC_Frame *f)
{
    rpc_send_screen_frame(f->seq);
}


/**
 * @brief  Reverse bits within a byte.
 *         bit 0 ↔ bit 7, bit 1 ↔ bit 6, etc.
 */
static inline uint8_t rpc_reverse_bits(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
    b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
    b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
    return b;
}


/**
 * @brief  Rotate a u8g2 tile-format framebuffer 180° in-place.
 *
 *         In the tile layout, position p mirrors to (size-1-p).
 *         Swap each pair and reverse the bits within each byte
 *         (which flips the 8 vertical pixels within each tile column).
 */
static void rpc_flip_buffer_180(uint8_t *buf, uint16_t size)
{
    uint16_t half = size / 2;
    for (uint16_t i = 0; i < half; i++)
    {
        uint16_t j = size - 1 - i;
        uint8_t a = rpc_reverse_bits(buf[i]);
        uint8_t b = rpc_reverse_bits(buf[j]);
        buf[i] = b;
        buf[j] = a;
    }
}


/**
 * @brief  Copy framebuffer and send as SCREEN_FRAME.
 *         Uses a critical section to prevent tearing.
 *
 *         The buffer is always normalized to logical display orientation
 *         before sending.  When the firmware uses U8G2_R2 (normal mode),
 *         the raw buffer is rotated 180° from the logical view, so we
 *         flip it.  When U8G2_R0 (southpaw), the buffer is already in
 *         logical orientation and is sent as-is.  This means the desktop
 *         app never needs to know about southpaw — it always receives
 *         correctly-oriented data.
 */
static void rpc_send_screen_frame(uint8_t seq)
{
    uint8_t *fb = u8g2_GetBufferPtr(&m1_u8g2);

    /* Copy framebuffer atomically, snapshot southpaw state too */
    bool need_flip;
    taskENTER_CRITICAL();
    memcpy(s_screen_fb_copy, fb, RPC_SCREEN_FB_SIZE);
    need_flip = (m1_southpaw_mode == 0);   /* R2 mode → buffer is 180° rotated */
    taskEXIT_CRITICAL();

    /* Normalize orientation so desktop always gets correct-side-up data */
    if (need_flip)
    {
        rpc_flip_buffer_180(s_screen_fb_copy, RPC_SCREEN_FB_SIZE);
    }

    m1_rpc_send_frame(RPC_CMD_SCREEN_FRAME, seq,
                      s_screen_fb_copy, RPC_SCREEN_FB_SIZE);
}


/**
 * @brief  Notify the RPC task that a new screen frame is ready.
 *         Called from m1_u8g2_nextpage() when streaming is active.
 */
void m1_rpc_notify_screen_update(void)
{
    if (s_rpc_task_hdl != NULL)
    {
        /* Called from task context (m1_u8g2_nextpage) — use task-safe variant */
        xTaskNotifyGive(s_rpc_task_hdl);
    }
}


/**
 * @brief  Check if screen streaming is currently active.
 */
bool m1_rpc_screen_streaming_active(void)
{
    return rpc_screen_stream.active;
}


/*============================================================================*/
/*                    I N P U T   C O M M A N D S                             */
/*============================================================================*/

/**
 * @brief  Inject a button event into the M1 input system.
 */
static void rpc_inject_button_event(uint8_t button_id, S_M1_Key_Event event)
{
    if (button_id >= NUM_BUTTONS_MAX)
        return;

    /* Build a button status snapshot with only this button active */
    S_M1_Buttons_Status btn_status;
    memset(&btn_status, 0, sizeof(btn_status));
    btn_status.event[button_id] = event;
    btn_status.timestamp = xTaskGetTickCount();

    /* Post to button_events_q first — the menu task reads events from here */
    xQueueSend(button_events_q_hdl, &btn_status, 0);

    /* Then notify the main queue so the menu task wakes up */
    S_M1_Main_Q_t q_msg;
    memset(&q_msg, 0, sizeof(q_msg));
    q_msg.q_evt_type = Q_EVENT_KEYPAD;
    q_msg.q_data.keypad_evt = 1;
    xQueueSend(main_q_hdl, &q_msg, 0);
}


/**
 * @brief  Handle BUTTON_PRESS — inject a press event.
 */
static void rpc_handle_button_press(const S_RPC_Frame *f)
{
    if (f->len < 1)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }
    rpc_inject_button_event(f->payload[0], BUTTON_EVENT_CLICK);
    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle BUTTON_RELEASE — inject a release (idle) event.
 *         The M1 button system doesn't have explicit release, so this is a no-op ACK.
 */
static void rpc_handle_button_release(const S_RPC_Frame *f)
{
    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle BUTTON_CLICK — inject a click event (press + release).
 */
static void rpc_handle_button_click(const S_RPC_Frame *f)
{
    if (f->len < 1)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }
    rpc_inject_button_event(f->payload[0], BUTTON_EVENT_CLICK);
    m1_rpc_send_ack(f->seq);
}


/*============================================================================*/
/*                    F I L E   C O M M A N D S                               */
/*============================================================================*/

/**
 * @brief  Build a full SD card path from the RPC payload path.
 *         Prepends "0:/" if the path doesn't already start with it.
 */
static void rpc_build_sd_path(char *out, size_t out_size,
                               const uint8_t *payload, uint16_t len)
{
    /* Ensure null-terminated string from payload */
    char raw_path[256];
    uint16_t copy_len = (len < sizeof(raw_path) - 1) ? len : sizeof(raw_path) - 1;
    memcpy(raw_path, payload, copy_len);
    raw_path[copy_len] = '\0';

    /* Check if path already has drive prefix */
    if (raw_path[0] == '0' && raw_path[1] == ':')
    {
        strncpy(out, raw_path, out_size - 1);
        out[out_size - 1] = '\0';
    }
    else
    {
        /* Prepend drive path */
        if (raw_path[0] == '/')
            snprintf(out, out_size, "0:%s", raw_path);
        else
            snprintf(out, out_size, "0:/%s", raw_path);
    }
}


/**
 * @brief  Handle FILE_LIST — list directory contents.
 *
 * Payload: path string (null-terminated or length-delimited)
 * Response: FILE_LIST_RESP with entries:
 *   Each entry: [type:1] [size:4 LE] [name_len:1] [name:N]
 *   type: 0=file, 1=directory
 *   Terminated by type=0xFF
 */
static void rpc_handle_file_list(const S_RPC_Frame *f)
{
    /* Large buffers are static to avoid stack overflow — the USB CDC
     * task that calls this has only 2 KB of stack.  RPC dispatch is
     * single-threaded so this is safe. */
    static char    path[256];
    static uint8_t resp[RPC_MAX_PAYLOAD];
    static DIR     dir;
    static FILINFO fno;

    M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: len=%u\r\n", f->len);

    if (!m1_sd_detected())
    {
        M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: SD not detected, NACK\r\n");
        m1_rpc_send_nack(f->seq, RPC_ERR_SD_NOT_READY);
        return;
    }

    if (f->len > 0)
        rpc_build_sd_path(path, sizeof(path), f->payload, f->len);
    else
        strcpy(path, SDCARD_DEFAULT_DRIVE_PATH);

    M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: path='%s'\r\n", path);

    FRESULT res;

    res = f_opendir(&dir, path);
    if (res != FR_OK)
    {
        M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: f_opendir FAILED res=%d\r\n", res);
        m1_rpc_send_nack(f->seq, RPC_ERR_FILE_NOT_FOUND);
        return;
    }

    M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: f_opendir OK\r\n");

    /* Build response payload:
     *   [path string + null]
     *   repeated: [is_dir:1] [size:4 LE] [date:2 LE] [time:2 LE] [name + null]
     *
     * This matches the FileEntry layout in rpc_protocol.h and what the
     * desktop parser (handleFileListResp) expects.
     */
    uint16_t resp_len = 0;
    uint16_t entry_count = 0;

    /* Path string (null-terminated) */
    uint16_t path_len = (uint16_t)strlen(path);
    if (path_len + 1 > RPC_MAX_PAYLOAD)
    {
        f_closedir(&dir);
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }
    memcpy(&resp[resp_len], path, path_len + 1);   /* include null */
    resp_len += path_len + 1;

    for (;;)
    {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == '\0')
        {
            M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: f_readdir end res=%d fname[0]=%d\r\n",
                     res, (int)fno.fname[0]);
            break;  /* End of directory or error */
        }

        /* Skip hidden/system files */
        if (fno.fattrib & (AM_HID | AM_SYS))
            continue;

        uint16_t name_len = (uint16_t)strlen(fno.fname);
        /* Entry: is_dir(1) + size(4) + date(2) + time(2) + name + null */
        uint16_t entry_size = 1 + 4 + 2 + 2 + name_len + 1;

        /* Check if entry fits */
        if (resp_len + entry_size > RPC_MAX_PAYLOAD)
        {
            M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: truncated, buffer full\r\n");
            break;  /* Truncate — too many entries */
        }

        M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: entry '%s' %s size=%lu\r\n",
                 fno.fname,
                 (fno.fattrib & AM_DIR) ? "DIR" : "FILE",
                 (unsigned long)fno.fsize);

        /* is_dir: 0=file, 1=directory */
        resp[resp_len++] = (fno.fattrib & AM_DIR) ? 1 : 0;

        /* File size (4 bytes, little-endian) */
        uint32_t fsize = (uint32_t)fno.fsize;
        resp[resp_len++] = (uint8_t)(fsize & 0xFF);
        resp[resp_len++] = (uint8_t)((fsize >> 8) & 0xFF);
        resp[resp_len++] = (uint8_t)((fsize >> 16) & 0xFF);
        resp[resp_len++] = (uint8_t)((fsize >> 24) & 0xFF);

        /* FAT date (2 bytes, little-endian) */
        resp[resp_len++] = (uint8_t)(fno.fdate & 0xFF);
        resp[resp_len++] = (uint8_t)((fno.fdate >> 8) & 0xFF);

        /* FAT time (2 bytes, little-endian) */
        resp[resp_len++] = (uint8_t)(fno.ftime & 0xFF);
        resp[resp_len++] = (uint8_t)((fno.ftime >> 8) & 0xFF);

        /* Name (null-terminated) */
        memcpy(&resp[resp_len], fno.fname, name_len);
        resp_len += name_len;
        resp[resp_len++] = '\0';

        entry_count++;
    }

    f_closedir(&dir);

    M1_LOG_I(M1_LOGDB_TAG, "FILE_LIST: sending %u entries, %u bytes\r\n",
             entry_count, resp_len);

    m1_rpc_send_frame(RPC_CMD_FILE_LIST_RESP, f->seq, resp, resp_len);
}


/**
 * @brief  Handle FILE_READ — read a file and send its contents in chunks.
 *
 * Payload: path string
 * Response: Multiple FILE_READ_DATA frames:
 *   [offset:4 LE] [data:N]
 *   Final frame has len < chunk_size (or 0 for exact multiple).
 */
static void rpc_handle_file_read(const S_RPC_Frame *f)
{
    /* Large buffers are static to avoid stack overflow — the USB CDC
     * task that calls this has only 2 KB of stack. */
    static char    path[256];
    static FIL     file;
    static uint8_t chunk_buf[4 + RPC_FILE_CHUNK_SIZE];  /* offset(4) + data */

    if (f->len < 1)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    if (!m1_sd_detected())
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_SD_NOT_READY);
        return;
    }

    rpc_build_sd_path(path, sizeof(path), f->payload, f->len);

    FRESULT res = f_open(&file, path, FA_READ);
    if (res != FR_OK)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_FILE_NOT_FOUND);
        return;
    }

    uint32_t offset = 0;
    UINT bytes_read;
    uint8_t seq = f->seq;

    for (;;)
    {
        /* Offset (little-endian) */
        chunk_buf[0] = (uint8_t)(offset & 0xFF);
        chunk_buf[1] = (uint8_t)((offset >> 8) & 0xFF);
        chunk_buf[2] = (uint8_t)((offset >> 16) & 0xFF);
        chunk_buf[3] = (uint8_t)((offset >> 24) & 0xFF);

        res = f_read(&file, &chunk_buf[4], RPC_FILE_CHUNK_SIZE, &bytes_read);
        if (res != FR_OK)
        {
            f_close(&file);
            m1_rpc_send_nack(seq, RPC_ERR_FILE_NOT_FOUND);
            return;
        }

        m1_rpc_send_frame(RPC_CMD_FILE_READ_DATA, seq,
                          chunk_buf, 4 + bytes_read);

        offset += bytes_read;
        seq++;  /* Increment seq for multi-frame response */

        /* If we read less than a full chunk, we're done */
        if (bytes_read < RPC_FILE_CHUNK_SIZE)
            break;

        /* Small delay to avoid flooding USB */
        osDelay(2);
    }

    f_close(&file);
}


/**
 * @brief  Handle FILE_WRITE_START — begin writing a file.
 *
 * Payload: [total_size:4 LE] [path string]
 */
static void rpc_handle_file_write_start(const S_RPC_Frame *f)
{
    m1_wdt_reset();

    if (f->len < 5)  /* Need at least 4 bytes size + 1 byte path */
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    if (!m1_sd_detected())
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_SD_NOT_READY);
        return;
    }

    /* Abort any previous write */
    if (s_file_write.active)
    {
        f_close(&s_file_write.file);
        s_file_write.active = false;
    }

    /* Parse total size */
    s_file_write.total_size = (uint32_t)f->payload[0] |
                              ((uint32_t)f->payload[1] << 8) |
                              ((uint32_t)f->payload[2] << 16) |
                              ((uint32_t)f->payload[3] << 24);

    /* Build path */
    char path[256];
    rpc_build_sd_path(path, sizeof(path), &f->payload[4], f->len - 4);

    /* Open file for writing (create/truncate) */
    FRESULT res = f_open(&s_file_write.file, path,
                         FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_FILE_NOT_FOUND);
        return;
    }

    s_file_write.bytes_written = 0;
    s_file_write.active = true;

    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle FILE_WRITE_DATA — write a chunk to the current file.
 *
 * Payload: [offset:4 LE] [data:N]
 */
static void rpc_handle_file_write_data(const S_RPC_Frame *f)
{
    m1_wdt_reset();

    if (!s_file_write.active)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_BUSY);
        return;
    }

    if (f->len < 5)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    uint32_t offset = (uint32_t)f->payload[0] |
                      ((uint32_t)f->payload[1] << 8) |
                      ((uint32_t)f->payload[2] << 16) |
                      ((uint32_t)f->payload[3] << 24);

    uint16_t data_len = f->len - 4;

    /* Seek to offset */
    f_lseek(&s_file_write.file, offset);

    /* Write data */
    UINT written;
    FRESULT res = f_write(&s_file_write.file, &f->payload[4],
                          data_len, &written);
    if (res != FR_OK || written != data_len)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_FILE_NOT_FOUND);
        return;
    }

    s_file_write.bytes_written += written;
    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle FILE_WRITE_FINISH — close the file being written.
 */
static void rpc_handle_file_write_finish(const S_RPC_Frame *f)
{
    m1_wdt_reset();

    if (!s_file_write.active)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_BUSY);
        return;
    }

    f_sync(&s_file_write.file);
    f_close(&s_file_write.file);
    s_file_write.active = false;

    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle FILE_DELETE — delete a file or empty directory.
 */
static void rpc_handle_file_delete(const S_RPC_Frame *f)
{
    if (f->len < 1)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    if (!m1_sd_detected())
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_SD_NOT_READY);
        return;
    }

    char path[256];
    rpc_build_sd_path(path, sizeof(path), f->payload, f->len);

    FRESULT res = f_unlink(path);
    if (res != FR_OK)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_FILE_NOT_FOUND);
        return;
    }

    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle FILE_MKDIR — create a directory.
 */
static void rpc_handle_file_mkdir(const S_RPC_Frame *f)
{
    if (f->len < 1)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    if (!m1_sd_detected())
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_SD_NOT_READY);
        return;
    }

    char path[256];
    rpc_build_sd_path(path, sizeof(path), f->payload, f->len);

    FRESULT res = f_mkdir(path);
    if (res != FR_OK && res != FR_EXIST)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_FILE_NOT_FOUND);
        return;
    }

    m1_rpc_send_ack(f->seq);
}


/*============================================================================*/
/*                F I R M W A R E   C O M M A N D S                           */
/*============================================================================*/

/**
 * @brief  Handle FW_INFO — return firmware bank information.
 */
static void rpc_handle_fw_info(const S_RPC_Frame *f)
{
    S_RPC_FwInfo info;
    memset(&info, 0, sizeof(info));

    info.active_bank = bl_get_active_bank();

    /* Resolve physical bank addresses.
     * The active bank is always mapped at 0x08000000, inactive at 0x08100000.
     * When SWAP_BANK is set, physical bank 2 sits at 0x08000000 and
     * physical bank 1 sits at 0x08100000.  We need info.bank1 to always
     * contain physical bank 1 data and info.bank2 physical bank 2 data,
     * regardless of which bank is active. */
    uint32_t phys_bank1_base, phys_bank2_base;
    if (info.active_bank == BANK1_ACTIVE) {
        phys_bank1_base = FW_START_ADDRESS;                      /* 0x08000000 */
        phys_bank2_base = FW_START_ADDRESS + M1_FLASH_BANK_SIZE; /* 0x08100000 */
    } else {
        phys_bank1_base = FW_START_ADDRESS + M1_FLASH_BANK_SIZE; /* 0x08100000 */
        phys_bank2_base = FW_START_ADDRESS;                      /* 0x08000000 */
    }

    uint32_t cfg_offset     = FW_CONFIG_RESERVED_ADDRESS - FW_START_ADDRESS;  /* 0xFFC00 */
    uint32_t crc_ext_offset = FW_CRC_EXT_BASE - FW_START_ADDRESS;            /* 0xFFC14 */

    /* --- Physical Bank 1 info ---
     * Use ECC-safe reads so a corrupted inactive bank cannot trigger
     * an unrecoverable NMI / device reset. */
    {
        uint32_t cfg_addr = phys_bank1_base + cfg_offset;
        uint32_t magic1, magic2;

        if (bl_safe_flash_read_u32(cfg_addr + 0, &magic1) &&
            bl_safe_flash_read_u32(cfg_addr + 16, &magic2) &&
            magic1 == FW_CONFIG_MAGIC_NUMBER_1 &&
            magic2 == FW_CONFIG_MAGIC_NUMBER_2)
        {
            /* Version word at offset 4: [rc, build, minor, major] */
            uint32_t ver_word;
            if (bl_safe_flash_read_u32(cfg_addr + 4, &ver_word)) {
                info.bank1.fw_version_rc    = (uint8_t)(ver_word & 0xFF);
                info.bank1.fw_version_build = (uint8_t)((ver_word >> 8) & 0xFF);
                info.bank1.fw_version_minor = (uint8_t)((ver_word >> 16) & 0xFF);
                info.bank1.fw_version_major = (uint8_t)((ver_word >> 24) & 0xFF);
            }

            uint32_t crc_addr = phys_bank1_base + crc_ext_offset;
            uint32_t crc_magic;
            if (bl_safe_flash_read_u32(crc_addr + 0, &crc_magic) &&
                crc_magic == FW_CRC_EXT_MAGIC_VALUE)
            {
                uint32_t img_size, crc_val;
                if (bl_safe_flash_read_u32(crc_addr + 4, &img_size) &&
                    bl_safe_flash_read_u32(crc_addr + 8, &crc_val))
                {
                    info.bank1.image_size = img_size;
                    info.bank1.crc32      = crc_val;
                    m1_wdt_reset();
                    info.bank1.crc_valid  = bl_verify_bank_crc(phys_bank1_base) ? 1 : 0;
                }
            }

            /* C3 build metadata at offset 32 */
            uint32_t c3_addr = phys_bank1_base + cfg_offset + FW_C3_META_BASE_OFFSET;
            uint32_t c3_magic;
            if (bl_safe_flash_read_u32(c3_addr + 0, &c3_magic) &&
                c3_magic == FW_C3_META_MAGIC_VALUE)
            {
                uint32_t rev_word;
                if (bl_safe_flash_read_u32(c3_addr + 4, &rev_word))
                    info.bank1.c3_revision = (uint8_t)(rev_word & 0xFF);

                /* Build date string at offset +8 (up to 19 chars + null) */
                for (int i = 0; i < 20; i += 4) {
                    uint32_t w;
                    if (bl_safe_flash_read_u32(c3_addr + 8 + i, &w))
                        memcpy(&info.bank1.build_date[i], &w, 4);
                }
                info.bank1.build_date[19] = '\0';
            }
        }
    }

    /* --- Physical Bank 2 info --- */
    {
        uint32_t cfg_addr = phys_bank2_base + cfg_offset;
        uint32_t magic1, magic2;

        if (bl_safe_flash_read_u32(cfg_addr + 0, &magic1) &&
            bl_safe_flash_read_u32(cfg_addr + 16, &magic2) &&
            magic1 == FW_CONFIG_MAGIC_NUMBER_1 &&
            magic2 == FW_CONFIG_MAGIC_NUMBER_2)
        {
            uint32_t ver_word;
            if (bl_safe_flash_read_u32(cfg_addr + 4, &ver_word)) {
                info.bank2.fw_version_rc    = (uint8_t)(ver_word & 0xFF);
                info.bank2.fw_version_build = (uint8_t)((ver_word >> 8) & 0xFF);
                info.bank2.fw_version_minor = (uint8_t)((ver_word >> 16) & 0xFF);
                info.bank2.fw_version_major = (uint8_t)((ver_word >> 24) & 0xFF);
            }

            uint32_t crc_addr = phys_bank2_base + crc_ext_offset;
            uint32_t crc_magic;
            if (bl_safe_flash_read_u32(crc_addr + 0, &crc_magic) &&
                crc_magic == FW_CRC_EXT_MAGIC_VALUE)
            {
                uint32_t img_size, crc_val;
                if (bl_safe_flash_read_u32(crc_addr + 4, &img_size) &&
                    bl_safe_flash_read_u32(crc_addr + 8, &crc_val))
                {
                    info.bank2.image_size = img_size;
                    info.bank2.crc32      = crc_val;
                    m1_wdt_reset();
                    info.bank2.crc_valid  = bl_verify_bank_crc(phys_bank2_base) ? 1 : 0;
                }
            }

            /* C3 build metadata at offset 32 */
            uint32_t c3_addr = phys_bank2_base + cfg_offset + FW_C3_META_BASE_OFFSET;
            uint32_t c3_magic;
            if (bl_safe_flash_read_u32(c3_addr + 0, &c3_magic) &&
                c3_magic == FW_C3_META_MAGIC_VALUE)
            {
                uint32_t rev_word;
                if (bl_safe_flash_read_u32(c3_addr + 4, &rev_word))
                    info.bank2.c3_revision = (uint8_t)(rev_word & 0xFF);

                for (int i = 0; i < 20; i += 4) {
                    uint32_t w;
                    if (bl_safe_flash_read_u32(c3_addr + 8 + i, &w))
                        memcpy(&info.bank2.build_date[i], &w, 4);
                }
                info.bank2.build_date[19] = '\0';
            }
        }
    }

    m1_rpc_send_frame(RPC_CMD_FW_INFO_RESP, f->seq,
                      (const uint8_t *)&info, sizeof(info));
}


/**
 * @brief  Handle FW_UPDATE_START — prepare to receive firmware image.
 *
 * Payload: [total_size:4 LE] [expected_crc32:4 LE]
 *
 * Erases the inactive flash bank, preparing for sequential writes.
 */
static void rpc_handle_fw_update_start(const S_RPC_Frame *f)
{
    if (f->len < 8)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    uint32_t total_size = (uint32_t)f->payload[0] |
                          ((uint32_t)f->payload[1] << 8) |
                          ((uint32_t)f->payload[2] << 16) |
                          ((uint32_t)f->payload[3] << 24);

    uint32_t expected_crc = (uint32_t)f->payload[4] |
                            ((uint32_t)f->payload[5] << 8) |
                            ((uint32_t)f->payload[6] << 16) |
                            ((uint32_t)f->payload[7] << 24);

    if (total_size > FW_IMAGE_SIZE_MAX)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_SIZE_TOO_LARGE);
        return;
    }

    /* Determine inactive bank base address.
     * The inactive bank is ALWAYS at the upper address (0x08100000) in the
     * mapped address space, regardless of SWAP_BANK setting:
     *   SWAP_BANK=0: 0x08000000=Bank1(active), 0x08100000=Bank2(inactive)
     *   SWAP_BANK=1: 0x08000000=Bank2(active), 0x08100000=Bank1(inactive)
     * The erase bank constant refers to the PHYSICAL bank. */
    uint32_t inactive_base = FW_START_ADDRESS + M1_FLASH_BANK_SIZE;  /* Always 0x08100000 */
    uint32_t inactive_bank;
    if (bl_get_active_bank() == BANK1_ACTIVE)
    {
        inactive_bank = FLASH_BANK_2;  /* Physical bank 2 */
    }
    else
    {
        inactive_bank = FLASH_BANK_1;  /* Physical bank 1 */
    }

    /* Unlock flash */
    HAL_FLASH_Unlock();

    /* Erase required sectors in inactive bank */
    uint16_t n_sectors = (total_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error;

    erase_init.TypeErase  = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks      = inactive_bank;
    erase_init.NbSectors  = 1;

    for (uint16_t i = 0; i < n_sectors; i++)
    {
        m1_wdt_reset();
        erase_init.Sector = i;
        if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK)
        {
            HAL_FLASH_Lock();
            m1_rpc_send_nack(f->seq, RPC_ERR_FLASH_ERROR);
            return;
        }
    }

    /* Initialize state */
    s_fw_update.active        = true;
    s_fw_update.total_size    = total_size;
    s_fw_update.expected_crc32 = expected_crc;
    s_fw_update.bytes_written = 0;
    s_fw_update.flash_addr    = (uint8_t *)inactive_base;

    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle FW_UPDATE_DATA — write a firmware chunk.
 *
 * Payload: [offset:4 LE] [data:1024]
 *
 * Writes directly to flash using bl_flash_if_write().
 */
static void rpc_handle_fw_update_data(const S_RPC_Frame *f)
{
    if (!s_fw_update.active)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_BUSY);
        return;
    }

    if (f->len < 5)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    uint32_t offset = (uint32_t)f->payload[0] |
                      ((uint32_t)f->payload[1] << 8) |
                      ((uint32_t)f->payload[2] << 16) |
                      ((uint32_t)f->payload[3] << 24);

    uint16_t data_len = f->len - 4;
    uint8_t *write_addr = s_fw_update.flash_addr + offset;

    m1_wdt_reset();

    /* Write using HAL — STM32H5 requires 16-byte aligned writes.
     * bl_flash_if_write handles alignment internally. */
    if (bl_flash_if_write((uint8_t *)&f->payload[4], write_addr, data_len) != BL_CODE_OK)
    {
        HAL_FLASH_Lock();
        s_fw_update.active = false;
        m1_rpc_send_nack(f->seq, RPC_ERR_FLASH_ERROR);
        return;
    }

    s_fw_update.bytes_written += data_len;
    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle FW_UPDATE_FINISH — finalize firmware write and verify CRC.
 */
static void rpc_handle_fw_update_finish(const S_RPC_Frame *f)
{
    if (!s_fw_update.active)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_BUSY);
        return;
    }

    HAL_FLASH_Lock();
    s_fw_update.active = false;

    /* Verify the written firmware using the CRC extension block embedded
     * in the binary itself.  Only NACK if CRC extension IS present but
     * the CRC doesn't match (indicates write corruption).
     * Firmware without CRC extension (plain .bin) is allowed through
     * for backward compatibility. */
    uint32_t inactive_base = FW_START_ADDRESS + M1_FLASH_BANK_SIZE;
    uint32_t crc_ext_offset = FW_CRC_EXT_BASE - FW_START_ADDRESS;
    const S_M1_FW_CRC_EXT_t *crc_ext =
        (const S_M1_FW_CRC_EXT_t *)(inactive_base + crc_ext_offset);

    if (crc_ext->magic == FW_CRC_EXT_MAGIC_VALUE)
    {
        /* CRC extension present — verify it */
        m1_wdt_reset();
        if (!bl_verify_bank_crc(inactive_base))
        {
            M1_LOG_E(M1_LOGDB_TAG, "FW update: post-write CRC verification FAILED\r\n");
            m1_rpc_send_nack(f->seq, RPC_ERR_CRC_MISMATCH);
            return;
        }
        M1_LOG_I(M1_LOGDB_TAG, "FW update: CRC verified OK\r\n");
    }
    else
    {
        M1_LOG_I(M1_LOGDB_TAG, "FW update: no CRC extension, skipping verification\r\n");
    }

    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle FW_BANK_SWAP — swap active flash bank and reboot.
 */
static void rpc_handle_fw_bank_swap(const S_RPC_Frame *f)
{
    /* Refuse to swap if the inactive bank has no valid firmware */
    if (!bl_is_inactive_bank_valid()) {
        M1_LOG_E(M1_LOGDB_TAG, "Bank swap refused: inactive bank is empty or invalid\r\n");
        m1_rpc_send_nack(f->seq, RPC_ERR_BANK_EMPTY);
        return;
    }

    /* Verify CRC of the inactive bank before allowing swap.
     * Only refuse if CRC extension IS present but doesn't match.
     * Banks without CRC extension (legacy firmware) are allowed through. */
    {
        uint32_t inactive_base = FW_START_ADDRESS + M1_FLASH_BANK_SIZE;
        uint32_t crc_ext_offset = FW_CRC_EXT_BASE - FW_START_ADDRESS;
        const S_M1_FW_CRC_EXT_t *crc_ext =
            (const S_M1_FW_CRC_EXT_t *)(inactive_base + crc_ext_offset);

        if (crc_ext->magic == FW_CRC_EXT_MAGIC_VALUE) {
            m1_wdt_reset();
            if (!bl_verify_bank_crc(inactive_base)) {
                M1_LOG_E(M1_LOGDB_TAG, "Bank swap refused: inactive bank CRC mismatch\r\n");
                m1_rpc_send_nack(f->seq, RPC_ERR_CRC_MISMATCH);
                return;
            }
        }
    }

    m1_rpc_send_ack(f->seq);
    osDelay(100);  /* Allow ACK transmission */

    /* Clean shutdown before bank swap — same as reboot handler.
     * m1_system_GPIO_init() drives ESP32_EN LOW and puts peripherals
     * in known states.  Without this, ESP32 flashing after bank swap
     * fails because the ESP32 is left in an indeterminate state. */
    m1_system_GPIO_init();

    bl_swap_banks();
    /* bl_swap_banks triggers a system reset — we won't return here.
     * If bl_swap_banks() returns false (flash error), the device stays
     * connected and qMonstatek will notice it didn't disconnect. */
}


/**
 * @brief  Handle FW_DFU_ENTER — enter DFU bootloader mode.
 */
static void rpc_handle_fw_dfu_enter(const S_RPC_Frame *f)
{
    m1_rpc_send_ack(f->seq);
    osDelay(100);  /* Allow ACK transmission */
    bl_jump_to_dfu();
    /* bl_jump_to_dfu does not return */
}


/*============================================================================*/
/*                   E S P 3 2   C O M M A N D S                              */
/*============================================================================*/

/**
 * @brief  Handle ESP_INFO — return ESP32 status information.
 */
static void rpc_handle_esp_info(const S_RPC_Frame *f)
{
    uint8_t resp[36];
    memset(resp, 0, sizeof(resp));

    /* First byte: ESP32 ready status */
    resp[0] = m1_esp32_get_init_status();

    /* Bytes 1-32: version string */
    strncpy((char *)&resp[1],
            resp[0] ? "AT ready" : "offline",
            32);

    m1_rpc_send_frame(RPC_CMD_ESP_INFO_RESP, f->seq, resp, 33);
}


/**
 * @brief  Handle ESP_UPDATE_START — connect to ESP32 and prepare for direct flash.
 *
 * Payload: [total_size:4 LE] [start_addr:4 LE]
 *
 * DEFERRED to RPC task — connect_to_target() and esp_loader_flash_start()
 * are slow (erase can take 5-15 seconds) and would block the USB CDC task.
 */
static void rpc_handle_esp_update_start(const S_RPC_Frame *f)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    esp_loader_error_t err;

    if (f->len < 8)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    /* Abort any previous ESP update */
    if (s_esp_update.active)
    {
        s_esp_update.active = false;
    }

    s_esp_update.total_size = (uint32_t)f->payload[0] |
                              ((uint32_t)f->payload[1] << 8) |
                              ((uint32_t)f->payload[2] << 16) |
                              ((uint32_t)f->payload[3] << 24);

    s_esp_update.start_addr = (uint32_t)f->payload[4] |
                              ((uint32_t)f->payload[5] << 8) |
                              ((uint32_t)f->payload[6] << 16) |
                              ((uint32_t)f->payload[7] << 24);

    /* Validate size + address to prevent flashing the wrong image type.
     * ESP32-C6 has 4MB flash. Factory images start at 0x0, app-only at
     * a non-zero offset. Reject anything that overflows flash bounds. */
    #define ESP32_FLASH_SIZE  (4U * 1024U * 1024U)  /* 4 MB */
    if (s_esp_update.total_size == 0 ||
        s_esp_update.total_size > ESP32_FLASH_SIZE ||
        s_esp_update.start_addr >= ESP32_FLASH_SIZE ||
        (s_esp_update.start_addr + s_esp_update.total_size) > ESP32_FLASH_SIZE)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    /* Init esp-serial-flasher port */
    loader_stm32_config_t config = {
        .huart      = &huart_esp,
        .port_io0   = BUTTON_RIGHT_GPIO_Port,
        .pin_num_io0 = BUTTON_RIGHT_Pin,
        .port_rst   = ESP32_EN_GPIO_Port,
        .pin_num_rst = ESP32_EN_Pin,
    };
    loader_port_stm32_init(&config);
    esp32_UART_init();

    /* Reconfigure BUTTON_RIGHT as output for ESP32 boot-mode pin (IO9) */
    GPIO_InitStruct.Pin   = BUTTON_RIGHT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUTTON_RIGHT_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BUTTON_RIGHT_GPIO_Port, BUTTON_RIGHT_Pin, GPIO_PIN_SET);

    /* Disable ESP32 SPI handshake/dataready EXTI interrupts.
     * The ESP32 is about to enter ROM bootloader mode where SPI is inactive.
     * Leaving these armed causes spurious ISRs that delay UART4 reception. */
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);   /* DataReady */
    HAL_NVIC_DisableIRQ(EXTI7_IRQn);   /* Handshake */

    /* Raise UART4 interrupt priority above all other priority-5 ISRs
     * (IR timers, sub-GHz, debug UART, SD card, keypad EXTIs, etc.)
     * so that no same-priority ISR can delay UART4 byte reception.
     * Priority 4 is the same level as LFRFID's TIM5 — nothing else
     * is higher except hard faults. */
    HAL_NVIC_SetPriority(UART4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1, 0);

    /* Reset overrun counter for this flash session */
    extern volatile uint32_t esp32_uart4_overrun_count;
    esp32_uart4_overrun_count = 0;

    /* Flush stale data and wait for ESP32 to settle.
     * After a failed flash, the ESP32 may be boot-looping with corrupted
     * firmware, spewing garbage on the UART. Flush the ring buffer, wait
     * for any in-flight bytes to arrive, then flush again. */
    m1_ringbuffer_reset(&esp32_rb_hdl);
    HAL_Delay(50);
    m1_ringbuffer_reset(&esp32_rb_hdl);

    /* Suspend software WDT task reporting for the entire ESP32 flash session.
     * The flash process blocks the CDC task for ~10ms per chunk; the WDT
     * handler at the same priority may not get enough run-time to check in,
     * and the system_periodic_task report can drift out of range.
     * The hardware IWDG is still fed via m1_wdt_reset() in loader_port_delay_ms
     * so the device is still protected against true hangs. */
    m1_wdt_suspend_task(M1_REPORT_ID_BUTTONS_HANDLER_TASK);

    /* Suspend screen streaming during ESP flash to eliminate USB TX
     * mutex contention between screen frames and flash ACK responses.
     * This reduces USB interrupt load and prevents timing jitter. */
    s_esp_update.screen_was_streaming = rpc_screen_stream.active;
    if (rpc_screen_stream.active)
        rpc_screen_stream.active = false;

    /* Connect to ESP32 ROM bootloader.
     * Use 230400 for RPC path — UART4 RX is byte-by-byte ISR with no DMA,
     * and concurrent USB interrupts (10-50µs) can cause overrun at higher
     * rates. At 230400, each byte has a 43µs window — safe with USB activity.
     * With FIFO enabled (8-byte buffer), overrun tolerance is ~347µs.
     * 460800+ caused MD5 mismatches from silent data corruption. */
    m1_wdt_reset();
    err = connect_to_target(230400);
    if (err != ESP_LOADER_SUCCESS)
    {
        /* Restore GPIO and clean up */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(BUTTON_RIGHT_GPIO_Port, &GPIO_InitStruct);
        esp32_UART_deinit();
        HAL_NVIC_SetPriority(UART4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        HAL_NVIC_EnableIRQ(EXTI7_IRQn);
        m1_wdt_resume_task(M1_REPORT_ID_BUTTONS_HANDLER_TASK);
        if (s_esp_update.screen_was_streaming)
            rpc_screen_stream.active = true;
        rpc_send_nack_sub(f->seq, RPC_ERR_ESP_FLASH, RPC_ESP_SUB_CONNECT);
        return;
    }

    /* Erase flash — this is the slow part (5-15 seconds).
     * Feed watchdog, then start the erase. The esp_loader_flash_start
     * implementation handles internal timeouts and we run in the RPC task
     * which wakes every 100ms, so no starvation. */
    m1_wdt_reset();
    err = esp_loader_flash_start(s_esp_update.start_addr,
                                  s_esp_update.total_size,
                                  1024);
    if (err != ESP_LOADER_SUCCESS)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(BUTTON_RIGHT_GPIO_Port, &GPIO_InitStruct);
        esp32_UART_deinit();
        HAL_NVIC_SetPriority(UART4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        HAL_NVIC_EnableIRQ(EXTI7_IRQn);
        m1_wdt_resume_task(M1_REPORT_ID_BUTTONS_HANDLER_TASK);
        if (s_esp_update.screen_was_streaming)
            rpc_screen_stream.active = true;
        rpc_send_nack_sub(f->seq, RPC_ERR_ESP_FLASH, RPC_ESP_SUB_ERASE);
        return;
    }

    s_esp_update.bytes_written = 0;
    s_esp_update.active = true;

    m1_wdt_reset();
    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle ESP_UPDATE_DATA — flash ESP32 firmware chunk directly.
 *
 * Payload: [offset:4 LE] [data:N]
 *
 * Runs inline (not deferred) — esp_loader_flash_write at 921600 baud
 * completes in ~1ms per 1024-byte chunk, fast enough for the CDC task.
 */
static void rpc_handle_esp_update_data(const S_RPC_Frame *f)
{
    if (!s_esp_update.active)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_BUSY);
        return;
    }

    if (f->len < 5)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    /* offset field (4 bytes) is informational — esp_loader_flash_write
     * writes sequentially from where flash_start left off */
    uint16_t data_len = f->len - 4;

    esp_loader_error_t err = esp_loader_flash_write(
        (void *)&f->payload[4], data_len);
    if (err != ESP_LOADER_SUCCESS)
    {
        s_esp_update.active = false;

        /* Full cleanup — restore GPIO and reset ESP32 so the device
         * doesn't get stuck with BUTTON_RIGHT as output and ESP32
         * in bootloader mode if qMonstatek never sends FINISH. */
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = BUTTON_RIGHT_Pin;
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(BUTTON_RIGHT_GPIO_Port, &gpio);

        esp32_UART_change_baudrate(ESP32_UART_BAUDRATE);
        m1_ringbuffer_reset(&esp32_rb_hdl);
        esp_loader_reset_target();
        HAL_Delay(100);
        esp32_UART_deinit();

        HAL_NVIC_SetPriority(UART4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        HAL_NVIC_EnableIRQ(EXTI7_IRQn);
        m1_wdt_resume_task(M1_REPORT_ID_BUTTONS_HANDLER_TASK);
        if (s_esp_update.screen_was_streaming)
            rpc_screen_stream.active = true;
        rpc_send_nack_sub(f->seq, RPC_ERR_ESP_FLASH, RPC_ESP_SUB_WRITE);
        return;
    }

    s_esp_update.bytes_written += data_len;
    m1_wdt_reset();
    m1_rpc_send_ack(f->seq);
}


/**
 * @brief  Handle ESP_UPDATE_FINISH — verify flash and reset ESP32.
 *
 * DEFERRED to RPC task — esp_loader_flash_verify() computes MD5 over
 * the entire flashed region, which can take a few seconds.
 */
static void rpc_handle_esp_update_finish(const S_RPC_Frame *f)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    esp_loader_error_t err;

    if (!s_esp_update.active)
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_BUSY);
        return;
    }

    s_esp_update.active = false;

    /* Verify flash integrity via MD5 */
    m1_wdt_reset();
#ifdef MD5_ENABLED
    err = esp_loader_flash_verify();
    if (err != ESP_LOADER_SUCCESS && err != ESP_LOADER_ERROR_UNSUPPORTED_FUNC)
    {
        /* Restore GPIO and clean up even on failure */
        GPIO_InitStruct.Pin   = BUTTON_RIGHT_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(BUTTON_RIGHT_GPIO_Port, &GPIO_InitStruct);

        esp32_UART_change_baudrate(ESP32_UART_BAUDRATE);
        m1_ringbuffer_reset(&esp32_rb_hdl);
        esp_loader_reset_target();
        HAL_Delay(100);
        esp32_UART_deinit();

        HAL_NVIC_SetPriority(UART4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        HAL_NVIC_EnableIRQ(EXTI7_IRQn);

        /* Send NACK with MD5 diagnostic payload:
         * [0]    error code (RPC_ERR_ESP_FLASH)
         * [1]    sub-error  (RPC_ESP_SUB_VERIFY)
         * [2-5]  total bytes hashed locally (uint32_t LE)
         * [6-9]  image_size passed to ROM   (uint32_t LE)
         * [10-41] expected MD5 (32 hex chars, local)
         * [42-73] actual MD5   (32 hex chars, ROM)
         * [74-77] UART4 overrun count (uint32_t LE)
         */
        {
            extern volatile uint32_t esp32_uart4_overrun_count;
            uint8_t diag[78];
            uint32_t total_hashed, img_size;
            diag[0] = RPC_ERR_ESP_FLASH;
            diag[1] = RPC_ESP_SUB_VERIFY;

            esp_loader_get_md5_diagnostic(
                &diag[10], &diag[42], &total_hashed, &img_size);

            memcpy(&diag[2], &total_hashed, 4);
            memcpy(&diag[6], &img_size, 4);

            uint32_t overruns = esp32_uart4_overrun_count;
            memcpy(&diag[74], &overruns, 4);

            m1_rpc_send_frame(RPC_CMD_NACK, f->seq, diag, sizeof(diag));
        }
        m1_wdt_resume_task(M1_REPORT_ID_BUTTONS_HANDLER_TASK);
        if (s_esp_update.screen_was_streaming)
            rpc_screen_stream.active = true;
        return;
    }
#endif

    /* Reset ESP32 back to normal boot and restore GPIO */
    esp32_UART_change_baudrate(ESP32_UART_BAUDRATE);
    m1_ringbuffer_reset(&esp32_rb_hdl);
    esp_loader_reset_target();
    HAL_Delay(100);
    esp32_UART_deinit();

    /* Restore BUTTON_RIGHT to input */
    GPIO_InitStruct.Pin   = BUTTON_RIGHT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUTTON_RIGHT_GPIO_Port, &GPIO_InitStruct);

    /* Restore UART4 priority and re-enable ESP32 SPI EXTI interrupts */
    HAL_NVIC_SetPriority(UART4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI7_IRQn);

    m1_wdt_resume_task(M1_REPORT_ID_BUTTONS_HANDLER_TASK);
    if (s_esp_update.screen_was_streaming)
        rpc_screen_stream.active = true;
    m1_rpc_send_ack(f->seq);
}


/*============================================================================*/
/*                 D E B U G  /  C L I   C O M M A N D S                      */
/*============================================================================*/

/**
 * @brief  Handle CLI_EXEC — execute a CLI command and return the output.
 *
 * Payload: null-terminated command string (e.g., "mtest esp32 0")
 * Response: CLI_RESP with the output text.
 */
static void rpc_handle_cli_exec(const S_RPC_Frame *f)
{
    static char cmd_buf[64];
    static char out_buf[RPC_MAX_PAYLOAD];

    if (f->len < 2)  /* At minimum: one char + null */
    {
        m1_rpc_send_nack(f->seq, RPC_ERR_INVALID_PAYLOAD);
        return;
    }

    /* Copy command string (ensure null-terminated) */
    uint16_t copy_len = (f->len < sizeof(cmd_buf)) ? f->len : sizeof(cmd_buf) - 1;
    memcpy(cmd_buf, f->payload, copy_len);
    cmd_buf[copy_len] = '\0';

    /* Strip trailing newline/null */
    uint16_t slen = (uint16_t)strlen(cmd_buf);
    while (slen > 0 && (cmd_buf[slen-1] == '\n' || cmd_buf[slen-1] == '\r'))
        cmd_buf[--slen] = '\0';

    M1_LOG_I(M1_LOGDB_TAG, "CLI_EXEC: '%s'\r\n", cmd_buf);

    /* Enable capture: intercepts M1_LOG_N / printf output during command
     * execution so we can return it via RPC instead of only to UART. */
    m1_logdb_capture_start(out_buf, sizeof(out_buf));

    /* Run through FreeRTOS CLI processor */
    BaseType_t more;
    do {
        char segment[configCOMMAND_INT_MAX_OUTPUT_SIZE];
        segment[0] = '\0';
        more = FreeRTOS_CLIProcessCommand(cmd_buf, segment,
                                          configCOMMAND_INT_MAX_OUTPUT_SIZE);
        /* FreeRTOS CLI output (pcWriteBuffer) bypasses the _write() capture
         * hook.  Pipe it through printf so it gets captured alongside any
         * M1_LOG_N output the command produced. */
        if (segment[0] != '\0') {
            printf("%s", segment);
            fflush(stdout);
        }
    } while (more != pdFALSE);

    /* Flush any remaining buffered printf output into the capture buffer */
    fflush(stdout);

    /* Stop capture and get the total captured length */
    uint16_t out_len = m1_logdb_capture_stop();

    if (out_len == 0)
    {
        const char *ok_msg = "OK\r\n";
        uint16_t ok_len = (uint16_t)strlen(ok_msg);
        memcpy(out_buf, ok_msg, ok_len);
        out_len = ok_len;
    }

    M1_LOG_I(M1_LOGDB_TAG, "CLI_EXEC: captured %u bytes\r\n", out_len);

    m1_rpc_send_frame(RPC_CMD_CLI_RESP, f->seq,
                      (const uint8_t *)out_buf, out_len);
}


/*============================================================================*/
/*  ESP32 UART Snoop — enable UART4, reset ESP32, capture boot output        */
/*============================================================================*/
static void rpc_handle_esp_uart_snoop(const S_RPC_Frame *f)
{
    static char snoop_buf[RPC_MAX_PAYLOAD];
    uint16_t snoop_len = 0;
    uint8_t rx_byte;

    M1_LOG_I(M1_LOGDB_TAG, "ESP_UART_SNOOP: starting\r\n");

    /* 1. Enable ESP32 UART (normally disabled at runtime) */
    esp32_UART_init();

    /* 2. Reset ESP32: power cycle */
    esp32_disable();
    HAL_Delay(100);
    esp32_enable();

    /* 3. Capture UART output for ~8 seconds (80 x 100ms polls) */
    for (int i = 0; i < 80; i++)
    {
        HAL_Delay(100);

        /* Drain ring buffer */
        while (m1_ringbuffer_read(&esp32_rb_hdl, &rx_byte, 1) == 1)
        {
            if (snoop_len < sizeof(snoop_buf) - 1)
            {
                snoop_buf[snoop_len++] = (char)rx_byte;
            }
        }

        /* Kick watchdog so we don't reset during the 8s capture */
        m1_wdt_reset();
    }

    snoop_buf[snoop_len] = '\0';

    M1_LOG_I(M1_LOGDB_TAG, "ESP_UART_SNOOP: captured %u bytes\r\n", snoop_len);

    /* 4. Send captured data back */
    m1_rpc_send_frame(RPC_CMD_ESP_UART_SNOOP_RESP, f->seq,
                      (const uint8_t *)snoop_buf, snoop_len);

    /* 5. Clean up — deinit UART4 so SPI can operate */
    esp32_UART_deinit();
}


/*============================================================================*/
/*                      R P C   T A S K                                       */
/*============================================================================*/

/**
 * @brief  RPC periodic task — handles screen streaming.
 *
 * This task:
 *   1. Waits for notification (screen update or periodic wake)
 *   2. If screen streaming is active and enough time has elapsed,
 *      sends a SCREEN_FRAME to the desktop app.
 *
 * Registered as a FreeRTOS task by m1_rpc_init().
 */
void m1_rpc_task(void *param)
{
    UNUSED(param);

    static uint8_t screen_seq = 0;

    for (;;)
    {
        /* Wait for notification with timeout.
         * Notifications come from:
         *  - m1_rpc_notify_screen_update() for screen frame ready
         *  - rpc_dispatch_frame() for deferred slow commands */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        /* Process deferred command (file delete, mkdir, etc.)
         * These run here instead of the CDC task to avoid blocking
         * the USB endpoint during slow SD card operations. */
        if (s_deferred_pending)
        {
            S_RPC_Frame frame;
            memcpy(&frame, (const void *)&s_deferred_frame, sizeof(S_RPC_Frame));
            s_deferred_pending = false;

            switch (frame.cmd)
            {
            case RPC_CMD_FILE_WRITE_START:  rpc_handle_file_write_start(&frame);  break;
            case RPC_CMD_FILE_WRITE_DATA:   rpc_handle_file_write_data(&frame);   break;
            case RPC_CMD_FILE_WRITE_FINISH: rpc_handle_file_write_finish(&frame); break;
            case RPC_CMD_FILE_DELETE:       rpc_handle_file_delete(&frame);       break;
            case RPC_CMD_FILE_MKDIR:        rpc_handle_file_mkdir(&frame);        break;
            case RPC_CMD_CLI_EXEC:          rpc_handle_cli_exec(&frame);          break;
            case RPC_CMD_ESP_UPDATE_START:  rpc_handle_esp_update_start(&frame);  break;
            case RPC_CMD_ESP_UPDATE_FINISH: rpc_handle_esp_update_finish(&frame); break;
            default: break;
            }
        }

        /* Screen streaming */
        if (rpc_screen_stream.active)
        {
            uint32_t now = xTaskGetTickCount();
            uint32_t elapsed = now - rpc_screen_stream.last_send_tick;

            if (elapsed >= pdMS_TO_TICKS(rpc_screen_stream.interval_ms))
            {
                rpc_send_screen_frame(screen_seq++);
                rpc_screen_stream.last_send_tick = now;
            }
        }
    }
}


/*============================================================================*/
/*                          I N I T                                            */
/*============================================================================*/

/**
 * @brief  Initialize the RPC subsystem.
 *         Call from system init before task scheduler starts.
 */
void m1_rpc_init(void)
{
    /* Parser state */
    s_parse_state = RPC_STATE_IDLE;

    /* TX mutex */
    s_tx_mutex = xSemaphoreCreateMutex();
    assert(s_tx_mutex != NULL);

    /* Screen streaming off by default */
    memset(&rpc_screen_stream, 0, sizeof(rpc_screen_stream));

    /* FW update state */
    memset(&s_fw_update, 0, sizeof(s_fw_update));

    /* ESP32 update state */
    memset(&s_esp_update, 0, sizeof(s_esp_update));

    /* File write state */
    memset(&s_file_write, 0, sizeof(s_file_write));

    /* Create RPC task */
    BaseType_t ret = xTaskCreate(
        m1_rpc_task,
        "m1_rpc_task_n",
        M1_TASK_STACK_SIZE_4096,
        NULL,
        TASK_PRIORITY_ESP32_TASKS,  /* Same priority as ESP32 tasks */
        &s_rpc_task_hdl
    );
    assert(ret == pdPASS);
    assert(s_rpc_task_hdl != NULL);
}

#endif /* M1_APP_RPC_ENABLE */
