/*
 * Command Handler — Responds to ESP32 SPI requests
 *
 * This is M1's "peripheral" role. Each handler gathers data from
 * the M1's existing subsystems and sends it back over SPI.
 *
 * Uses the same HAL drivers as the main m1-firmware:
 *   - u8g2 for framebuffer access
 *   - battery.h for power status
 *   - m1_sdcard.h + FatFS for SD card operations
 *   - m1_fw_update_bl.h for flash write / bank swap
 *   - m1_system.h for device state + button injection
 */

#include "m1_cmd_handler.h"
#include "m1_spi_slave.h"

/* M1 system headers — m1_tasks.h must precede m1_system.h for type defs */
#include "m1_tasks.h"
#include "m1_system.h"
#include "m1_lcd.h"
#include "m1_sdcard.h"
#include "m1_fw_update_bl.h"
#include "m1_watchdog.h"
#include "battery.h"
#include "u8g2.h"
#include "ff.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>
#include <stdio.h>

/* ---------- Constants ---------- */

#define SCREEN_FB_SIZE          1024    /* 128x64 / 8 */
#define FILE_CHUNK_SIZE         1024
#define SDCARD_DRIVE_PATH       "0:/"

/* ---------- Externs from m1-firmware ---------- */

extern u8g2_t m1_u8g2;
extern uint8_t m1_southpaw_mode;
extern S_M1_Device_Status_t m1_device_stat;
extern QueueHandle_t button_events_q_hdl;
extern QueueHandle_t main_q_hdl;

/* ---------- Log ring buffer ---------- */

#define LOG_RING_SIZE  4096

static char     s_log_ring[LOG_RING_SIZE];
static uint16_t s_log_head;
static uint16_t s_log_tail;

/* ---------- Firmware update state ---------- */

typedef struct {
    bool     active;
    uint32_t total_size;
    uint32_t expected_crc32;
    uint32_t bytes_written;
    uint8_t *flash_addr;
} fw_update_state_t;

static fw_update_state_t s_fw_update;

/* ---------- Static buffers (avoid stack overflow) ---------- */

static uint8_t s_screen_fb_copy[SCREEN_FB_SIZE];
static char    s_path_buf[256];
static uint8_t s_resp_buf[M1P_MAX_PAYLOAD_SIZE];
static DIR     s_dir;
static FILINFO s_fno;
static FIL     s_file;

/* ---------- Log ring buffer implementation ---------- */

void cmd_handler_push_log(const char *msg, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        s_log_ring[s_log_head] = msg[i];
        s_log_head = (s_log_head + 1) % LOG_RING_SIZE;
        if (s_log_head == s_log_tail)
            s_log_tail = (s_log_tail + 1) % LOG_RING_SIZE;
    }
    spi_slave_notify_log_available();
}

int cmd_handler_log_pending(void)
{
    return s_log_head != s_log_tail;
}

static uint16_t log_drain(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (s_log_tail != s_log_head && count < max_len) {
        buf[count++] = s_log_ring[s_log_tail];
        s_log_tail = (s_log_tail + 1) % LOG_RING_SIZE;
    }
    if (!cmd_handler_log_pending())
        spi_slave_deassert_data_ready();
    return count;
}

/* ---------- Screen helpers ---------- */

static inline uint8_t reverse_bits(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
    b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
    b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
    return b;
}

static void flip_buffer_180(uint8_t *buf, uint16_t size)
{
    uint16_t half = size / 2;
    for (uint16_t i = 0; i < half; i++) {
        uint16_t j = size - 1 - i;
        uint8_t a = reverse_bits(buf[i]);
        uint8_t b = reverse_bits(buf[j]);
        buf[i] = b;
        buf[j] = a;
    }
}

/* ---------- SD path helper ---------- */

static void build_sd_path(char *out, size_t out_size,
                           const uint8_t *payload, uint16_t len)
{
    char raw_path[256];
    uint16_t copy_len = (len < sizeof(raw_path) - 1) ? len : sizeof(raw_path) - 1;
    memcpy(raw_path, payload, copy_len);
    raw_path[copy_len] = '\0';

    if (raw_path[0] == '0' && raw_path[1] == ':')
        strncpy(out, raw_path, out_size - 1);
    else if (raw_path[0] == '/')
        snprintf(out, out_size, "0:%s", raw_path);
    else
        snprintf(out, out_size, "0:/%s", raw_path);

    out[out_size - 1] = '\0';
}

/* ---------- Button ID mapping ---------- */

/*
 * Protocol button IDs → M1 button IDs
 * Protocol: UP=0, DOWN=1, LEFT=2, RIGHT=3, SELECT=4, BACK=5
 * M1:       OK=0, UP=1,   LEFT=2, RIGHT=3, DOWN=4,   BACK=5
 */
static const uint8_t s_button_map[] = {
    [M1P_BTN_UP]     = 1,  /* BUTTON_UP_KP_ID */
    [M1P_BTN_DOWN]   = 4,  /* BUTTON_DOWN_KP_ID */
    [M1P_BTN_LEFT]   = 2,  /* BUTTON_LEFT_KP_ID */
    [M1P_BTN_RIGHT]  = 3,  /* BUTTON_RIGHT_KP_ID */
    [M1P_BTN_SELECT] = 0,  /* BUTTON_OK_KP_ID */
    [M1P_BTN_BACK]   = 5,  /* BUTTON_BACK_KP_ID */
};

#define NUM_BUTTONS_MAX  6

static void inject_button_event(uint8_t m1_button_id, uint8_t event)
{
    if (m1_button_id >= NUM_BUTTONS_MAX)
        return;

    S_M1_Buttons_Status btn_status;
    memset(&btn_status, 0, sizeof(btn_status));
    btn_status.event[m1_button_id] = (S_M1_Key_Event)event;
    btn_status.timestamp = xTaskGetTickCount();

    xQueueSend(button_events_q_hdl, &btn_status, 0);

    S_M1_Main_Q_t q_msg;
    memset(&q_msg, 0, sizeof(q_msg));
    q_msg.q_evt_type = Q_EVENT_KEYPAD;
    q_msg.q_data.keypad_evt = 1;
    xQueueSend(main_q_hdl, &q_msg, 0);
}

/* ========================================================================== */
/*                        C O M M A N D   H A N D L E R S                     */
/* ========================================================================== */

static void handle_ping(const m1p_header_t *hdr)
{
    spi_slave_send_response(M1P_CMD_PONG, 0, hdr->seq, NULL, 0);
}

static void handle_get_device_info(const m1p_header_t *hdr)
{
    m1p_device_info_t info;
    memset(&info, 0, sizeof(info));

    /* Battery */
    S_M1_Power_Status_t pwr;
    battery_power_status_get(&pwr);
    info.battery_pct = pwr.battery_level;
    info.battery_mv  = (uint16_t)(pwr.battery_voltage * 1000.0f);
    info.charging    = (pwr.stat == 1 || pwr.stat == 2) ? 1 : 0;

    /* SD card */
    info.sd_present  = m1_sd_detected();
    if (info.sd_present) {
        info.sd_total_kb = m1_sdcard_get_total_capacity();
        info.sd_free_kb  = m1_sdcard_get_free_capacity();
    }

    /* Hardware */
    info.hw_revision   = m1_fw_config.ism_band_region;
    info.screen_width  = 128;
    info.screen_height = 64;
    info.screen_bpp    = 1;
    strncpy(info.device_name, "Monstatek M1", sizeof(info.device_name) - 1);

    spi_slave_send_response(M1P_CMD_GET_DEVICE_INFO, 0, hdr->seq,
                            (const uint8_t *)&info, sizeof(info));
}

static void handle_get_fw_info(const m1p_header_t *hdr)
{
    m1p_fw_info_t info;
    memset(&info, 0, sizeof(info));

    info.major        = m1_fw_config.fw_version_major;
    info.minor        = m1_fw_config.fw_version_minor;
    info.patch        = m1_fw_config.fw_version_build;
    info.build_number = m1_fw_config.fw_version_rc;
    snprintf(info.build_date, sizeof(info.build_date), "%s", __DATE__);

    spi_slave_send_response(M1P_CMD_GET_FW_INFO, 0, hdr->seq,
                            (const uint8_t *)&info, sizeof(info));
}

static void handle_get_screen(const m1p_header_t *hdr)
{
    uint8_t *fb = u8g2_GetBufferPtr(&m1_u8g2);

    /* Atomic copy to prevent tearing */
    taskENTER_CRITICAL();
    memcpy(s_screen_fb_copy, fb, SCREEN_FB_SIZE);
    bool need_flip = (m1_southpaw_mode == 0);
    taskEXIT_CRITICAL();

    /* Normalize orientation — when U8G2_R2 (normal mode), raw buffer is
     * rotated 180 from logical view. Flip so ESP32 always gets correct-side-up. */
    if (need_flip)
        flip_buffer_180(s_screen_fb_copy, SCREEN_FB_SIZE);

    spi_slave_send_response(M1P_CMD_SCREEN_DATA, 0, hdr->seq,
                            s_screen_fb_copy, SCREEN_FB_SIZE);
}

static void handle_button_press(const m1p_header_t *hdr,
                                const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(m1p_button_press_t)) {
        spi_slave_send_error(hdr->seq, M1P_ERR_BAD_LENGTH);
        return;
    }

    const m1p_button_press_t *btn = (const m1p_button_press_t *)payload;

    if (btn->button_id > M1P_BTN_BACK) {
        spi_slave_send_error(hdr->seq, M1P_ERR_UNKNOWN_CMD);
        return;
    }

    uint8_t m1_id = s_button_map[btn->button_id];

    /* Map action: 0=press→CLICK, 1=release→IDLE, 2=long_press→LCLICK */
    S_M1_Key_Event event;
    switch (btn->action) {
    case 0:  event = BUTTON_EVENT_CLICK;  break;
    case 2:  event = BUTTON_EVENT_LCLICK; break;
    default: event = BUTTON_EVENT_IDLE;   break;
    }

    inject_button_event(m1_id, event);

    spi_slave_send_response(M1P_CMD_BUTTON_ACK, 0, hdr->seq, NULL, 0);
}

static void handle_file_list(const m1p_header_t *hdr,
                             const uint8_t *payload, uint16_t len)
{
    if (!m1_sd_detected()) {
        spi_slave_send_error(hdr->seq, M1P_ERR_SD_ERROR);
        return;
    }

    if (len > 0)
        build_sd_path(s_path_buf, sizeof(s_path_buf), payload, len);
    else
        strcpy(s_path_buf, SDCARD_DRIVE_PATH);

    FRESULT res = f_opendir(&s_dir, s_path_buf);
    if (res != FR_OK) {
        spi_slave_send_error(hdr->seq, M1P_ERR_FILE_NOT_FOUND);
        return;
    }

    /*
     * Build response payload matching existing RPC wire format:
     *   [path string + null]
     *   repeated: [is_dir:1] [size:4 LE] [date:2 LE] [time:2 LE] [name + null]
     */
    uint16_t resp_len = 0;

    /* Path string (null-terminated) */
    uint16_t path_len = (uint16_t)strlen(s_path_buf);
    memcpy(&s_resp_buf[resp_len], s_path_buf, path_len + 1);
    resp_len += path_len + 1;

    for (;;) {
        res = f_readdir(&s_dir, &s_fno);
        if (res != FR_OK || s_fno.fname[0] == '\0')
            break;

        /* Skip hidden/system files */
        if (s_fno.fattrib & (AM_HID | AM_SYS))
            continue;

        uint16_t name_len = (uint16_t)strlen(s_fno.fname);
        uint16_t entry_size = 1 + 4 + 2 + 2 + name_len + 1;

        if (resp_len + entry_size > M1P_MAX_PAYLOAD_SIZE)
            break;  /* Truncate if full */

        /* is_dir */
        s_resp_buf[resp_len++] = (s_fno.fattrib & AM_DIR) ? 1 : 0;

        /* File size (4 bytes LE) */
        uint32_t fsize = (uint32_t)s_fno.fsize;
        s_resp_buf[resp_len++] = (uint8_t)(fsize & 0xFF);
        s_resp_buf[resp_len++] = (uint8_t)((fsize >> 8) & 0xFF);
        s_resp_buf[resp_len++] = (uint8_t)((fsize >> 16) & 0xFF);
        s_resp_buf[resp_len++] = (uint8_t)((fsize >> 24) & 0xFF);

        /* FAT date (2 bytes LE) */
        s_resp_buf[resp_len++] = (uint8_t)(s_fno.fdate & 0xFF);
        s_resp_buf[resp_len++] = (uint8_t)((s_fno.fdate >> 8) & 0xFF);

        /* FAT time (2 bytes LE) */
        s_resp_buf[resp_len++] = (uint8_t)(s_fno.ftime & 0xFF);
        s_resp_buf[resp_len++] = (uint8_t)((s_fno.ftime >> 8) & 0xFF);

        /* Name (null-terminated) */
        memcpy(&s_resp_buf[resp_len], s_fno.fname, name_len);
        resp_len += name_len;
        s_resp_buf[resp_len++] = '\0';
    }

    f_closedir(&s_dir);

    spi_slave_send_response(M1P_CMD_FILE_LIST_DATA, 0, hdr->seq,
                            s_resp_buf, resp_len);
}

static void handle_file_read(const m1p_header_t *hdr,
                             const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(m1p_file_read_req_t)) {
        spi_slave_send_error(hdr->seq, M1P_ERR_BAD_LENGTH);
        return;
    }

    if (!m1_sd_detected()) {
        spi_slave_send_error(hdr->seq, M1P_ERR_SD_ERROR);
        return;
    }

    const m1p_file_read_req_t *req = (const m1p_file_read_req_t *)payload;

    /* Build proper SD path */
    build_sd_path(s_path_buf, sizeof(s_path_buf),
                  (const uint8_t *)req->path, strlen(req->path));

    FRESULT res = f_open(&s_file, s_path_buf, FA_READ);
    if (res != FR_OK) {
        spi_slave_send_error(hdr->seq, M1P_ERR_FILE_NOT_FOUND);
        return;
    }

    /* Seek to requested offset */
    if (req->offset > 0) {
        res = f_lseek(&s_file, req->offset);
        if (res != FR_OK) {
            f_close(&s_file);
            spi_slave_send_error(hdr->seq, M1P_ERR_SD_ERROR);
            return;
        }
    }

    /* Read data — prepend offset to response */
    uint16_t max_read = req->max_len;
    if (max_read > M1P_MAX_PAYLOAD_SIZE - 4)
        max_read = M1P_MAX_PAYLOAD_SIZE - 4;

    /* [offset:4 LE][data:N] */
    uint32_t offset = req->offset;
    s_resp_buf[0] = (uint8_t)(offset & 0xFF);
    s_resp_buf[1] = (uint8_t)((offset >> 8) & 0xFF);
    s_resp_buf[2] = (uint8_t)((offset >> 16) & 0xFF);
    s_resp_buf[3] = (uint8_t)((offset >> 24) & 0xFF);

    UINT bytes_read;
    res = f_read(&s_file, &s_resp_buf[4], max_read, &bytes_read);
    f_close(&s_file);

    if (res != FR_OK) {
        spi_slave_send_error(hdr->seq, M1P_ERR_SD_ERROR);
        return;
    }

    spi_slave_send_response(M1P_CMD_FILE_DATA, 0, hdr->seq,
                            s_resp_buf, 4 + bytes_read);
}

static void handle_file_write(const m1p_header_t *hdr,
                              const uint8_t *payload, uint16_t len)
{
    /*
     * Payload: [offset:4 LE] [path_len:2 LE] [path:N] [data:M]
     * If offset == 0, create/truncate file. Otherwise append at offset.
     */
    if (len < 7) {  /* 4 offset + 2 path_len + at least 1 byte path */
        spi_slave_send_error(hdr->seq, M1P_ERR_BAD_LENGTH);
        return;
    }

    if (!m1_sd_detected()) {
        spi_slave_send_error(hdr->seq, M1P_ERR_SD_ERROR);
        return;
    }

    uint32_t offset = (uint32_t)payload[0] |
                      ((uint32_t)payload[1] << 8) |
                      ((uint32_t)payload[2] << 16) |
                      ((uint32_t)payload[3] << 24);

    uint16_t path_len = payload[4] | (payload[5] << 8);
    if (path_len > 250 || 6 + path_len > len) {
        spi_slave_send_error(hdr->seq, M1P_ERR_BAD_LENGTH);
        return;
    }

    build_sd_path(s_path_buf, sizeof(s_path_buf), &payload[6], path_len);

    const uint8_t *data = &payload[6 + path_len];
    uint16_t data_len = len - 6 - path_len;

    BYTE mode = (offset == 0) ? (FA_WRITE | FA_CREATE_ALWAYS) : (FA_WRITE | FA_OPEN_ALWAYS);
    FRESULT res = f_open(&s_file, s_path_buf, mode);
    if (res != FR_OK) {
        spi_slave_send_error(hdr->seq, M1P_ERR_SD_ERROR);
        return;
    }

    if (offset > 0)
        f_lseek(&s_file, offset);

    UINT written;
    res = f_write(&s_file, data, data_len, &written);
    f_close(&s_file);

    if (res != FR_OK || written != data_len) {
        spi_slave_send_error(hdr->seq, M1P_ERR_SD_ERROR);
        return;
    }

    spi_slave_send_response(M1P_CMD_FILE_WRITE_ACK, 0, hdr->seq, NULL, 0);
}

static void handle_file_delete(const m1p_header_t *hdr,
                               const uint8_t *payload, uint16_t len)
{
    if (len < 1) {
        spi_slave_send_error(hdr->seq, M1P_ERR_BAD_LENGTH);
        return;
    }

    if (!m1_sd_detected()) {
        spi_slave_send_error(hdr->seq, M1P_ERR_SD_ERROR);
        return;
    }

    build_sd_path(s_path_buf, sizeof(s_path_buf), payload, len);

    FRESULT res = f_unlink(s_path_buf);
    if (res != FR_OK) {
        spi_slave_send_error(hdr->seq, M1P_ERR_FILE_NOT_FOUND);
        return;
    }

    spi_slave_send_response(M1P_CMD_FILE_DELETE_ACK, 0, hdr->seq, NULL, 0);
}

/* ---------- Firmware Update ---------- */

static void handle_fw_update_begin(const m1p_header_t *hdr,
                                   const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(m1p_fw_update_begin_t)) {
        spi_slave_send_error(hdr->seq, M1P_ERR_BAD_LENGTH);
        return;
    }

    const m1p_fw_update_begin_t *begin = (const m1p_fw_update_begin_t *)payload;

    if (begin->total_size > FW_IMAGE_SIZE_MAX) {
        spi_slave_send_error(hdr->seq, M1P_ERR_FW_ERROR);
        return;
    }

    /* Inactive bank is always at 0x08100000 */
    uint32_t inactive_base = FW_START_ADDRESS + M1_FLASH_BANK_SIZE;
    uint32_t inactive_bank;
    if (bl_get_active_bank() == BANK1_ACTIVE)
        inactive_bank = FLASH_BANK_2;
    else
        inactive_bank = FLASH_BANK_1;

    HAL_FLASH_Unlock();

    /* Erase required sectors in inactive bank */
    uint16_t n_sectors = (begin->total_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error;

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks     = inactive_bank;
    erase_init.NbSectors = 1;

    for (uint16_t i = 0; i < n_sectors; i++) {
        m1_wdt_reset();
        erase_init.Sector = i;
        if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
            HAL_FLASH_Lock();
            spi_slave_send_error(hdr->seq, M1P_ERR_FW_ERROR);
            return;
        }
    }

    s_fw_update.active         = true;
    s_fw_update.total_size     = begin->total_size;
    s_fw_update.expected_crc32 = begin->crc32;
    s_fw_update.bytes_written  = 0;
    s_fw_update.flash_addr     = (uint8_t *)inactive_base;

    spi_slave_send_response(M1P_CMD_FW_UPDATE_ACK, 0, hdr->seq, NULL, 0);
}

static void handle_fw_update_data(const m1p_header_t *hdr,
                                  const uint8_t *payload, uint16_t len)
{
    if (!s_fw_update.active) {
        spi_slave_send_error(hdr->seq, M1P_ERR_BUSY);
        return;
    }

    if (len < sizeof(m1p_fw_update_data_t) + 1) {
        spi_slave_send_error(hdr->seq, M1P_ERR_BAD_LENGTH);
        return;
    }

    const m1p_fw_update_data_t *chunk = (const m1p_fw_update_data_t *)payload;
    const uint8_t *data = payload + sizeof(m1p_fw_update_data_t);
    uint16_t data_len = len - sizeof(m1p_fw_update_data_t);
    uint8_t *write_addr = s_fw_update.flash_addr + chunk->offset;

    m1_wdt_reset();

    if (bl_flash_if_write((uint8_t *)data, write_addr, data_len) != 0) {
        HAL_FLASH_Lock();
        s_fw_update.active = false;
        spi_slave_send_error(hdr->seq, M1P_ERR_FW_ERROR);
        return;
    }

    s_fw_update.bytes_written += data_len;

    spi_slave_send_response(M1P_CMD_FW_UPDATE_ACK, 0, hdr->seq, NULL, 0);
}

static void handle_fw_update_end(const m1p_header_t *hdr)
{
    if (!s_fw_update.active) {
        spi_slave_send_error(hdr->seq, M1P_ERR_BUSY);
        return;
    }

    HAL_FLASH_Lock();
    s_fw_update.active = false;

    /* Verify CRC if extension block is present in written firmware */
    uint32_t inactive_base = FW_START_ADDRESS + M1_FLASH_BANK_SIZE;
    if (bl_verify_bank_crc(inactive_base)) {
        spi_slave_send_response(M1P_CMD_FW_UPDATE_ACK, 0, hdr->seq, NULL, 0);
    } else {
        /* CRC extension present but mismatch — write corruption */
        spi_slave_send_error(hdr->seq, M1P_ERR_FW_ERROR);
    }
}

static void handle_reset(const m1p_header_t *hdr)
{
    spi_slave_send_response(M1P_CMD_PONG, 0, hdr->seq, NULL, 0);

    /* Brief delay to let response transmit */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Validate inactive bank and swap */
    if (bl_is_inactive_bank_valid()) {
        bl_swap_banks();  /* Triggers system reset — does not return */
    } else {
        HAL_NVIC_SystemReset();
    }
}

static void handle_get_log(const m1p_header_t *hdr)
{
    uint8_t buf[M1P_MAX_PAYLOAD_SIZE];
    uint16_t len = log_drain(buf, sizeof(buf));

    spi_slave_send_response(M1P_CMD_LOG_DATA, 0, hdr->seq, buf, len);
}

/* ========================================================================== */
/*                           D I S P A T C H                                  */
/* ========================================================================== */

void cmd_handler_init(void)
{
    s_log_head = 0;
    s_log_tail = 0;
    memset(&s_fw_update, 0, sizeof(s_fw_update));
}

void cmd_handler_dispatch(const m1p_header_t *hdr,
                          const uint8_t *payload, uint16_t payload_len)
{
    switch (hdr->cmd) {
    /* System */
    case M1P_CMD_PING:
        handle_ping(hdr);
        break;
    case M1P_CMD_RESET:
        handle_reset(hdr);
        break;
    case M1P_CMD_GET_DEVICE_INFO:
        handle_get_device_info(hdr);
        break;
    case M1P_CMD_GET_FW_INFO:
        handle_get_fw_info(hdr);
        break;

    /* Screen */
    case M1P_CMD_GET_SCREEN:
        handle_get_screen(hdr);
        break;

    /* Input */
    case M1P_CMD_BUTTON_PRESS:
        handle_button_press(hdr, payload, payload_len);
        break;

    /* File system */
    case M1P_CMD_FILE_LIST:
        handle_file_list(hdr, payload, payload_len);
        break;
    case M1P_CMD_FILE_READ:
        handle_file_read(hdr, payload, payload_len);
        break;
    case M1P_CMD_FILE_WRITE:
        handle_file_write(hdr, payload, payload_len);
        break;
    case M1P_CMD_FILE_DELETE:
        handle_file_delete(hdr, payload, payload_len);
        break;

    /* Firmware update */
    case M1P_CMD_FW_UPDATE_BEGIN:
        handle_fw_update_begin(hdr, payload, payload_len);
        break;
    case M1P_CMD_FW_UPDATE_DATA:
        handle_fw_update_data(hdr, payload, payload_len);
        break;
    case M1P_CMD_FW_UPDATE_END:
        handle_fw_update_end(hdr);
        break;

    /* Logs */
    case M1P_CMD_GET_LOG:
        handle_get_log(hdr);
        break;

    default:
        spi_slave_send_error(hdr->seq, M1P_ERR_UNKNOWN_CMD);
        break;
    }
}
