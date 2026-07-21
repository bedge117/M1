/*
 * Command Handler — Dispatches SPI requests from ESP32
 *
 * Each command handler gathers the requested data from M1's
 * subsystems (display, battery, SD, flash) and sends the
 * response back via spi_slave_send_response().
 *
 * Uses M1 drivers directly:
 *   - u8g2 framebuffer (m1_lcd.h)
 *   - Battery via BQ25896/BQ27421 (battery.h)
 *   - SD card via FatFS (m1_sdcard.h, ff.h)
 *   - Flash write/bank swap (m1_fw_update_bl.h)
 *   - Button injection (m1_system.h queues)
 */

#ifndef M1_CMD_HANDLER_H
#define M1_CMD_HANDLER_H

#include "m1_protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize command handler subsystem.
 * Call after all M1 peripherals are initialized.
 */
void cmd_handler_init(void);

/*
 * Dispatch a received SPI command.
 * Called by spi_slave_process() after parsing the incoming frame.
 */
void cmd_handler_dispatch(const m1p_header_t *hdr,
                          const uint8_t *payload, uint16_t payload_len);

/*
 * Push a log message into the ring buffer.
 * Call from anywhere in M1 firmware. ESP32 fetches via CMD_GET_LOG
 * when DATA_READY is asserted.
 */
void cmd_handler_push_log(const char *msg, uint16_t len);

/*
 * Check if there are pending log messages.
 */
int cmd_handler_log_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* M1_CMD_HANDLER_H */
