/*
 * SPI Slave — M1 side of the ESP32-M1 link
 *
 * Hardware: STM32H573, SPI3
 *   SCLK = PB3  (AF6_SPI3, input from ESP32)
 *   MOSI = PB2  (AF7_SPI3, input from ESP32)
 *   MISO = PB4  (AF6_SPI3, output to ESP32)
 *   CS   = PB10 (NSS hard input)
 *   DATA_READY = PD7 (output to ESP32, active high)
 *
 * M1 is SPI slave. ESP32 drives clock and initiates all transactions.
 * M1 asserts DATA_READY (PD7) when it has unsolicited data.
 */

#ifndef M1_SPI_SLAVE_H
#define M1_SPI_SLAVE_H

#include "m1_protocol.h"
#include "stm32h5xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware pin definitions — matches M1 PCB */
#define SPI_SLAVE_PERIPH        SPI3
#define SPI_SLAVE_CLK_ENABLE()  __HAL_RCC_SPI3_CLK_ENABLE()
#define SPI_SLAVE_GPIO_CLK()    do { __HAL_RCC_GPIOB_CLK_ENABLE(); \
                                     __HAL_RCC_GPIOD_CLK_ENABLE(); } while(0)

#define SPI_SLAVE_SCLK_PIN      GPIO_PIN_3
#define SPI_SLAVE_SCLK_PORT     GPIOB
#define SPI_SLAVE_SCLK_AF       GPIO_AF6_SPI3

#define SPI_SLAVE_MOSI_PIN      GPIO_PIN_2
#define SPI_SLAVE_MOSI_PORT     GPIOB
#define SPI_SLAVE_MOSI_AF       GPIO_AF7_SPI3

#define SPI_SLAVE_MISO_PIN      GPIO_PIN_4
#define SPI_SLAVE_MISO_PORT     GPIOB
#define SPI_SLAVE_MISO_AF       GPIO_AF6_SPI3

#define SPI_SLAVE_NSS_PIN       GPIO_PIN_10
#define SPI_SLAVE_NSS_PORT      GPIOB

#define SPI_SLAVE_DR_PIN        GPIO_PIN_7    /* DATA_READY output */
#define SPI_SLAVE_DR_PORT       GPIOD

/* Initialize SPI3 as slave + DMA + DATA_READY GPIO */
int spi_slave_init(void);

/* Assert DATA_READY to signal ESP32 we have pending data */
void spi_slave_assert_data_ready(void);

/* Deassert DATA_READY */
void spi_slave_deassert_data_ready(void);

/* Check if a pending log message should trigger DATA_READY */
void spi_slave_notify_log_available(void);

/*
 * Send a response frame back to ESP32.
 * Called from command handlers after processing a request.
 * Loads the TX DMA buffer so the next SPI clock cycle sends this data.
 */
int spi_slave_send_response(uint8_t cmd, uint8_t flags, uint16_t seq,
                            const uint8_t *payload, uint16_t payload_len);

/* Send an error response */
int spi_slave_send_error(uint16_t seq, m1p_error_t error);

/*
 * Main processing loop — call from the SPI task or main loop.
 * Checks for SPI RX complete, parses frame, dispatches to command handler.
 */
void spi_slave_process(void);

/*
 * Call from HAL_SPI_TxRxCpltCallback when SPI3 transfer completes.
 */
void spi_slave_txrx_complete_callback(SPI_HandleTypeDef *hspi);

/*
 * Get the SPI handle (for use in HAL callbacks).
 */
SPI_HandleTypeDef *spi_slave_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* M1_SPI_SLAVE_H */
