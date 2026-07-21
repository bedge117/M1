/* See COPYING.txt for license details. */
/*
 * m1_link.h — "M1 Link" reliable full-duplex SPI transport (Option B), STM32 master.
 *
 * The M1 (STM32H573) is the SPI master; the ESP32-C6 is the slave. Every
 * transaction is a fixed M1L_MTU (512-byte) full-duplex exchange carrying one
 * m1_esp_rpc frame each way (IDLE filler when a side has nothing). The master
 * MUST wait for the ESP HANDSHAKE line (PD7) high before clocking — that is the
 * reliability gate (the slave raises it only once its DMA is armed).
 *
 * Bring-up uses polling HAL_SPI_TransmitReceive on the existing hspi_esp handle
 * (Mode 1, software NSS on PB10, ~9.375 MHz) — no DMA/cache concerns. Reuses the
 * SPI3 init already done by m1_esp32_hal.c.
 *
 * Refs: Reference Guides/kb/ref_stm32h5_spi_master.md, ref_esp_hosted_spi_protocol.md
 */
#ifndef M1_LINK_H_
#define M1_LINK_H_

#include <stdint.h>
#include <stdbool.h>

/* One fixed-size full-duplex exchange. Sends tx_frame (padded to M1L_MTU with a
 * zero tail; sends an IDLE frame if tx_frame is NULL/empty) while capturing the
 * ESP's frame. Returns: >0 = length of a received non-IDLE frame copied into
 * rx_frame; 0 = the ESP sent IDLE (nothing); <0 = error (-1 handshake timeout,
 * -2 SPI error). */
int m1_link_transfer(const uint8_t *tx_frame, uint16_t tx_len,
                     uint8_t *rx_frame, uint16_t rx_size);

/* Send a request frame and poll (IDLE exchanges) until the matching RESP/NAK
 * (same msg_id) arrives or timeout_ms elapses. Returns response length (>0),
 * 0 on timeout, <0 on error. */
int m1_link_request(const uint8_t *req_frame, uint16_t req_len,
                    uint8_t *resp, uint16_t resp_size, uint32_t timeout_ms);

/* Logical request/response with automatic fragmentation of a large request
 * and reassembly of a large response. Mutex-serialized. Preferred entry point. */
int m1_link_request_msg(uint8_t type, uint16_t msg_id,
                        const uint8_t *payload, uint16_t plen,
                        uint8_t *resp, uint16_t resp_size, uint32_t timeout_ms);

/* Bring-up breadcrumb: write `val` to TAMP BKP<reg_off>R (survives IWDG reset).
 * Read over SWD after a hang:  mdw 0x44007D20 6  (BKP8R..BKP13R). */
void m1_link_bc(uint32_t reg_off, uint32_t val);

/* Idle-deassert CS. Call once after the ESP SPI (hspi_esp) is initialized. */
void m1_link_init(void);

#endif /* M1_LINK_H_ */
