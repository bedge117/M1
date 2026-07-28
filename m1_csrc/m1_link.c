/* See COPYING.txt for license details. */
/*
 * m1_link.c — "M1 Link" full-duplex SPI transport, STM32 master side (bring-up).
 * Implemented against Reference Guides/kb/ref_stm32h5_spi_master.md.
 *
 * NOTE: no D-cache maintenance — the STM32H5 Cortex-M33 has no L1 D-cache, and
 * these buffers live in internal SRAM. (SCB_*DCache_by_Addr do not exist for M33.)
 */
#include "m1_link.h"
#include "m1_esp_rpc.h"
#include "m1_esp32_hal.h"       /* hspi_esp */
#include "main.h"               /* ESP32_HANDSHAKE_*, ESP32_SPI3_NSS_* pin defines */
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "m1_log_debug.h"       /* M1_LOG_I — bring-up diagnostics */
#include <string.h>

#define M1_LOGDB_TAG  "m1link"  /* per-file log tag (matches project convention) */

#define M1L_HS_TIMEOUT_MS   100     /* wait for slave-armed HANDSHAKE */
#define M1L_SPI_TIMEOUT_MS  100     /* one 512B transfer @ ~9.375MHz ~= 0.5ms */

/* 4-byte aligned SRAM buffers (no DMA here, but keeps things tidy). */
static uint8_t s_tx[M1L_MTU] __attribute__((aligned(4)));
static uint8_t s_rx[M1L_MTU] __attribute__((aligned(4)));

/* Serializes whole request/response exchanges (menu ops vs the qMonstatek
 * relay poll both drive the link from different tasks). */
static SemaphoreHandle_t s_link_mutex;

static inline bool handshake_high(void)
{
    return HAL_GPIO_ReadPin(ESP32_HANDSHAKE_GPIO_Port, ESP32_HANDSHAKE_Pin) == GPIO_PIN_SET;
}
static inline void cs_assert(void)   /* active low, software NSS */
{
    HAL_GPIO_WritePin(ESP32_SPI3_NSS_GPIO_Port, ESP32_SPI3_NSS_Pin, GPIO_PIN_RESET);
}
static inline void cs_release(void)
{
    HAL_GPIO_WritePin(ESP32_SPI3_NSS_GPIO_Port, ESP32_SPI3_NSS_Pin, GPIO_PIN_SET);
}

/* Bring-up breadcrumbs: survive an IWDG reset in TAMP backup regs so we can
 * see exactly how far a hanging scan got. Read over SWD after the reset:
 *   mdw 0x44007D20 5  ->  BKP8R=stage BKP9R=hs BKP10R=send_r BKP11R=rc BKP12R=n
 * Stage codes: 0x01 enter req, 0x51 before SPI xfer, 0x52 after SPI xfer,
 *              0x03 in rx-loop, 0xFF req done. */
void m1_link_bc(uint32_t reg_off, uint32_t val)
{
    PWR->DBPCR |= PWR_DBPCR_DBP;
    *(volatile uint32_t *)(0x44007C00u + 0x100u + reg_off * 4u) = val;
}
#define M1L_BC_STAGE(v)  m1_link_bc(8, (v))   /* BKP8R */

void m1_link_init(void)
{
    if (!s_link_mutex) s_link_mutex = xSemaphoreCreateMutex();
    cs_release();
}

int m1_link_transfer(const uint8_t *tx_frame, uint16_t tx_len,
                     uint8_t *rx_frame, uint16_t rx_size)
{
    /* Build the outgoing 512B buffer: caller's frame or an IDLE filler. */
    if (tx_frame && tx_len > 0 && tx_len <= M1L_MTU) {
        memcpy(s_tx, tx_frame, tx_len);
        if (tx_len < M1L_MTU) memset(s_tx + tx_len, 0, M1L_MTU - tx_len);
    } else {
        size_t n = m1esp_build(s_tx, M1L_MTU, M1ESP_IDLE, 0, NULL, 0);
        if (n < M1L_MTU) memset(s_tx + n, 0, M1L_MTU - n);
    }

    /* Gate: never clock while the slave is not armed (HANDSHAKE low).
     * YIELD between polls — a tight busy-spin here starves the FreeRTOS idle
     * hook and the watchdog-feeding path, which (when the ESP never raises
     * HANDSHAKE) triggers an IWDG reset instead of a clean -1 timeout. */
    uint32_t t0 = HAL_GetTick();
    while (!handshake_high()) {
        if ((HAL_GetTick() - t0) > M1L_HS_TIMEOUT_MS) return -1;
        vTaskDelay(1);
    }

    /* One fixed-size full-duplex transaction. */
    cs_assert();
    M1L_BC_STAGE(0x51);   /* breadcrumb: about to clock the SPI transfer */
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi_esp, s_tx, s_rx,
                                                   M1L_MTU, M1L_SPI_TIMEOUT_MS);
    M1L_BC_STAGE(0x52);   /* breadcrumb: SPI transfer returned (st in BKP9R) */
    m1_link_bc(9, (uint32_t)st);
    cs_release();
    if (st != HAL_OK)
    {
        /* SELF-HEAL: a transient SPI fault (OVR / mode-fault / HAL timeout) leaves
         * SPI3's FIFO/packing byte-shifted, so EVERY later fixed-size exchange
         * fails CRC and the ESP reads as permanently "not available" until a full
         * re-init. Abort resets the peripheral state machine + flushes the FIFOs,
         * so the NEXT transaction recovers on its own — no deinit/init needed. */
        HAL_SPI_Abort(&hspi_esp);
        return -2;
    }

    /* Deliver a valid, non-IDLE received frame. */
    m1esp_header_t h;
    const uint8_t *pl;
    uint16_t pll;
    if (m1esp_parse(s_rx, M1L_MTU, &h, &pl, &pll) == 0 && h.msg_type != M1ESP_IDLE) {
        uint16_t flen = M1ESP_HEADER_SIZE + pll + M1ESP_CRC_SIZE;
        uint16_t n = flen > rx_size ? rx_size : flen;
        if (rx_frame) memcpy(rx_frame, s_rx, n);
        return n;
    }
    return 0;   /* ESP had nothing (IDLE) */
}

int m1_link_request(const uint8_t *req_frame, uint16_t req_len,
                    uint8_t *resp, uint16_t resp_size, uint32_t timeout_ms)
{
    m1esp_header_t rq;
    const uint8_t *x;
    uint16_t xl;
    if (m1esp_parse(req_frame, req_len, &rq, &x, &xl) != 0) return -3;

    static uint8_t frame[M1L_MTU];              /* one received transaction */
    static uint8_t reasm[M1ESP_MAX_PAYLOAD];    /* reassembled response payload */
    uint16_t reasm_len = 0;
    bool     reasm_active = false;

    /* First transaction carries the request; the response arrives later. */
    int r = m1_link_transfer(req_frame, req_len, frame, sizeof(frame));
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        if (r > 0) {
            m1esp_header_t rh;
            const uint8_t *y;
            uint16_t yl;
            if (m1esp_parse(frame, r, &rh, &y, &yl) == 0 && rh.msg_id == rq.msg_id) {
                if (rh.msg_type == M1ESP_FRAG) {
                    if ((uint32_t)reasm_len + yl <= sizeof(reasm)) {
                        memcpy(reasm + reasm_len, y, yl);
                        reasm_len += yl;
                    }
                    reasm_active = true;
                }
                else if (rh.msg_type == M1ESP_RESP || rh.msg_type == M1ESP_NAK) {
                    if (reasm_active) {
                        /* Terminating fragment: append + rebuild the whole frame. */
                        if ((uint32_t)reasm_len + yl <= sizeof(reasm)) {
                            memcpy(reasm + reasm_len, y, yl);
                            reasm_len += yl;
                        }
                        size_t fn = m1esp_build(resp, resp_size, rh.msg_type,
                                                rh.msg_id, reasm, reasm_len);
                        return fn ? (int)fn : -4;
                    }
                    /* Single-frame response — copy it out as-is. */
                    uint16_t n = r > resp_size ? resp_size : (uint16_t)r;
                    memcpy(resp, frame, n);
                    return n;
                }
            }
        } else if (r < -1) {
            return r;                           /* hard SPI error */
        }
        if ((HAL_GetTick() - t0) >= timeout_ms) return 0;   /* timeout */
        vTaskDelay(1);
        r = m1_link_transfer(NULL, 0, frame, sizeof(frame));  /* poll with IDLE */
    }
}

int m1_link_request_msg(uint8_t type, uint16_t msg_id,
                        const uint8_t *payload, uint16_t plen,
                        uint8_t *resp, uint16_t resp_size, uint32_t timeout_ms)
{
    M1L_BC_STAGE(0x01);          /* breadcrumb: entered request (pre-mutex) */
    if (s_link_mutex) xSemaphoreTake(s_link_mutex, portMAX_DELAY);
    M1L_BC_STAGE(0x02);          /* breadcrumb: got the link mutex */

    static uint8_t txf[M1L_MTU];
    static uint8_t frame[M1L_MTU];
    static uint8_t reasm[M1ESP_MAX_PAYLOAD];
    uint16_t reasm_len = 0;
    bool     reasm_active = false;
    int rc;

    /* --- Send phase: fragment the request (FRAG frames + terminating). --- */
    uint32_t off = 0;
    while (plen - off > M1L_PAYLOAD_MAX) {
        size_t fn = m1esp_build(txf, sizeof(txf), M1ESP_FRAG, msg_id,
                                payload + off, M1L_PAYLOAD_MAX);
        m1_link_transfer(txf, (uint16_t)fn, frame, sizeof(frame));  /* rx is IDLE here */
        off += M1L_PAYLOAD_MAX;
    }
    size_t fn = m1esp_build(txf, sizeof(txf), type, msg_id,
                            payload + off, (uint16_t)(plen - off));
    M1L_BC_STAGE(0x30);                         /* breadcrumb: about to send request frame */
    int r = m1_link_transfer(txf, (uint16_t)fn, frame, sizeof(frame));
    M1L_BC_STAGE(0x31);                         /* breadcrumb: send transfer returned */
    m1_link_bc(10, (uint32_t)r);                /* BKP10R = send_r */
    m1_link_bc(11, (uint32_t)handshake_high()); /* BKP11R = hs after send */

    /* Bring-up diagnostics: send_r distinguishes the failure mode —
     *   -1 = HANDSHAKE never went high (ESP slave not armed / not clocking)
     *   -2 = HAL SPI error
     *    0 = clocked OK but ESP returned IDLE (didn't answer the request)
     *   >0 = got a frame back immediately */
    M1_LOG_I(M1_LOGDB_TAG, "m1link REQ id=0x%04X plen=%u hs=%d send_r=%d\r\n",
             msg_id, plen, (int)handshake_high(), r);

    /* --- Receive phase: poll + reassemble the matching response. --- */
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        if (r > 0) {
            m1esp_header_t rh;
            const uint8_t *y;
            uint16_t yl;
            if (m1esp_parse(frame, r, &rh, &y, &yl) == 0 && rh.msg_id == msg_id) {
                if (rh.msg_type == M1ESP_FRAG) {
                    if ((uint32_t)reasm_len + yl <= sizeof(reasm)) {
                        memcpy(reasm + reasm_len, y, yl);
                        reasm_len += yl;
                    }
                    reasm_active = true;
                }
                else if (rh.msg_type == M1ESP_RESP || rh.msg_type == M1ESP_NAK) {
                    if (reasm_active) {
                        if ((uint32_t)reasm_len + yl <= sizeof(reasm)) {
                            memcpy(reasm + reasm_len, y, yl);
                            reasm_len += yl;
                        }
                        size_t rn = m1esp_build(resp, resp_size, rh.msg_type,
                                                rh.msg_id, reasm, reasm_len);
                        rc = rn ? (int)rn : -4;
                        goto out;
                    }
                    uint16_t n = r > resp_size ? resp_size : (uint16_t)r;
                    memcpy(resp, frame, n);
                    rc = n;
                    goto out;
                }
            }
        } else if (r < -1) { rc = r; goto out; }
        if ((HAL_GetTick() - t0) >= timeout_ms) { rc = 0; goto out; }
        M1L_BC_STAGE(0x40);   /* breadcrumb: in rx poll loop */
        vTaskDelay(1);
        r = m1_link_transfer(NULL, 0, frame, sizeof(frame));
    }
out:
    M1L_BC_STAGE(0xFF);           /* breadcrumb: request finished cleanly */
    m1_link_bc(12, (uint32_t)rc); /* BKP12R = rc */
    M1_LOG_I(M1_LOGDB_TAG, "m1link DONE id=0x%04X rc=%d\r\n", msg_id, rc);
    if (s_link_mutex) xSemaphoreGive(s_link_mutex);
    return rc;
}
