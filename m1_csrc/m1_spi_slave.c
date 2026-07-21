/*
 * SPI Slave — M1 side, responds to ESP32 master
 *
 * Uses STM32H573 SPI3 in slave mode with DMA.
 * ESP32 drives clock. Flow:
 *   1. ESP32 clocks in request frame → SPI RX DMA complete interrupt fires
 *   2. spi_slave_process() parses request, dispatches to command_handler
 *   3. Command handler calls spi_slave_send_response() to load TX buffer
 *   4. ESP32 clocks again → gets our response from TX DMA
 *
 * Pin mapping (matches M1 PCB):
 *   SCLK = PB3 (AF6_SPI3)     ← ESP32 GPIO7
 *   MOSI = PB2 (AF7_SPI3)     ← ESP32 GPIO12
 *   MISO = PB4 (AF6_SPI3)     → ESP32 GPIO13
 *   NSS  = PB10               ← ESP32 GPIO15
 *   DATA_READY = PD7           → ESP32 GPIO14
 */

#include "m1_spi_slave.h"
#include "m1_cmd_handler.h"
#include <string.h>

/* SPI handle */
static SPI_HandleTypeDef s_hspi;

/* DMA handles */
static DMA_HandleTypeDef s_hdma_rx;
static DMA_HandleTypeDef s_hdma_tx;

/* Buffers — 32-byte aligned for D-Cache maintenance */
static uint8_t s_rx_buf[M1P_MAX_FRAME_SIZE] __attribute__((aligned(32)));
static uint8_t s_tx_buf[M1P_MAX_FRAME_SIZE] __attribute__((aligned(32)));

static volatile int s_rx_complete;

/* ---------- GPIO init ---------- */

static void gpio_init(void)
{
    SPI_SLAVE_GPIO_CLK();

    GPIO_InitTypeDef gpio = {0};

    /* SCLK — PB3 AF6 (input, ESP32 drives) */
    gpio.Pin       = SPI_SLAVE_SCLK_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = SPI_SLAVE_SCLK_AF;
    HAL_GPIO_Init(SPI_SLAVE_SCLK_PORT, &gpio);

    /* MOSI — PB2 AF7 (input, ESP32 drives) */
    gpio.Pin       = SPI_SLAVE_MOSI_PIN;
    gpio.Alternate = SPI_SLAVE_MOSI_AF;
    HAL_GPIO_Init(SPI_SLAVE_MOSI_PORT, &gpio);

    /* MISO — PB4 AF6 (output, M1 drives) */
    gpio.Pin       = SPI_SLAVE_MISO_PIN;
    gpio.Alternate = SPI_SLAVE_MISO_AF;
    HAL_GPIO_Init(SPI_SLAVE_MISO_PORT, &gpio);

    /* NSS — PB10 (input, ESP32 drives chip select) */
    gpio.Pin       = SPI_SLAVE_NSS_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Alternate = GPIO_AF6_SPI3;  /* SPI3_NSS alternate function */
    HAL_GPIO_Init(SPI_SLAVE_NSS_PORT, &gpio);

    /* DATA_READY — PD7 (output, M1 signals ESP32) */
    gpio.Pin   = SPI_SLAVE_DR_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI_SLAVE_DR_PORT, &gpio);

    HAL_GPIO_WritePin(SPI_SLAVE_DR_PORT, SPI_SLAVE_DR_PIN, GPIO_PIN_RESET);
}

/* ---------- DMA init ---------- */

static void dma_init(void)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    /* RX DMA — GPDMA1 Channel 4 */
    s_hdma_rx.Instance                 = GPDMA1_Channel4;
    s_hdma_rx.Init.Request             = GPDMA1_REQUEST_SPI3_RX;
    s_hdma_rx.Init.BlkHWRequest        = DMA_BREQ_SINGLE_BURST;
    s_hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_rx.Init.SrcInc              = DMA_SINC_FIXED;
    s_hdma_rx.Init.DestInc             = DMA_DINC_INCREMENTED;
    s_hdma_rx.Init.SrcDataWidth        = DMA_SRC_DATAWIDTH_BYTE;
    s_hdma_rx.Init.DestDataWidth       = DMA_DEST_DATAWIDTH_BYTE;
    s_hdma_rx.Init.Priority            = DMA_LOW_PRIORITY_HIGH_WEIGHT;
    s_hdma_rx.Init.SrcBurstLength      = 1;
    s_hdma_rx.Init.DestBurstLength     = 1;
    s_hdma_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    s_hdma_rx.Init.TransferEventMode   = DMA_TCEM_BLOCK_TRANSFER;
    s_hdma_rx.Init.Mode                = DMA_NORMAL;
    HAL_DMA_Init(&s_hdma_rx);
    __HAL_LINKDMA(&s_hspi, hdmarx, s_hdma_rx);

    /* TX DMA — GPDMA1 Channel 5 */
    s_hdma_tx.Instance                 = GPDMA1_Channel5;
    s_hdma_tx.Init.Request             = GPDMA1_REQUEST_SPI3_TX;
    s_hdma_tx.Init.BlkHWRequest        = DMA_BREQ_SINGLE_BURST;
    s_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    s_hdma_tx.Init.SrcInc              = DMA_SINC_INCREMENTED;
    s_hdma_tx.Init.DestInc             = DMA_DINC_FIXED;
    s_hdma_tx.Init.SrcDataWidth        = DMA_SRC_DATAWIDTH_BYTE;
    s_hdma_tx.Init.DestDataWidth       = DMA_DEST_DATAWIDTH_BYTE;
    s_hdma_tx.Init.Priority            = DMA_LOW_PRIORITY_HIGH_WEIGHT;
    s_hdma_tx.Init.SrcBurstLength      = 1;
    s_hdma_tx.Init.DestBurstLength     = 1;
    s_hdma_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    s_hdma_tx.Init.TransferEventMode   = DMA_TCEM_BLOCK_TRANSFER;
    s_hdma_tx.Init.Mode                = DMA_NORMAL;
    HAL_DMA_Init(&s_hdma_tx);
    __HAL_LINKDMA(&s_hspi, hdmatx, s_hdma_tx);

    /* DMA interrupts */
    HAL_NVIC_SetPriority(GPDMA1_Channel4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);
}

/* ---------- SPI3 slave init ---------- */

static void spi3_init(void)
{
    SPI_SLAVE_CLK_ENABLE();

    s_hspi.Instance               = SPI_SLAVE_PERIPH;
    s_hspi.Init.Mode              = SPI_MODE_SLAVE;
    s_hspi.Init.Direction         = SPI_DIRECTION_2LINES;
    s_hspi.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi.Init.CLKPolarity       = SPI_POLARITY_LOW;       /* CPOL=0 */
    s_hspi.Init.CLKPhase          = SPI_PHASE_2EDGE;        /* CPHA=1 — Mode 1, matches ESP32 */
    s_hspi.Init.NSS               = SPI_NSS_HARD_INPUT;     /* ESP32 drives CS */
    s_hspi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    s_hspi.Init.CRCPolynomial     = 0x7;
    s_hspi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;  /* Slave — no pulse */
    s_hspi.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    s_hspi.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    s_hspi.Init.MasterSSIdleness  = SPI_MASTER_SS_IDLENESS_00CYCLE;
    s_hspi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    s_hspi.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    s_hspi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    s_hspi.Init.IOSwap            = SPI_IO_SWAP_DISABLE;
    s_hspi.Init.ReadyMasterManagement   = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
    s_hspi.Init.ReadyPolarity     = SPI_RDY_POLARITY_HIGH;

    HAL_SPI_Init(&s_hspi);

    /* SPI interrupt for error handling */
    HAL_NVIC_SetPriority(SPI3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

/* ---------- Start DMA receive ---------- */

static void start_rx(void)
{
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    /* STM32H573 (Cortex-M33) has no D-Cache — no cache maintenance needed */
    HAL_SPI_Receive_DMA(&s_hspi, s_rx_buf, M1P_MAX_FRAME_SIZE);
}

/* ---------- Public API ---------- */

int spi_slave_init(void)
{
    s_rx_complete = 0;

    gpio_init();
    dma_init();
    spi3_init();

    /* Start listening for first frame from ESP32 */
    start_rx();

    return 0;
}

SPI_HandleTypeDef *spi_slave_get_handle(void)
{
    return &s_hspi;
}

void spi_slave_assert_data_ready(void)
{
    HAL_GPIO_WritePin(SPI_SLAVE_DR_PORT, SPI_SLAVE_DR_PIN, GPIO_PIN_SET);
}

void spi_slave_deassert_data_ready(void)
{
    HAL_GPIO_WritePin(SPI_SLAVE_DR_PORT, SPI_SLAVE_DR_PIN, GPIO_PIN_RESET);
}

void spi_slave_notify_log_available(void)
{
    if (cmd_handler_log_pending()) {
        spi_slave_assert_data_ready();
    }
}

void spi_slave_txrx_complete_callback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI_SLAVE_PERIPH) {
        s_rx_complete = 1;
    }
}

int spi_slave_send_response(uint8_t cmd, uint8_t flags, uint16_t seq,
                            const uint8_t *payload, uint16_t payload_len)
{
    size_t frame_len = m1p_build_frame(s_tx_buf, sizeof(s_tx_buf),
                                       cmd, flags | M1P_FLAG_RESPONSE, seq,
                                       payload, payload_len);
    if (frame_len == 0) return -1;

    /* STM32H573 (Cortex-M33) has no D-Cache — no cache maintenance needed */

    /*
     * Load TX DMA. When ESP32 clocks the next transaction to read
     * our response, this data gets shifted out on MISO.
     */
    HAL_SPI_Transmit_DMA(&s_hspi, s_tx_buf, frame_len);

    return 0;
}

int spi_slave_send_error(uint16_t seq, m1p_error_t error)
{
    uint8_t err_code = (uint8_t)error;
    return spi_slave_send_response(0x00, M1P_FLAG_ERROR, seq,
                                   &err_code, 1);
}

void spi_slave_process(void)
{
    if (!s_rx_complete) return;
    s_rx_complete = 0;

    /* STM32H573 (Cortex-M33) has no D-Cache — no cache maintenance needed */

    /* Parse incoming frame */
    m1p_header_t hdr;
    const uint8_t *payload;
    uint16_t payload_len;

    int ret = m1p_parse_frame(s_rx_buf, sizeof(s_rx_buf),
                              &hdr, &payload, &payload_len);
    if (ret != 0) {
        /* Bad frame — could be ESP32 clocking zeros to read our response */
        goto restart;
    }

    /* Skip response frames (our own echoes in full-duplex) */
    if (hdr.flags & M1P_FLAG_RESPONSE) {
        goto restart;
    }

    /* Dispatch to command handler — it will call spi_slave_send_response() */
    cmd_handler_dispatch(&hdr, payload, payload_len);

restart:
    /* Re-arm DMA receive for next frame */
    start_rx();
}

/* ---------- IRQ Handlers ---------- */

/*
 * GPDMA1_Channel4/5 and SPI3 IRQ handlers already exist in m1_int_hdl.c.
 * The existing SPI3_IRQHandler calls HAL_SPI_IRQHandler on the master handle.
 * For the experimental brain architecture, those handlers need to be updated
 * in m1_int_hdl.c to call our DMA/SPI handles instead.
 *
 * TODO: When integrating, update m1_int_hdl.c:
 *   GPDMA1_Channel4_IRQHandler → HAL_DMA_IRQHandler(spi_slave DMA RX)
 *   GPDMA1_Channel5_IRQHandler → HAL_DMA_IRQHandler(spi_slave DMA TX)
 *   SPI3_IRQHandler → HAL_SPI_IRQHandler(spi_slave_get_handle())
 */

/*
 * HAL callback — overrides weak symbol from HAL
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_slave_txrx_complete_callback(hspi);
}
