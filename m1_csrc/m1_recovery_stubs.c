/* See COPYING.txt for license details. */

/*
 * m1_recovery_stubs.c
 *
 * Recovery FW ONLY (compiled only when -DRECOVERY=ON defines M1_RECOVERY_BUILD).
 *
 * The recovery build strips the NFC / Sub-GHz / IR / ESP32 / CLI feature source
 * files from the image. A handful of KEPT core files (m1_rpc.c, m1_int_hdl.c,
 * main.c, stm32h5xx_it.c, usbd_cdc_if.c) still REFERENCE symbols from those
 * subsystems — but only inside interrupt handlers and ESP-flash RPC paths that
 * are NEVER reached in recovery (the radios, ESP32, and IR/timer peripherals
 * are never initialised, so their ISRs never fire and their RPC commands are
 * never issued).
 *
 * These definitions exist solely to satisfy the linker. None of them is ever
 * executed at run time. Functions are linked by name only, so exact original
 * signatures are unnecessary; globals just provide storage that is never read.
 */

#ifdef M1_RECOVERY_BUILD

#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include "m1_ring_buffer.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- globals (HAL handles / flags), never accessed at run time ---- */
TIM_HandleTypeDef  Timerhdl_IrRx;
TIM_HandleTypeDef  Timerhdl_IrTx;
TIM_HandleTypeDef  timerhdl_subghz_rx;
TIM_HandleTypeDef  timerhdl_subghz_tx;
SPI_HandleTypeDef  hspi_esp;
UART_HandleTypeDef huart_esp;
DMA_HandleTypeDef  hdma_subghz_tx;
DMA_HandleTypeDef  hgpdma1_channel5_tx;
EXTI_HandleTypeDef H_EXTI_0;
EXTI_HandleTypeDef si4463_exti_hdl;
EXTI_HandleTypeDef esp32_exti_handshake;
EXTI_HandleTypeDef esp32_exti_dataready;
SemaphoreHandle_t  sem_esp32_trans;
QueueHandle_t      esp_spi_msg_queue;
osThreadId_t       cmdLineTaskHandle;
S_M1_RingBuffer    esp32_rb_hdl;
S_M1_RingBuffer    subghz_rx_rawdata_rb;
volatile uint8_t   radio_state_flag;
volatile uint8_t   si446x_nIRQ_active;
uint8_t            subghz_tx_tc_flag;
uint8_t            subghz_record_mode_flag;
volatile uint8_t   ir_ota_data_tx_active;
volatile uint16_t  ir_ota_data_tx_counter;
uint16_t           ir_ota_data_tx_len;
uint16_t          *pir_ota_data_tx_buffer;
/* Feature struct types — opaque storage (their heavy headers aren't included;
 * never accessed since their ISRs never fire in recovery). */
uint8_t            subghz_decenc_ctl[512];   /* was SubGHz_DecEnc_t   */
volatile uint8_t   IrRx_Edge_Det[8];         /* was S_M1_IR_Det       */

/* ---- functions (never called; linked by name only) ---- */
void    esp32_enable(void)                        { }
void    esp32_disable(void)                       { }
void    esp32_UART_init(void)                     { }
void    esp32_UART_deinit(void)                   { }
void    esp32_UART_change_baudrate(uint32_t b)    { (void)b; }
void    esp32_uartrx_handler(uint8_t rx)          { (void)rx; }
uint8_t m1_esp32_get_init_status(void)            { return 0; }
void    m1_qmon_relay_suspend(bool s)             { (void)s; }
int     connect_to_target(uint32_t b)             { (void)b; return -1; }
int     esp_loader_flash_start(uint32_t o, uint32_t i, uint32_t bs) { (void)o; (void)i; (void)bs; return -1; }
int     esp_loader_flash_write(void *p, uint32_t s){ (void)p; (void)s; return -1; }
int     esp_loader_flash_verify(void)             { return -1; }
void    esp_loader_reset_target(void)             { }
void    esp_loader_get_md5_diagnostic(uint8_t *e, uint8_t *a, uint32_t o, uint32_t s) { (void)e; (void)a; (void)o; (void)s; }
int     loader_port_stm32_init(void *cfg)         { (void)cfg; return -1; }
bool    m1_esp_client_screen_push(const uint8_t *fb, uint16_t len) { (void)fb; (void)len; return false; }
bool    m1_esp_client_fw_version(char *o, uint16_t c) { (void)o; (void)c; return false; }
bool    m1_esp_client_beacon_stop(void)           { return false; }
bool    m1_esp_client_now_stop(void)              { return false; }
bool    m1_esp_client_hs_stop(void)               { return false; }
bool    m1_esp_client_deauth_stop(void)           { return false; }
bool    m1_esp_client_ble_hid_deinit(void)        { return false; }
bool    m1_esp_client_wifi_disconnect(void)       { return false; }
bool    m1_esp_client_zb_sniff_stop(void)         { return false; }
void    irsnd_on(void)                            { }
void    irsnd_off(void)                           { }
uint8_t irmp_start_bit_is_detected(void)          { return 0; }
long    FreeRTOS_CLIProcessCommand(const char *in, char *out, size_t len) { (void)in; (void)out; (void)len; return 0; }

#endif /* M1_RECOVERY_BUILD */
