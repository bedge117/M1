/* See COPYING.txt for license details. */

/*
 * m1_usb_cdc.h
 *
 *      Author:
 */
#ifndef M1_USB_CDC_MSC_H_
#define M1_USB_CDC_MSC_H_

/*********************************************/
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_conf.h"

#if M1_USB_MODE == M1_CFG_USB_CDC_MSC
// USB MSC + CDC
#include "usbd_composite_builder.h"
#include "usbd_msc_storage.h"
#elif M1_USB_MODE == M1_CFG_USB_MSC
// USB MSC
#include "usbd_msc_storage.h"
#elif M1_USB_MODE == M1_CFG_USB_CDC
#endif

#include "usbd_cdc_if.h"

#include "app_freertos.h"
#include "semphr.h"
#include "message_buffer.h"

/*********************************************/
/* USB CDC operation mode configuration */
typedef enum
{
  CDC_MODE_LOG_CLI = 0,
  CDC_MODE_VCP
} enCdcMode;

/*********************************************/
#define USB_FS_CHUNK_SIZE       64

#define USB_RX_BUF_SIZE         1024  //128 //512 //1024  //(USB_FS_CHUNK_SIZE * 8)
#define USB_TX_BUF_SIZE         1024  //(USB_FS_CHUNK_SIZE * 8)

#define RXSTREAMBUF_UART_SIZE   256
#define RXSTREAMBUF_USB_SIZE    2048 //8192 //4096 //2048 //USB_RX_BUF_SIZE*2

/*********************************************/
extern uint8_t CDC_InstID;

extern enCdcMode m1_usbcdc_mode;
extern USBD_CDC_LineCodingTypeDef linecoding;

extern TaskHandle_t usb2ser_task_hdl;
extern TaskHandle_t ser2usb_task_hdl;

extern USBD_DescriptorsTypeDef Class_Desc;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

extern StreamBufferHandle_t h_uart_rx_streambuf;
extern volatile uint16_t head_usart1_dma;
extern volatile uint16_t tail_usart1_dma;

extern StreamBufferHandle_t h_usb_rx_streambuf;
extern SemaphoreHandle_t ser2usb_task_semaphore;
extern SemaphoreHandle_t usb2ser_tx_semaphore;

extern volatile uint8_t usbcdc_rx_paused;
extern volatile int8_t m1_USB_CDC_ready;
extern volatile uint8_t tx_cptl_usart1;

uint16_t usart_get_rx_data_length(void);
void vUsb2SerTask(void *pvParameters);
void vSer2UsbTask(void *pvParameters);
uint16_t usart_rxget_data_length(void);
void usart_rxupdate_head_pointer(void);
void usart_rxdata_process_from_isr(void);
void usb_rxdata_process(void);

void USB_DRD_FS_IRQHandler(void);
void CDC_Signal_Next_Tx(void);
void m1_usb_cdc_comdefault(void);
void m1_usb_cdc_comconfig(void);

/*********************************************/
// USB MSC
extern volatile int8_t m1_USB_MSC_ready;

uint8_t m1_usb_msc_process(void);
uint8_t m1_usb_msc_sd_detected(void);

/*********************************************/
// USB mode switching (BadUSB)
#define M1_USB_MODE_NORMAL   1
#define M1_USB_MODE_HID     -2

#ifdef M1_APP_BADUSB_ENABLE
void m1_usb_switch_to_hid(void);
void m1_usb_switch_to_normal(void);
static inline int8_t m1_usb_get_current_mode(void) { return m1_USB_CDC_ready; }
#endif

#endif /* M1_USB_CDC_MSC_H_ */

