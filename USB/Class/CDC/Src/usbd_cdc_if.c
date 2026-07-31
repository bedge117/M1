/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"
#include "app_freertos.h"
#include "m1_cli.h"
#include "m1_log_debug.h"
#include "m1_usb_cdc_msc.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Defines
  * @{
  */

static uint8_t usbRxBuffer[USB_RX_BUF_SIZE];
static uint16_t usbRxBufIndex = 0;
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Macros
  * @{
  */

/**
  * @}
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */

/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[USB_FS_CHUNK_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[USB_FS_CHUNK_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;


/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_CDC_Interface_fops =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
#if M1_USB_MODE == M1_CFG_USB_CDC_MSC
  // USE_USBD_COMPOSITE
  hUsbDeviceFS.classId = CDC_InstID;
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0, CDC_InstID);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
#elif M1_USB_MODE == M1_CFG_USB_CDC
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
#endif

  return (0);
}

/**
  * @brief  CDC_De\Init_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /*
     Add your deinitialization code here
  */
  return (0);
}


/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  UNUSED(length);

  switch (cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      /* Add your code here */
      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
      /* Add your code here */
      break;

    case CDC_SET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_GET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_CLEAR_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_SET_LINE_CODING:
      linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | \
                                         (pbuf[2] << 16) | (pbuf[3] << 24));
      linecoding.format     = pbuf[4];
      linecoding.paritytype = pbuf[5];
      linecoding.datatype   = pbuf[6];

      /* Store-only. Previously this called m1_usb_cdc_comconfig() from the USB
       * control-transfer (ISR) context, which tore down UART/DMA AND ran
       * m1_logdb_deinit() -> vSemaphoreDelete(mutex_log_write_trans). That mutex
       * is taken with portMAX_DELAY by every M1_LOG_* caller (incl. the RPC
       * worker); deleting it out from under a blocked task hangs that task
       * forever. The host setting line coding does NOT require reconfiguring the
       * physical USART for the qM RPC port, so we only record the coding. */
      break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(linecoding.bitrate);
      pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
      pbuf[4] = linecoding.format;
      pbuf[5] = linecoding.paritytype;
      pbuf[6] = linecoding.datatype;

      /* Add your code here */
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* Add your code here */
      break;

    case CDC_SEND_BREAK:
      /* Add your code here */
      break;

    default:
      break;
  }

  return (0);
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */

uint32_t DEBUG_ovf_cnt = 0;
uint32_t DEBUG_max_usbRxBufIndex = 0;
uint32_t DEBUG_paused_cnt = 0;
uint32_t DEBUG_prev_data[4];
uint32_t DEBUG_ovrl_cnt = 0;
uint32_t DEBUG_drop_cnt = 0;       /* Bytes dropped due to intermediate buffer overflow */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  size_t sentBytes;
  size_t freeSpace;
  size_t SendBytesbf;

  if (m1_usbcdc_mode == CDC_MODE_VCP)
  {
    if (h_usb_rx_streambuf == NULL || *Len == 0) {
      /* Re-arm USB endpoint even for zero-length packets to prevent stalls */
      USBD_CDC_ReceivePacket(&hUsbDeviceFS);
      return USBD_OK;
    }

    if (strncmp((char *)DEBUG_prev_data, (char *)Buf, 4) != 0)
    {
      DEBUG_ovrl_cnt++;
    }
    memcpy(DEBUG_prev_data, Buf, 4);

    /* First, try to drain the intermediate buffer into the stream buffer
     * BEFORE appending new data. This maximizes throughput when the
     * stream buffer was previously full and has since been partially consumed. */
    freeSpace = xStreamBufferSpacesAvailable(h_usb_rx_streambuf);

    if (freeSpace > 0 && usbRxBufIndex > 0) {
      SendBytesbf = (usbRxBufIndex < freeSpace) ? usbRxBufIndex : freeSpace;

      sentBytes = xStreamBufferSendFromISR(h_usb_rx_streambuf,
                                         (void *)usbRxBuffer,
                                         SendBytesbf,
                                         &xHigherPriorityTaskWoken);
      if (sentBytes > 0) {
        if (sentBytes < usbRxBufIndex) {
          memmove(usbRxBuffer,
                  usbRxBuffer + sentBytes,
                  usbRxBufIndex - sentBytes);
        }
        usbRxBufIndex -= sentBytes;
        freeSpace -= sentBytes;

        /* Notify the task — this first drain block previously pushed bytes into
         * the stream buffer WITHOUT waking vUsb2SerTask, which is the lost-wakeup
         * that could strand the endpoint paused with data pending. */
        if (usb2ser_task_hdl != NULL) {
          vTaskNotifyGiveFromISR(usb2ser_task_hdl, &xHigherPriorityTaskWoken);
        }
      }
    }

    /* Append incoming USB data to the intermediate buffer */
    if (usbRxBufIndex + *Len <= USB_RX_BUF_SIZE) {
      memcpy(&usbRxBuffer[usbRxBufIndex], Buf, *Len);
      usbRxBufIndex += *Len;

      if (usbRxBufIndex > DEBUG_max_usbRxBufIndex) DEBUG_max_usbRxBufIndex = usbRxBufIndex;

    } else {
      /* Intermediate buffer overflow: copy what fits, drop the rest */
      uint32_t space_left = USB_RX_BUF_SIZE - usbRxBufIndex;
      if (space_left > 0) {
        memcpy(&usbRxBuffer[usbRxBufIndex], Buf, space_left);
        usbRxBufIndex += space_left;
        DEBUG_drop_cnt += (*Len - space_left);
      } else {
        DEBUG_drop_cnt += *Len;
      }
      DEBUG_ovf_cnt++;
    }

    /* Try to drain intermediate buffer to stream buffer again (now includes new data) */
    freeSpace = xStreamBufferSpacesAvailable(h_usb_rx_streambuf);

    if (freeSpace > 0 && usbRxBufIndex > 0) {
      SendBytesbf = (usbRxBufIndex < freeSpace) ? usbRxBufIndex : freeSpace;

      sentBytes = xStreamBufferSendFromISR(h_usb_rx_streambuf,
                                         (void *)usbRxBuffer,
                                         SendBytesbf,
                                         &xHigherPriorityTaskWoken);
      if (sentBytes > 0) {
        if (sentBytes < usbRxBufIndex) {
          memmove(usbRxBuffer,
                  usbRxBuffer + sentBytes,
                  usbRxBufIndex - sentBytes);
        }
        usbRxBufIndex -= sentBytes;

        /* Notify Task */
        if (usb2ser_task_hdl != NULL) {
          vTaskNotifyGiveFromISR(usb2ser_task_hdl, &xHigherPriorityTaskWoken);
        }
      }
    }

    /* Flow control: pause USB reception if stream buffer has no space
     * AND intermediate buffer is getting full */
    if (freeSpace == 0 && usbRxBufIndex >= (USB_RX_BUF_SIZE / 2)) {
      usbcdc_rx_paused = 1;
      __DSB();  /* Ensure the paused flag write is visible to all bus masters */
    }

    if (usbcdc_rx_paused == 0)
    {
      USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
    else
    {
      DEBUG_paused_cnt++;
    }

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else
  { // == CDC_MODE_LOG_CLI
    Buf[*Len] = 0;
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    memcpy(logdb_rx_buffer, &Buf[0], *Len);
    // add device ID, logdb_rx_buffer[1] = TASK_NOTIFY_USBCDC
    logdb_rx_buffer[1] = TASK_NOTIFY_USBCDC;
    uint32_t u32buf = logdb_rx_buffer[0] + (TASK_NOTIFY_USBCDC << 8);

    //send the char to the command line task, one char
    xTaskNotify(cmdLineTaskHandle, (uint32_t)u32buf, eSetValueWithOverwrite);
  }

  return (USBD_OK);
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted over USB OUT endpoint 1
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data sent (in bytes)
  * @param  epnum: Endpoint number
  * @retval USBD_OK: Results of operation
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  QueueHandle_t q_item;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (m1_usbcdc_mode == CDC_MODE_VCP)
  {
    // After the USB transfer is complete, signal the task to start the next transfer.
    CDC_Signal_Next_Tx();
  }
  else
  {
    if ( !m1_logdb_check_empty_state() ) // There's still data to send?
    {
      xQueueSendFromISR(log_q_hdl, &q_item, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } // if ( !m1_logdb_check_empty_state() )
    m1_logdb_update_tx_buffer(); // Update ring buffer counter
  }

  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted over USB OUT endpoint 1
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data sent (in bytes)
  * @param  epnum: Endpoint number
  * @retval USBD_OK: Results of operation
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  /* pClassData is NULL until the CDC class is registered/enumerated, and can go
   * NULL again on a USB reset/disconnect. Without this guard, hcdc->TxState
   * dereferences NULL+0x214 -> precise BusFault -> HardFault -> random reboot
   * (confirmed via TAMP fault capture: BFAR=0x214, PC in CDC_Transmit_FS). */
  if (hcdc == NULL) {
    return USBD_FAIL;
  }

  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }

#if M1_USB_MODE == M1_CFG_USB_CDC_MSC
  // MSC+CDC Composite
  hUsbDeviceFS.classId = CDC_InstID;
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len, CDC_InstID);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS, CDC_InstID);

#elif M1_USB_MODE == M1_CFG_USB_CDC
  // CDC only
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
#endif

  return result;
}

/* True while a CDC IN transfer is still in flight. The class holds the caller's
 * TX buffer by pointer until the transfer completes (TxState clears in the USB
 * IRQ), so anyone reusing a shared TX buffer must wait for this to go 0 before
 * overwriting it — otherwise the in-flight frame is corrupted on the wire. */
uint8_t CDC_Transmit_Busy(void)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc == NULL)
    return 0;   /* not enumerated — nothing in flight */
  return (hcdc->TxState != 0) ? 1 : 0;
}

/* Abort a stuck IN transfer: a suspended/non-reading host may never generate the
 * DataIn completion that clears TxState, leaving the class holding the caller's
 * TX buffer forever. Flush the endpoint and clear TxState so the buffer is safe
 * to reuse. Safe to call when not enumerated (guards NULL pClassData). */
void CDC_TxAbort(void)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc == NULL)
    return;
  (void)USBD_LL_FlushEP(&hUsbDeviceFS, CDC_IN_EP);
  hcdc->TxState = 0U;
}

/* Re-arm the CDC OUT (RX) endpoint, forcing the CDC instance on a composite
 * device. hUsbDeviceFS.classId is mutable and shared with the MSC class; a prior
 * MSC transfer can leave it pointing at MSC, so re-arming without forcing the CDC
 * instance can arm the wrong endpoint and leave the serial port deaf. Returns
 * USBD_OK only when the rearm was accepted; callers must clear the paused flag
 * ONLY on USBD_OK. */
uint8_t CDC_RearmRx(void)
{
#if M1_USB_MODE == M1_CFG_USB_CDC_MSC
  hUsbDeviceFS.classId = CDC_InstID;
#endif
  return USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

