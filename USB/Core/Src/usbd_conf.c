/**
  ******************************************************************************
  * @file    usbd_conf_template.c
  * @author  MCD Application Team
  * @brief   USB Device configuration and interface file
  *          This template should be copied to the user folder,
  *          renamed and customized following user needs.
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
#include "main.h"
#include "usbd_core.h"
#include "usbd_cdc.h" 				/* Include class header file */
#include "usbd_msc.h"         /* Include class header file */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* Private function prototypes -----------------------------------------------*/
static USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

/* Private functions ---------------------------------------------------------*/
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
  if ( hpcd->Init.speed != PCD_SPEED_FULL)
  {
    Error_Handler();
  }
    /* Set Speed. */
  USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);
  /* Reset Device. */
  USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  Suspend event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  /* Don't override HID-mode sentinel (-2) */
  if (m1_USB_CDC_ready > -2)
    m1_USB_CDC_ready = -1;
  if (m1_USB_MSC_ready > -2)
    m1_USB_MSC_ready = -1;
}

/**
  * @brief  Resume event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  /* Only restore CDC from normal suspend (-1).
   * HID mode uses sentinel -2 which must NOT be overridden. */
#if M1_USB_MODE == M1_CFG_USB_CDC_MSC
  if (m1_USB_CDC_ready == -1)
    m1_USB_CDC_ready = 0;
#elif M1_USB_MODE == M1_CFG_USB_CDC
  if (m1_USB_CDC_ready == -1)
    m1_USB_CDC_ready = 0;
#endif
}

/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  pdev->pData = &hpcd_USB_DRD_FS;
  uint16_t pma_address = 0x40;  // PMA Address start : 0x40

#if M1_USB_MODE == M1_CFG_USB_CDC_MSC
  // CDC+MSC
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, 0x00, PCD_SNG_BUF, pma_address);         // EP0 OUT, 0x0
  pma_address += 64;
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, 0x80, PCD_SNG_BUF, pma_address);         // EP0 IN, 0x01

  /* MSC Endpoints */
  pma_address += 64;
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , MSC_IN_EP, PCD_SNG_BUF, pma_address);   // Bulk IN, 0x81
  pma_address += 64;
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , MSC_OUT_EP, PCD_SNG_BUF, pma_address);  // Bulk OUT, 0x1

  /* CDC Endpoints */
  pma_address += 64;
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, CDC_IN_EP, PCD_SNG_BUF, pma_address);    // Bulk IN, 0x82
  pma_address += 64;
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, CDC_OUT_EP, PCD_SNG_BUF, pma_address);   // Bulk OUT, 0x02
  pma_address += 64;
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, CDC_CMD_EP, PCD_SNG_BUF, pma_address);   // Interrupt, 0x83

#elif M1_USB_MODE == M1_CFG_USB_MSC
  // MSC Only

  /* Control Endpoints */
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, 0x00, PCD_SNG_BUF, 0x40);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, 0x80, PCD_SNG_BUF, 0x80);

  /* MSC Endpoints */
  pma_address += 64;
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , MSC_IN_EP, PCD_SNG_BUF, pma_address);   // Bulk IN, 0x81
  pma_address += 64;
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , MSC_OUT_EP, PCD_SNG_BUF, pma_address);  // Bulk OUT, 0x1

#elif M1_USB_MODE == M1_CFG_USB_CDC
  // CDC Only

  /* Control Endpoints */
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, 0x00, PCD_SNG_BUF, 0x40);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, 0x80, PCD_SNG_BUF, 0x80);

  /* CDC Endpoints */
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, CDC_IN_EP, PCD_SNG_BUF, 0xC0);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, CDC_OUT_EP, PCD_SNG_BUF, 0x100);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData, CDC_CMD_EP, PCD_SNG_BUF, 0x140);
#endif

  return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_DeInit(pdev->pData);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Starts the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_Start(pdev->pData);
      return  USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_Stop(pdev->pData);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
             uint8_t ep_type, uint16_t ep_mps)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev,
             uint8_t ep_addr)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
      PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
      if((ep_addr & 0x80) == 0x80)
      {
             return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
      }
      else
      {
             return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
      }
}

/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  dev_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev,
             uint8_t dev_addr)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
             uint8_t *pbuf, uint32_t size)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
             uint8_t ep_addr, uint8_t *pbuf,
             uint32_t size)
{
      HAL_StatusTypeDef hal_status;
      hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
      return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Returns the last transferred packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Received Data Size
  */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
      return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}
#ifdef USBD_HS_TESTMODE_ENABLE
/**
  * @brief  Set High speed Test mode.
  * @param  pdev: Device handle
  * @param  testmode: test mode
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_SetTestMode(USBD_HandleTypeDef *pdev, uint8_t testmode)
{
  UNUSED(pdev);
  UNUSED(testmode);

  return USBD_OK;
}
#endif /* USBD_HS_TESTMODE_ENABLE */

/**
  * @brief  Static single allocation.
  * @param  size: Size of allocated memory
  * @retval None
  */
void *USBD_static_malloc(uint32_t size)
{
      UNUSED(size);
      static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef) / 4) + 1]; /* On 32-bit boundary */
      return mem;
}

/**
  * @brief  Dummy memory free
  * @param  p: Pointer to allocated  memory address
  * @retval None
  */
void USBD_static_free(void *p)
{
      UNUSED(p);
}

/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay)
{
      HAL_Delay(Delay);
}

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
      USBD_StatusTypeDef usb_status = USBD_OK;
      switch (hal_status)
      {
      case HAL_OK :
             usb_status = USBD_OK;
             break;
      case HAL_ERROR :
             usb_status = USBD_FAIL;
             break;
      case HAL_BUSY :
             usb_status = USBD_BUSY;
             break;
      case HAL_TIMEOUT :
             usb_status = USBD_FAIL;
             break;
      default :
             usb_status = USBD_FAIL;
             break;
      }
      return usb_status;
}

