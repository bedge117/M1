/**
  ******************************************************************************
  * @file    usbd_hid.c
  * @brief   USB HID Keyboard class implementation
  *
  * Standalone (non-composite) USB HID keyboard class for BadUSB
  * keystroke injection. Presents as a boot-protocol keyboard device.
  ******************************************************************************
  */

#include "usbd_hid.h"
#include "usbd_ctlreq.h"
#include "usbd_conf.h"
#include <string.h>

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);

USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL,                 /* EP0_TxSent */
  NULL,                 /* EP0_RxReady */
  USBD_HID_DataIn,
  NULL,                 /* DataOut */
  NULL,                 /* SOF */
  NULL,                 /* IsoINIncomplete */
  NULL,                 /* IsoOUTIncomplete */
  NULL,                 /* GetHSConfigDescriptor (not HS) */
  USBD_HID_GetFSCfgDesc,
  NULL,                 /* GetOtherSpeedConfigDescriptor */
  USBD_HID_GetDeviceQualifierDesc,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  NULL,                 /* GetUsrStrDescriptor */
#endif
};

/* USB HID Keyboard Report Descriptor (boot keyboard, 65 bytes) */
__ALIGN_BEGIN static const uint8_t HID_Keyboard_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE] __ALIGN_END =
{
  0x05, 0x01,       /* Usage Page (Generic Desktop) */
  0x09, 0x06,       /* Usage (Keyboard) */
  0xA1, 0x01,       /* Collection (Application) */

  /* Modifier keys (8 bits) */
  0x05, 0x07,       /*   Usage Page (Key Codes) */
  0x19, 0xE0,       /*   Usage Minimum (224 = Left Control) */
  0x29, 0xE7,       /*   Usage Maximum (231 = Right GUI) */
  0x15, 0x00,       /*   Logical Minimum (0) */
  0x25, 0x01,       /*   Logical Maximum (1) */
  0x75, 0x01,       /*   Report Size (1) */
  0x95, 0x08,       /*   Report Count (8) */
  0x81, 0x02,       /*   Input (Data, Variable, Absolute) = modifier byte */

  /* Reserved byte */
  0x95, 0x01,       /*   Report Count (1) */
  0x75, 0x08,       /*   Report Size (8) */
  0x81, 0x01,       /*   Input (Constant) = reserved byte */

  /* LED output report (5 bits) */
  0x95, 0x05,       /*   Report Count (5) */
  0x75, 0x01,       /*   Report Size (1) */
  0x05, 0x08,       /*   Usage Page (LEDs) */
  0x19, 0x01,       /*   Usage Minimum (1 = Num Lock) */
  0x29, 0x05,       /*   Usage Maximum (5 = Kana) */
  0x91, 0x02,       /*   Output (Data, Variable, Absolute) = LED bits */

  /* LED padding (3 bits) */
  0x95, 0x01,       /*   Report Count (1) */
  0x75, 0x03,       /*   Report Size (3) */
  0x91, 0x01,       /*   Output (Constant) = padding */

  /* Key array (6 bytes) */
  0x95, 0x06,       /*   Report Count (6) */
  0x75, 0x08,       /*   Report Size (8) */
  0x15, 0x00,       /*   Logical Minimum (0) */
  0x26, 0xFF, 0x00, /*   Logical Maximum (255) */
  0x05, 0x07,       /*   Usage Page (Key Codes) */
  0x19, 0x00,       /*   Usage Minimum (0) */
  0x2A, 0xFF, 0x00, /*   Usage Maximum (255) */
  0x81, 0x00,       /*   Input (Data, Array) = keycodes */

  0xC0              /* End Collection */
};

/* USB HID Configuration Descriptor (standalone, single interface) */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                              /* bLength */
  USB_DESC_TYPE_CONFIGURATION,       /* bDescriptorType */
  USB_HID_CONFIG_DESC_SIZ, 0x00,    /* wTotalLength */
  0x01,                              /* bNumInterfaces */
  0x01,                              /* bConfigurationValue */
  0x00,                              /* iConfiguration */
  0x80 | (USBD_SELF_POWERED << 6),  /* bmAttributes */
  USBD_MAX_POWER,                   /* bMaxPower */

  /* Interface Descriptor */
  0x09,                              /* bLength */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType */
  0x00,                              /* bInterfaceNumber */
  0x00,                              /* bAlternateSetting */
  0x01,                              /* bNumEndpoints */
  0x03,                              /* bInterfaceClass: HID */
  0x01,                              /* bInterfaceSubClass: Boot */
  0x01,                              /* bInterfaceProtocol: Keyboard */
  0x00,                              /* iInterface */

  /* HID Descriptor */
  0x09,                              /* bLength */
  HID_DESCRIPTOR_TYPE,               /* bDescriptorType: HID */
  0x11, 0x01,                        /* bcdHID: 1.11 */
  0x00,                              /* bCountryCode */
  0x01,                              /* bNumDescriptors */
  HID_REPORT_DESC,                   /* bDescriptorType: Report */
  HID_KEYBOARD_REPORT_DESC_SIZE,     /* wDescriptorLength (low) */
  0x00,                              /* wDescriptorLength (high) */

  /* Endpoint Descriptor (IN) */
  0x07,                              /* bLength */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
  HID_EPIN_ADDR,                     /* bEndpointAddress */
  0x03,                              /* bmAttributes: Interrupt */
  HID_EPIN_SIZE, 0x00,              /* wMaxPacketSize */
  HID_FS_BINTERVAL,                  /* bInterval */
};

/* Device Qualifier Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, 0x02,     /* bcdUSB: 2.00 */
  0x00,            /* bDeviceClass */
  0x00,            /* bDeviceSubClass */
  0x00,            /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,/* bMaxPacketSize0 */
  0x01,            /* bNumConfigurations */
  0x00,            /* bReserved */
};

/* Class data handle (static allocation) */
static USBD_HID_HandleTypeDef hid_handle;

/* Debug instrumentation counters (stub — kept for m1_badusb.c compat) */
USBD_HID_DbgCounters_t hid_dbg;

void USBD_HID_ResetDbgCounters(void)
{
  memset(&hid_dbg, 0, sizeof(hid_dbg));
}

/**
  * @brief  Initialize HID keyboard class
  */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  (void)cfgidx;

  /* Open interrupt IN endpoint */
  (void)USBD_LL_OpenEP(pdev, HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
  pdev->ep_in[HID_EPIN_ADDR & 0x7FU].is_used = 1U;
  pdev->ep_in[HID_EPIN_ADDR & 0x7FU].bInterval = HID_FS_BINTERVAL;

  pdev->pClassData = &hid_handle;

  hid_handle.state = HID_IDLE;
  hid_handle.Protocol = 0U; /* Boot protocol */
  hid_handle.IdleState = 0U;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  De-initialize HID keyboard class
  */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  (void)cfgidx;

  /* Close endpoint */
  (void)USBD_LL_CloseEP(pdev, HID_EPIN_ADDR);
  pdev->ep_in[HID_EPIN_ADDR & 0x7FU].is_used = 0U;
  pdev->ep_in[HID_EPIN_ADDR & 0x7FU].bInterval = 0U;

  pdev->pClassData = NULL;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  Handle HID setup requests
  */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t len;
  uint8_t *pbuf;

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case HID_REQ_GET_PROTOCOL:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
          break;

        case HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case HID_REQ_GET_IDLE:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
          break;

        case HID_REQ_SET_REPORT:
          /* Host sends LED state — receive into Report_buf but don't act on it */
          if (req->wLength <= sizeof(hhid->Report_buf))
          {
            (void)USBD_CtlPrepareRx(pdev, hhid->Report_buf, req->wLength);
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == HID_REPORT_DESC)
          {
            pbuf = (uint8_t *)HID_Keyboard_ReportDesc;
            len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE, req->wLength);
            (void)USBD_CtlSendData(pdev, pbuf, len);
          }
          else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
          {
            /* Return HID descriptor from offset 18 in config descriptor */
            pbuf = USBD_HID_CfgFSDesc + 18U;
            len = MIN(USB_HID_DESC_SIZ, req->wLength);
            (void)USBD_CtlSendData(pdev, pbuf, len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_INTERFACE:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
          break;

        case USB_REQ_SET_INTERFACE:
          hhid->AltSetting = (uint8_t)(req->wValue);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  Send an 8-byte HID keyboard report
  * @param  pdev: USB device handle
  * @param  report: pointer to 8-byte report buffer
  * @param  len: report length (should be 8)
  * @retval USBD status
  */
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev,
                              uint8_t *report, uint16_t len)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (pdev->dev_state != USBD_STATE_CONFIGURED)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (hhid->state == HID_IDLE)
  {
    hhid->state = HID_BUSY;
    (void)USBD_LL_Transmit(pdev, HID_EPIN_ADDR, report, len);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  DataIn callback — endpoint transfer complete
  */
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  (void)epnum;
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

  if (hhid != NULL)
  {
    hhid->state = HID_IDLE;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  Return FS configuration descriptor
  */
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_CfgFSDesc);
  return USBD_HID_CfgFSDesc;
}

/**
  * @brief  Return device qualifier descriptor
  */
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_DeviceQualifierDesc);
  return USBD_HID_DeviceQualifierDesc;
}
