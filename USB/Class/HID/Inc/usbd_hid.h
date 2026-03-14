/**
  ******************************************************************************
  * @file    usbd_hid.h
  * @brief   USB HID Keyboard class header
  ******************************************************************************
  */

#ifndef __USBD_HID_H
#define __USBD_HID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"

/* HID Class defines */
#define HID_EPIN_ADDR                  0x81U
#define HID_EPIN_SIZE                  8U

#define USB_HID_CONFIG_DESC_SIZ        34U
#define USB_HID_DESC_SIZ               9U

#define HID_DESCRIPTOR_TYPE            0x21U
#define HID_REPORT_DESC                0x22U

#define HID_HS_BINTERVAL               2U    /* Flipper Zero uses 2ms */
#define HID_FS_BINTERVAL               2U    /* 2ms polling for fast keystroke injection */

#define HID_REQ_SET_PROTOCOL           0x0BU
#define HID_REQ_GET_PROTOCOL           0x03U
#define HID_REQ_SET_IDLE               0x0AU
#define HID_REQ_GET_IDLE               0x02U
#define HID_REQ_SET_REPORT             0x09U
#define HID_REQ_GET_REPORT             0x01U

#define HID_KEYBOARD_REPORT_DESC_SIZE  65U

/* Alias for composite builder compatibility */
#define HID_MOUSE_REPORT_DESC_SIZE     HID_KEYBOARD_REPORT_DESC_SIZE

/* HID Keyboard report modifier bits */
#define HID_MOD_LCTRL                  0x01U
#define HID_MOD_LSHIFT                 0x02U
#define HID_MOD_LALT                   0x04U
#define HID_MOD_LGUI                   0x08U
#define HID_MOD_RCTRL                  0x10U
#define HID_MOD_RSHIFT                 0x20U
#define HID_MOD_RALT                   0x40U
#define HID_MOD_RGUI                   0x80U

/* HID state */
typedef enum
{
  HID_IDLE = 0,
  HID_BUSY,
} HID_StateTypeDef;

/* HID functional descriptor */
typedef struct
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bcdHID;
  uint8_t  bCountryCode;
  uint8_t  bNumDescriptors;
  uint8_t  bHIDDescriptorType;
  uint16_t wItemLength;
} __PACKED USBD_HIDDescTypeDef;

/* HID class handle */
typedef struct
{
  uint8_t  Report_buf[8];
  uint32_t Protocol;
  uint32_t IdleState;
  uint32_t AltSetting;
  HID_StateTypeDef state;
} USBD_HID_HandleTypeDef;

extern USBD_ClassTypeDef USBD_HID;

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev,
                             uint8_t *report, uint16_t len);

/* Debug instrumentation counters */
typedef struct
{
  uint32_t send_calls;       /* Total USBD_HID_SendReport calls */
  uint32_t send_queued;      /* Reports actually queued (state was IDLE) */
  uint32_t send_busy;        /* Reports dropped (state was BUSY) */
  uint32_t send_null;        /* Reports failed (pClassData NULL) */
  uint32_t send_not_cfg;     /* Reports failed (not CONFIGURED) */
  uint32_t datain_count;     /* DataIn callbacks (host ACKs) */
  uint32_t init_count;       /* USBD_HID_Init calls */
  uint32_t deinit_count;     /* USBD_HID_DeInit calls */
  uint32_t setup_count;      /* Setup requests received */
  uint32_t last_tx_ret;      /* Last USBD_LL_Transmit return value */
} USBD_HID_DbgCounters_t;

extern USBD_HID_DbgCounters_t hid_dbg;

void USBD_HID_ResetDbgCounters(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_HID_H */
