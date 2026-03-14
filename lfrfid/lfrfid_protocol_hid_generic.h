/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — HID Generic (any HID Prox format)
 *
 * Based on the Flipper Zero HID Generic protocol approach.
 * Accepts any HID Proximity card regardless of bit format
 * (26, 34, 35, 37-bit, etc.) by checking only for valid
 * preamble + Manchester encoding.
 *
 * Original project:
 * https://github.com/flipperdevices/flipperzero-firmware
 *
 * Copyright (C) Flipper Devices Inc.
 * Licensed under the GNU General Public License v3.0 (GPLv3).
 *
 * Modifications and additional implementation:
 * Copyright (C) 2026 Monstatek
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef LFRFID_PROTOCOL_HID_GENERIC_H_
#define LFRFID_PROTOCOL_HID_GENERIC_H_

#define HID_GENERIC_PREAMBLE        0x1D
#define HID_GENERIC_ENCODED_SIZE    13      /* 1 preamble + 11 data + 1 preamble */
#define HID_GENERIC_DATA_BYTES      11      /* Manchester-encoded data between preambles */
#define HID_GENERIC_DECODED_SIZE    6       /* 44 decoded bits → 6 bytes */

extern const LFRFIDProtocolBase protocol_hid_generic;

#endif /* LFRFID_PROTOCOL_HID_GENERIC_H_ */
