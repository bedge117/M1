/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Indala 26-bit PSK protocol
 *
 * Based on the Flipper Zero Indala26 protocol approach.
 * PSK1 modulation, 255us per bit, 64-bit encoded frame.
 *
 * Original project:
 * https://github.com/flipperdevices/flipperzero-firmware
 *
 * Copyright (C) Flipper Devices Inc.
 * Licensed under the GNU General Public License v3.0 (GPLv3).
 *
 * Modifications and additional implementation:
 * Copyright (C) 2026 Monstatek
 */

#ifndef LFRFID_PROTOCOL_INDALA26_H_
#define LFRFID_PROTOCOL_INDALA26_H_

#define INDALA26_US_PER_BIT          255
#define INDALA26_ENCODED_BIT_SIZE    64
#define INDALA26_ENCODED_DATA_SIZE   (((INDALA26_ENCODED_BIT_SIZE) / 8) + 5)  /* 8 + 5 preamble */
#define INDALA26_DECODED_DATA_SIZE   4

extern const LFRFIDProtocolBase protocol_indala26;

#endif /* LFRFID_PROTOCOL_INDALA26_H_ */
