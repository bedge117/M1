/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — FDX-A (FECAVA) animal tag protocol decoder
 *
 * Based on the Flipper Zero FDX-A protocol.
 * FSK2a modulation, 112-bit frame with Manchester encoding.
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
/*************************** I N C L U D E S **********************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "app_freertos.h"
#include "cmsis_os.h"
#include "main.h"
#include "uiView.h"

#include "lfrfid.h"
#include "lfrfid_bit_lib.h"

/***************************** V A R I A B L E S ******************************/

static uint8_t g_fdxa_encoded[FDX_A_ENCODED_SIZE];   /* 14 bytes */
static uint8_t g_fdxa_decoded[FDX_A_DECODED_SIZE];    /* 5 bytes */
static fsk_symbol_state_t g_fdxa_sym_st;
static fsk_bit_state_t    g_fdxa_bit_st;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

static void fdxa_push_bit(uint8_t *buf, size_t len, uint8_t bit)
{
    for (size_t i = 0; i < len - 1; i++)
        buf[i] = (uint8_t)((buf[i] << 1) | (buf[i + 1] >> 7));
    buf[len - 1] = (uint8_t)((buf[len - 1] << 1) | (bit & 1));
}

/*============================================================================*/
/* Flipper-exact FDX-A validation                                             */
/* Preamble: 0x55, 0x1D at byte[0:1] AND byte[12:13]                        */
/* Manchester pair validation on bytes 2-11 (01 or 10 only)                  */
/* Odd parity check on each decoded byte                                      */
/*============================================================================*/
static bool fdxa_can_be_decoded(const uint8_t *data)
{
    /* Check preamble at both ends */
    if (data[0] != 0x55 || data[1] != 0x1D)
        return false;
    if (data[12] != 0x55 || data[13] != 0x1D)
        return false;

    /* Manchester validation on data bytes 2-11 */
    for (size_t i = 2; i < 12; i++) {
        for (size_t n = 0; n < 4; n++) {
            uint8_t pair = (data[i] >> (6 - (n * 2))) & 0x03;
            if (pair != 0x01 && pair != 0x02)
                return false;
        }
    }

    /* Manchester decode and odd parity check per byte */
    /* 10 data bytes -> 40 Manchester pairs -> 40 bits  */
    /* Every 9th bit is a parity bit: 8 data + 1 parity */
    uint8_t decoded_bits[5];
    memset(decoded_bits, 0, sizeof(decoded_bits));
    int out_bit = 0;

    for (size_t i = 2; i < 12; i++) {
        for (int n = 3; n >= 0; n--) {
            uint8_t pair = (data[i] >> (n * 2)) & 0x03;
            uint8_t bit = (pair == 0x02) ? 1 : 0;
            if (out_bit < 40) {
                if (bit)
                    decoded_bits[out_bit / 8] |= (uint8_t)(1 << (7 - (out_bit % 8)));
                out_bit++;
            }
        }
    }

    /* Check odd parity: for each group of 9 bits (8 data + 1 parity), */
    /* the count of 1s must be odd                                      */
    /* 40 bits = 4 groups of 9 bits + 4 remaining bits                  */
    for (int g = 0; g < 4; g++) {
        int base = g * 9;
        uint8_t count = 0;
        for (int b = 0; b < 9 && (base + b) < 40; b++) {
            if (decoded_bits[(base + b) / 8] & (1 << (7 - ((base + b) % 8))))
                count++;
        }
        if ((count & 1) == 0)
            return false;
    }

    return true;
}

/*============================================================================*/
/* Manchester decode: bytes 2-11 -> 40 decoded bits                           */
/* Remove parity bits (every 9th) -> 32 data bits = 4 bytes                   */
/* Plus remaining 4 bits -> total 5 bytes decoded                             */
/*============================================================================*/
static void fdxa_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, FDX_A_DECODED_SIZE);

    /* First Manchester-decode all 40 bits */
    uint8_t raw_bits[5];
    memset(raw_bits, 0, sizeof(raw_bits));
    int out_bit = 0;

    for (size_t i = 2; i < 12; i++) {
        for (int n = 3; n >= 0; n--) {
            uint8_t pair = (encoded[i] >> (n * 2)) & 0x03;
            uint8_t bit = (pair == 0x02) ? 1 : 0;
            if (bit)
                raw_bits[out_bit / 8] |= (uint8_t)(1 << (7 - (out_bit % 8)));
            out_bit++;
        }
    }

    /* Remove parity bits (every 9th bit: positions 8, 17, 26, 35) */
    /* Copy data bits (skip parity) into decoded */
    int dst = 0;
    for (int b = 0; b < 40; b++) {
        /* Skip parity bit positions: 8, 17, 26, 35 */
        if (b == 8 || b == 17 || b == 26 || b == 35)
            continue;
        bool val = (raw_bits[b / 8] >> (7 - (b % 8))) & 1;
        if (val)
            decoded[dst / 8] |= (uint8_t)(1 << (7 - (dst % 8)));
        dst++;
    }
}

/*============================================================================*/
static void fdxa_begin_impl(void)
{
    memset(g_fdxa_encoded, 0, sizeof(g_fdxa_encoded));
    memset(g_fdxa_decoded, 0, sizeof(g_fdxa_decoded));
    fsk_symbol_state_init(&g_fdxa_sym_st);
    fsk_bit_state_init(&g_fdxa_bit_st);
}

/*============================================================================*/
static bool fdxa_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        uint8_t symbol;
        if (fsk_symbol_feed(&g_fdxa_sym_st, &evt[i], &symbol)) {
            uint8_t bit;
            if (fsk_bit_feed(&g_fdxa_bit_st, symbol, &bit)) {
                fdxa_push_bit(g_fdxa_encoded, FDX_A_ENCODED_SIZE, bit);

                if (fdxa_can_be_decoded(g_fdxa_encoded)) {
                    fdxa_decode(g_fdxa_encoded, g_fdxa_decoded);
                    memcpy(lfrfid_tag_info.uid, g_fdxa_decoded,
                           min(sizeof(lfrfid_tag_info.uid), FDX_A_DECODED_SIZE));
                    return true;
                }
            }
        }
    }
    return false;
}

/*============================================================================*/
static uint8_t *fdxa_get_data(void *proto) { (void)proto; return g_fdxa_decoded; }

/*============================================================================*/
static void fdxa_render_data(void *proto, char *result)
{
    (void)proto;
    sprintf(result,
            "ID: %02X%02X%02X%02X%02X",
            g_fdxa_decoded[0], g_fdxa_decoded[1], g_fdxa_decoded[2],
            g_fdxa_decoded[3], g_fdxa_decoded[4]);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_fdx_a = {
    .name = "FDX-A",
    .manufacturer = "FECAVA",
    .data_size = FDX_A_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)fdxa_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)fdxa_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)fdxa_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)fdxa_render_data,
};
