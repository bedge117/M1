/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — HID Ex Generic (extended HID Prox)
 *
 * Based on the Flipper Zero HID Ex Generic protocol.
 * FSK2a modulation, 200-bit frame with Manchester encoding.
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

static uint8_t g_hidex_encoded[HID_EX_ENCODED_SIZE];   /* 25 bytes */
static uint8_t g_hidex_decoded[HID_EX_DECODED_SIZE];    /* 12 bytes */
static fsk_symbol_state_t g_hidex_sym_st;
static fsk_bit_state_t    g_hidex_bit_st;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

static void hidex_push_bit(uint8_t *buf, size_t len, uint8_t bit)
{
    for (size_t i = 0; i < len - 1; i++)
        buf[i] = (uint8_t)((buf[i] << 1) | (buf[i + 1] >> 7));
    buf[len - 1] = (uint8_t)((buf[len - 1] << 1) | (bit & 1));
}

/*============================================================================*/
/* Flipper-exact HID Ex Generic validation                                    */
/* Preamble 0x1D at byte[0] AND byte[24] (both ends of 25-byte buffer)      */
/* Manchester pair validation for bytes 1-23 (no 0x00 or 0x03 pairs)         */
/*============================================================================*/
static bool hidex_can_be_decoded(const uint8_t *data)
{
    if (data[0] != HID_EX_PREAMBLE)
        return false;
    if (data[HID_EX_ENCODED_SIZE - 1] != HID_EX_PREAMBLE)
        return false;

    /* Validate Manchester pairs in data bytes 1..23 */
    for (size_t i = 1; i <= HID_EX_DATA_BYTES; i++) {
        for (size_t n = 0; n < 4; n++) {
            uint8_t pair = (data[i] >> (n * 2)) & 0x03;
            if (pair == 0x03 || pair == 0x00)
                return false;
        }
    }
    return true;
}

/*============================================================================*/
/* Manchester decode: 184 encoded bits (bytes 1-23) -> 92 decoded bits        */
/* Pair 0x02 (10) -> bit 1, Pair 0x01 (01) -> bit 0, MSB-first (n=3..0)     */
/* 92 decoded bits = 11.5 bytes -> stored in 12 bytes                         */
/*============================================================================*/
static void hidex_manchester_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, HID_EX_DECODED_SIZE);
    int out_bit = 0;

    for (size_t i = 1; i <= HID_EX_DATA_BYTES; i++) {
        for (int n = 3; n >= 0; n--) {
            uint8_t pair = (encoded[i] >> (n * 2)) & 0x03;
            uint8_t bit = (pair == 0x02) ? 1 : 0;  /* 10 = 1, 01 = 0 */
            if (bit)
                decoded[out_bit / 8] |= (uint8_t)(1 << (7 - (out_bit % 8)));
            out_bit++;
            if (out_bit >= HID_EX_DECODED_SIZE * 8)
                return;
        }
    }
}

/*============================================================================*/
static void hidex_begin_impl(void)
{
    memset(g_hidex_encoded, 0, sizeof(g_hidex_encoded));
    memset(g_hidex_decoded, 0, sizeof(g_hidex_decoded));
    fsk_symbol_state_init(&g_hidex_sym_st);
    fsk_bit_state_init(&g_hidex_bit_st);
}

/*============================================================================*/
static bool hidex_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        uint8_t symbol;
        if (fsk_symbol_feed(&g_hidex_sym_st, &evt[i], &symbol)) {
            uint8_t bit;
            if (fsk_bit_feed(&g_hidex_bit_st, symbol, &bit)) {
                hidex_push_bit(g_hidex_encoded, HID_EX_ENCODED_SIZE, bit);

                if (hidex_can_be_decoded(g_hidex_encoded)) {
                    hidex_manchester_decode(g_hidex_encoded, g_hidex_decoded);
                    memcpy(lfrfid_tag_info.uid, g_hidex_decoded,
                           min(sizeof(lfrfid_tag_info.uid), HID_EX_DECODED_SIZE));
                    return true;
                }
            }
        }
    }
    return false;
}

/*============================================================================*/
static uint8_t *hidex_get_data(void *proto) { (void)proto; return g_hidex_decoded; }

/*============================================================================*/
static void hidex_render_data(void *proto, char *result)
{
    (void)proto;
    sprintf(result,
            "HID Extended Prox\n"
            "Hex: %02X%02X%02X%02X%02X",
            g_hidex_decoded[0], g_hidex_decoded[1], g_hidex_decoded[2],
            g_hidex_decoded[3], g_hidex_decoded[4]);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_hid_ex = {
    .name = "HID Ex Generic",
    .manufacturer = "HID",
    .data_size = HID_EX_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)hidex_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)hidex_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)hidex_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)hidex_render_data,
};
