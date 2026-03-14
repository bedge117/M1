/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Paradox protocol decoder
 *
 * Based on the Flipper Zero Paradox protocol.
 * FSK2a modulation, 96-bit frame with Manchester encoding.
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

static uint8_t g_paradox_encoded[PARADOX_ENCODED_SIZE];   /* 13 bytes */
static uint8_t g_paradox_decoded[PARADOX_DECODED_SIZE];    /* 6 bytes */
static fsk_symbol_state_t g_paradox_sym_st;
static fsk_bit_state_t    g_paradox_bit_st;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

static void paradox_push_bit(uint8_t *buf, size_t len, uint8_t bit)
{
    for (size_t i = 0; i < len - 1; i++)
        buf[i] = (uint8_t)((buf[i] << 1) | (buf[i + 1] >> 7));
    buf[len - 1] = (uint8_t)((buf[len - 1] << 1) | (bit & 1));
}

/*============================================================================*/
/* Flipper-exact Paradox validation                                           */
/* Preamble 0x0F at byte[0] AND byte[12] (both ends of 13-byte buffer)       */
/* Manchester pair validation for bytes 1-11 (no 0x00 or 0x03 pairs)         */
/*============================================================================*/
static bool paradox_can_be_decoded(const uint8_t *data)
{
    if (data[0] != 0x0F)
        return false;
    if (data[PARADOX_ENCODED_SIZE - 1] != 0x0F)
        return false;

    /* Manchester pair validation for bytes 1-11 */
    for (size_t i = 1; i <= 11; i++) {
        for (size_t n = 0; n < 4; n++) {
            uint8_t pair = (data[i] >> (n * 2)) & 0x03;
            if (pair == 0x03 || pair == 0x00)
                return false;
        }
    }
    return true;
}

/*============================================================================*/
/* Manchester decode: 88 encoded bits (bytes 1-11) -> 44 decoded bits         */
/* Pair 0x02 (10) -> bit 1, Pair 0x01 (01) -> bit 0, MSB-first (n=3..0)     */
/* Then 4 trailing zero bits appended to fill 48 decoded bits (6 bytes)       */
/*============================================================================*/
static void paradox_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, PARADOX_DECODED_SIZE);
    int out_bit = 0;

    for (size_t i = 1; i <= 11; i++) {
        for (int n = 3; n >= 0; n--) {
            uint8_t pair = (encoded[i] >> (n * 2)) & 0x03;
            uint8_t bit = (pair == 0x02) ? 1 : 0;  /* 10 = 1, 01 = 0 */
            if (bit)
                decoded[out_bit / 8] |= (uint8_t)(1 << (7 - (out_bit % 8)));
            out_bit++;
            if (out_bit >= PARADOX_DECODED_SIZE * 8)
                return;
        }
    }
    /* Remaining bits (44..47) stay zero — 4 trailing zero bits */
}

/*============================================================================*/
static void paradox_begin_impl(void)
{
    memset(g_paradox_encoded, 0, sizeof(g_paradox_encoded));
    memset(g_paradox_decoded, 0, sizeof(g_paradox_decoded));
    fsk_symbol_state_init(&g_paradox_sym_st);
    fsk_bit_state_init(&g_paradox_bit_st);
}

/*============================================================================*/
static bool paradox_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        uint8_t symbol;
        if (fsk_symbol_feed(&g_paradox_sym_st, &evt[i], &symbol)) {
            uint8_t bit;
            if (fsk_bit_feed(&g_paradox_bit_st, symbol, &bit)) {
                paradox_push_bit(g_paradox_encoded, PARADOX_ENCODED_SIZE, bit);

                if (paradox_can_be_decoded(g_paradox_encoded)) {
                    paradox_decode(g_paradox_encoded, g_paradox_decoded);
                    memcpy(lfrfid_tag_info.uid, g_paradox_decoded,
                           min(sizeof(lfrfid_tag_info.uid), PARADOX_DECODED_SIZE));
                    return true;
                }
            }
        }
    }
    return false;
}

/*============================================================================*/
static uint8_t *paradox_get_data(void *proto) { (void)proto; return g_paradox_decoded; }

/*============================================================================*/
static void paradox_render_data(void *proto, char *result)
{
    (void)proto;
    /* FC at decoded bits 10-17, Card at decoded bits 18-33 */
    uint8_t fc = bl_get_bits(g_paradox_decoded, 10, 8);
    uint16_t cn = bl_get_bits_16(g_paradox_decoded, 18, 16);

    sprintf(result,
            "FC: %u  Card: %u\n"
            "Hex: %02X%02X%02X%02X%02X%02X",
            fc, cn,
            g_paradox_decoded[0], g_paradox_decoded[1],
            g_paradox_decoded[2], g_paradox_decoded[3],
            g_paradox_decoded[4], g_paradox_decoded[5]);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_paradox = {
    .name = "Paradox",
    .manufacturer = "Paradox",
    .data_size = PARADOX_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)paradox_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)paradox_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)paradox_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)paradox_render_data,
};
