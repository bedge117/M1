/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — AWID protocol decoder
 *
 * Based on the Flipper Zero AWID protocol.
 * FSK2a modulation, 96-bit frame, supports 26/34/37-bit formats.
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

static uint8_t g_awid_encoded[AWID_ENCODED_SIZE];   /* 13 bytes */
static uint8_t g_awid_decoded[AWID_DECODED_SIZE];    /* 9 bytes */
static fsk_symbol_state_t g_awid_sym_st;
static fsk_bit_state_t    g_awid_bit_st;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

static void awid_push_bit(uint8_t *buf, size_t len, uint8_t bit)
{
    for (size_t i = 0; i < len - 1; i++)
        buf[i] = (uint8_t)((buf[i] << 1) | (buf[i + 1] >> 7));
    buf[len - 1] = (uint8_t)((buf[len - 1] << 1) | (bit & 1));
}

/*============================================================================*/
/* Helper: extract a single bit from MSB-first byte array                     */
/*============================================================================*/
static uint8_t awid_get_bit(const uint8_t *data, int bit_pos)
{
    return (data[bit_pos / 8] >> (7 - (bit_pos % 8))) & 1;
}

/*============================================================================*/
/* Flipper-exact AWID validation                                              */
/* Preamble: data[0] == 0x01 AND data[12] == 0x01 (both ends of 13-byte buf) */
/* Odd parity every 4 bits for 88 bits starting at bit 8                      */
/* Format length must be 26, 34, 36, 37, or 50                                */
/*============================================================================*/
static bool awid_can_be_decoded(uint8_t *data)
{
    /* Check preamble at both ends (Flipper: data[0] and data[LAST]) */
    if (data[0] != 0x01)
        return false;
    if (data[AWID_ENCODED_SIZE - 1] != 0x01)
        return false;

    /* Check odd parity for every 4 bits starting from bit 8, for 88 bits */
    /* 88 bits / 4 = 22 groups */
    for (int g = 0; g < 22; g++) {
        int base = 8 + g * 4;
        uint8_t count = 0;
        for (int b = 0; b < 4; b++)
            count += awid_get_bit(data, base + b);
        if ((count & 1) == 0)  /* must be odd */
            return false;
    }

    /* Remove parity bits (every 4th bit) from bit 8..95 in-place */
    /* This compacts 88 bits -> 66 data bits */
    /* We work on a copy to extract the format byte */
    uint8_t temp[AWID_ENCODED_SIZE];
    memcpy(temp, data, AWID_ENCODED_SIZE);

    /* Remove every 4th bit: bits 11, 15, 19, 23, ... */
    /* After removal, extract 8 bits at position 8 for format length */
    int dst_bit = 8;
    for (int g = 0; g < 22; g++) {
        int base = 8 + g * 4;
        for (int b = 0; b < 3; b++) {  /* copy 3 data bits, skip 1 parity */
            uint8_t v = awid_get_bit(data, base + b);
            /* set bit in temp */
            int byte_pos = dst_bit / 8;
            int bit_in_byte = 7 - (dst_bit % 8);
            if (v)
                temp[byte_pos] |= (uint8_t)(1 << bit_in_byte);
            else
                temp[byte_pos] &= (uint8_t)~(1 << bit_in_byte);
            dst_bit++;
        }
    }

    /* Format length is 8 bits at position 8 in the compacted data */
    uint8_t len = 0;
    for (int b = 0; b < 8; b++)
        len = (uint8_t)((len << 1) | ((temp[(8 + b) / 8] >> (7 - ((8 + b) % 8))) & 1));

    if (len != 26 && len != 50 && len != 37 && len != 34 && len != 36)
        return false;

    return true;
}

/*============================================================================*/
/* Decode: copy 66 bits from bit 8 after parity removal                       */
/*============================================================================*/
static void awid_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, AWID_DECODED_SIZE);

    /* Remove parity (every 4th bit from bit 8) and copy 66 data bits */
    int dst_bit = 0;
    for (int g = 0; g < 22; g++) {
        int base = 8 + g * 4;
        for (int b = 0; b < 3; b++) {
            uint8_t v = awid_get_bit(encoded, base + b);
            if (v)
                decoded[dst_bit / 8] |= (uint8_t)(1 << (7 - (dst_bit % 8)));
            dst_bit++;
        }
    }
}

/*============================================================================*/
static void awid_begin_impl(void)
{
    memset(g_awid_encoded, 0, sizeof(g_awid_encoded));
    memset(g_awid_decoded, 0, sizeof(g_awid_decoded));
    fsk_symbol_state_init(&g_awid_sym_st);
    fsk_bit_state_init(&g_awid_bit_st);
}

/*============================================================================*/
static bool awid_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        uint8_t symbol;
        if (fsk_symbol_feed(&g_awid_sym_st, &evt[i], &symbol)) {
            uint8_t bit;
            if (fsk_bit_feed(&g_awid_bit_st, symbol, &bit)) {
                awid_push_bit(g_awid_encoded, AWID_ENCODED_SIZE, bit);

                if (awid_can_be_decoded(g_awid_encoded)) {
                    awid_decode(g_awid_encoded, g_awid_decoded);
                    memcpy(lfrfid_tag_info.uid, g_awid_decoded,
                           min(sizeof(lfrfid_tag_info.uid), AWID_DECODED_SIZE));
                    return true;
                }
            }
        }
    }
    return false;
}

/*============================================================================*/
static uint8_t *awid_get_data(void *proto) { (void)proto; return g_awid_decoded; }

/*============================================================================*/
static void awid_render_data(void *proto, char *result)
{
    (void)proto;
    uint8_t fmt = g_awid_decoded[0];

    if (fmt == 26) {
        uint8_t fc = 0;
        uint16_t cn = 0;
        /* FC at bits 9-16 of decoded (after format byte) */
        for (int b = 0; b < 8; b++)
            fc = (uint8_t)((fc << 1) | ((g_awid_decoded[(9+b)/8] >> (7-((9+b)%8))) & 1));
        /* Card at bits 17-32 of decoded */
        for (int b = 0; b < 16; b++)
            cn = (uint16_t)((cn << 1) | ((g_awid_decoded[(17+b)/8] >> (7-((17+b)%8))) & 1));

        sprintf(result, "Format: %u\nFC: %u  Card: %u", fmt, fc, cn);
    } else {
        sprintf(result, "Format: %u\nHex: %02X%02X%02X%02X%02X",
                fmt, g_awid_decoded[0], g_awid_decoded[1], g_awid_decoded[2],
                g_awid_decoded[3], g_awid_decoded[4]);
    }
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_awid = {
    .name = "AWID",
    .manufacturer = "AWID",
    .data_size = AWID_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)awid_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)awid_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)awid_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)awid_render_data,
};
