/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Pyramid (Farpointe) protocol decoder
 *
 * Based on the Flipper Zero Pyramid protocol.
 * FSK2a modulation, 128-bit frame with CRC-8 validation.
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

static uint8_t g_pyr_encoded[PYRAMID_ENCODED_SIZE];   /* 19 bytes */
static uint8_t g_pyr_decoded[PYRAMID_DECODED_SIZE];    /* 4 bytes */
static fsk_symbol_state_t g_pyr_sym_st;
static fsk_bit_state_t    g_pyr_bit_st;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

static void pyr_push_bit(uint8_t *buf, size_t len, uint8_t bit)
{
    for (size_t i = 0; i < len - 1; i++)
        buf[i] = (uint8_t)((buf[i] << 1) | (buf[i + 1] >> 7));
    buf[len - 1] = (uint8_t)((buf[len - 1] << 1) | (bit & 1));
}

/*============================================================================*/
/* Helper: extract a single bit from MSB-first byte array                     */
/*============================================================================*/
static uint8_t pyr_get_bit(const uint8_t *data, int bit_pos)
{
    return (data[bit_pos / 8] >> (7 - (bit_pos % 8))) & 1;
}

/*============================================================================*/
/* Helper: extract 8 bits from MSB-first byte array at arbitrary bit offset   */
/*============================================================================*/
static uint8_t pyr_get_byte(const uint8_t *data, int bit_pos)
{
    uint8_t val = 0;
    for (int b = 0; b < 8; b++)
        val = (uint8_t)((val << 1) | pyr_get_bit(data, bit_pos + b));
    return val;
}

/*============================================================================*/
/* Helper: extract 16 bits from MSB-first byte array at arbitrary bit offset  */
/*============================================================================*/
static uint16_t pyr_get_16(const uint8_t *data, int bit_pos)
{
    uint16_t val = 0;
    for (int b = 0; b < 16; b++)
        val = (uint16_t)((val << 1) | pyr_get_bit(data, bit_pos + b));
    return val;
}

/*============================================================================*/
/* CRC-8 with reflected input/output (matches Flipper bit_lib_crc8)           */
/* poly=0x31, init=0x00, refin=true, refout=true, xorout=0x00                */
/*============================================================================*/
static uint8_t reflect_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

static uint8_t pyramid_crc8(const uint8_t *data, size_t size)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < size; i++) {
        crc ^= reflect_byte(data[i]);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return reflect_byte(crc);
}

/*============================================================================*/
/* Flipper-exact Pyramid validation                                           */
/* 19-byte buffer.                                                            */
/* Preamble at bits 0-23:   0x00 0x01 0x01 (bytes 0,1,2)                     */
/* Preamble at bits 128-143: 0x00 0x01 (bytes 16,17)                          */
/* CRC-8 of 13 bytes (bits 16-119) compared with byte at bits 120-127         */
/* After parity removal: format must be 26                                    */
/*============================================================================*/
static bool pyramid_can_be_decoded(const uint8_t *data)
{
    /* Check first preamble: bits 0-23 = 0x000001 0x01 */
    if (pyr_get_16(data, 0) != 0x0001)
        return false;
    if (pyr_get_byte(data, 16) != 0x01)
        return false;

    /* Check trailing preamble: bits 128-143 = 0x0001 */
    if (pyr_get_16(data, 128) != 0x0001)
        return false;

    /* CRC-8 validation: 13 data bytes at bits 16-119, checksum at bits 120-127 */
    uint8_t checksum = pyr_get_byte(data, 120);
    uint8_t checksum_data[13];
    for (int i = 0; i < 13; i++)
        checksum_data[i] = pyr_get_byte(data, 16 + (i * 8));

    uint8_t calc_crc = pyramid_crc8(checksum_data, 13);
    if (checksum != calc_crc)
        return false;

    /* Remove parity: every 8th bit from bit 8, for 15*8=120 bits */
    /* This gives 105 data bits. Find first set bit to determine format. */
    /* After removing every 8th bit: 120 - 15 = 105 bits remain */
    uint8_t temp[PYRAMID_ENCODED_SIZE];
    memcpy(temp, data, PYRAMID_ENCODED_SIZE);

    int dst_bit = 8;
    for (int i = 0; i < 120; i++) {
        if (((i + 1) % 8) == 0)
            continue;  /* skip every 8th bit (parity) */
        uint8_t v = pyr_get_bit(data, 8 + i);
        int byte_pos = dst_bit / 8;
        int bit_in_byte = 7 - (dst_bit % 8);
        if (v)
            temp[byte_pos] |= (uint8_t)(1 << bit_in_byte);
        else
            temp[byte_pos] &= (uint8_t)~(1 << bit_in_byte);
        dst_bit++;
    }

    /* Find first set bit in compacted data to determine format length */
    int j;
    for (j = 0; j < 105; j++) {
        if ((temp[(j) / 8] >> (7 - ((j) % 8))) & 1)
            break;
    }
    uint8_t fmt_len = (uint8_t)(105 - j);

    /* Only accept 26-bit format (matches Flipper) */
    if (fmt_len != 26)
        return false;

    return true;
}

/*============================================================================*/
/* Decode Pyramid 26-bit format                                               */
/* After parity removal, FC and Card are at known bit positions               */
/*============================================================================*/
static void pyramid_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, PYRAMID_DECODED_SIZE);

    /* Remove parity bits (every 8th from bit 8, for 120 bits) */
    uint8_t temp[PYRAMID_ENCODED_SIZE];
    memset(temp, 0, sizeof(temp));
    int dst_bit = 0;
    for (int i = 0; i < 120; i++) {
        if (((i + 1) % 8) == 0)
            continue;
        uint8_t v = pyr_get_bit(encoded, 8 + i);
        if (v)
            temp[dst_bit / 8] |= (uint8_t)(1 << (7 - (dst_bit % 8)));
        dst_bit++;
    }
    /* temp now has 105 compacted bits starting at bit 0 */

    /* Format = 26 */
    decoded[0] = 26;

    /* In Flipper, after parity removal, FC is at bit 73+8=81, Card at 81+8=89
     * in the encoded_data buffer. But encoded_data starts at bit 0 of the
     * 19-byte buffer, and parity removal starts at bit 8.
     * In our temp buffer (parity-removed data starting at bit 0),
     * the equivalent positions are: FC at bit 73, Card at bit 81.
     * FC = 8 bits, Card = 16 bits. */
    uint8_t fc = 0;
    for (int b = 0; b < 8; b++)
        fc = (uint8_t)((fc << 1) | ((temp[(73+b)/8] >> (7 - ((73+b)%8))) & 1));

    uint16_t cn = 0;
    for (int b = 0; b < 16; b++)
        cn = (uint16_t)((cn << 1) | ((temp[(81+b)/8] >> (7 - ((81+b)%8))) & 1));

    decoded[1] = fc;
    decoded[2] = (uint8_t)(cn >> 8);
    decoded[3] = (uint8_t)(cn & 0xFF);
}

/*============================================================================*/
static void pyramid_begin_impl(void)
{
    memset(g_pyr_encoded, 0, sizeof(g_pyr_encoded));
    memset(g_pyr_decoded, 0, sizeof(g_pyr_decoded));
    fsk_symbol_state_init(&g_pyr_sym_st);
    fsk_bit_state_init(&g_pyr_bit_st);
}

/*============================================================================*/
static bool pyramid_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        uint8_t symbol;
        if (fsk_symbol_feed(&g_pyr_sym_st, &evt[i], &symbol)) {
            uint8_t bit;
            if (fsk_bit_feed(&g_pyr_bit_st, symbol, &bit)) {
                pyr_push_bit(g_pyr_encoded, PYRAMID_ENCODED_SIZE, bit);

                if (pyramid_can_be_decoded(g_pyr_encoded)) {
                    pyramid_decode(g_pyr_encoded, g_pyr_decoded);
                    memcpy(lfrfid_tag_info.uid, g_pyr_decoded,
                           min(sizeof(lfrfid_tag_info.uid), PYRAMID_DECODED_SIZE));
                    return true;
                }
            }
        }
    }
    return false;
}

/*============================================================================*/
static uint8_t *pyramid_get_data(void *proto) { (void)proto; return g_pyr_decoded; }

/*============================================================================*/
static void pyramid_render_data(void *proto, char *result)
{
    (void)proto;
    sprintf(result,
            "Format: %u\nFC: %u  Card: %u",
            g_pyr_decoded[0],
            g_pyr_decoded[1],
            (uint16_t)((g_pyr_decoded[2] << 8) | g_pyr_decoded[3]));
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_pyramid = {
    .name = "Pyramid",
    .manufacturer = "Farpointe",
    .data_size = PYRAMID_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)pyramid_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)pyramid_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)pyramid_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)pyramid_render_data,
};
