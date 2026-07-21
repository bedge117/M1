/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — IoProx (Kantech ioProx/XSF) protocol decoder
 *
 * Based on the Flipper Zero IoProxXSF protocol.
 * FSK modulation, 64-bit frame.
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
#include "t5577.h"
#include "bit_lib.h"

/***************************** V A R I A B L E S ******************************/

static uint8_t g_ioprox_encoded[IOPROX_ENCODED_SIZE];   /* 8 bytes */
static uint8_t g_ioprox_decoded[IOPROX_DECODED_SIZE];    /* 4 bytes */
static fsk_symbol_state_t g_ioprox_sym_st;
static fsk_bit_state_t    g_ioprox_bit_st;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

static void ioprox_push_bit(uint8_t *buf, size_t len, uint8_t bit)
{
    for (size_t i = 0; i < len - 1; i++)
        buf[i] = (uint8_t)((buf[i] << 1) | (buf[i + 1] >> 7));
    buf[len - 1] = (uint8_t)((buf[len - 1] << 1) | (bit & 1));
}

/*============================================================================*/
/* Flipper-exact IoProx validation                                            */
/* byte[0] = 0x00                                                             */
/* Framing bits: positions 8,17,26,35,44,53 = 1; position 62 = 0             */
/* Checksum: 0xFF - (v[0] + v[1] + v[2] + v[3]) == v[4]                     */
/* Data bytes extracted at 9-bit intervals starting at bit 9                  */
/*============================================================================*/
static bool ioprox_can_be_decoded(const uint8_t *data)
{
    /* byte 0 must be 0x00 */
    if (data[0] != 0x00)
        return false;

    /* Check framing bits at positions 8, 17, 26, 35, 44, 53 = 1; 62 = 0 */
    if (!bl_get_bit(data, 8))  return false;   /* must be 1 */
    if (!bl_get_bit(data, 17)) return false;   /* must be 1 */
    if (!bl_get_bit(data, 26)) return false;   /* must be 1 */
    if (!bl_get_bit(data, 35)) return false;   /* must be 1 */
    if (!bl_get_bit(data, 44)) return false;   /* must be 1 */
    if (!bl_get_bit(data, 53)) return false;   /* must be 1 */
    if (bl_get_bit(data, 62))  return false;   /* must be 0 */

    /* Extract 5 data bytes at 9-bit intervals starting at bit 9 */
    uint8_t v0 = bl_get_bits(data, 9,  8);   /* version */
    uint8_t v1 = bl_get_bits(data, 18, 8);   /* facility code */
    uint8_t v2 = bl_get_bits(data, 27, 8);   /* code byte high */
    uint8_t v3 = bl_get_bits(data, 36, 8);   /* code byte low */
    uint8_t v4 = bl_get_bits(data, 45, 8);   /* checksum field */

    /* Checksum: 0xFF minus sum of first 4 data bytes */
    uint8_t checksum = (uint8_t)(0xFF - (v0 + v1 + v2 + v3));
    if (v4 != checksum)
        return false;

    return true;
}

/*============================================================================*/
/* Decode: extract version, facility, code from 9-bit-spaced data             */
/* decoded[0] = version, decoded[1] = facility,                               */
/* decoded[2] = code_h, decoded[3] = code_l                                   */
/*============================================================================*/
static void ioprox_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, IOPROX_DECODED_SIZE);
    decoded[0] = bl_get_bits(encoded, 9,  8);   /* version */
    decoded[1] = bl_get_bits(encoded, 18, 8);   /* facility */
    decoded[2] = bl_get_bits(encoded, 27, 8);   /* code high */
    decoded[3] = bl_get_bits(encoded, 36, 8);   /* code low */
}

/*============================================================================*/
static void ioprox_begin_impl(void)
{
    memset(g_ioprox_encoded, 0, sizeof(g_ioprox_encoded));
    memset(g_ioprox_decoded, 0, sizeof(g_ioprox_decoded));
    fsk_symbol_state_init(&g_ioprox_sym_st);
    fsk_bit_state_init(&g_ioprox_bit_st);
}

/*============================================================================*/
static bool ioprox_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        uint8_t symbol;
        if (fsk_symbol_feed(&g_ioprox_sym_st, &evt[i], &symbol)) {
            uint8_t bit;
            if (fsk_bit_feed(&g_ioprox_bit_st, symbol, &bit)) {
                ioprox_push_bit(g_ioprox_encoded, IOPROX_ENCODED_SIZE, bit);

                if (ioprox_can_be_decoded(g_ioprox_encoded)) {
                    ioprox_decode(g_ioprox_encoded, g_ioprox_decoded);
                    memcpy(lfrfid_tag_info.uid, g_ioprox_decoded,
                           min(sizeof(lfrfid_tag_info.uid), IOPROX_DECODED_SIZE));
                    return true;
                }
            }
        }
    }
    return false;
}

/*============================================================================*/
static uint8_t *ioprox_get_data(void *proto) { (void)proto; return g_ioprox_decoded; }

/*============================================================================*/
static void ioprox_render_data(void *proto, char *result)
{
    (void)proto;
    uint8_t version = g_ioprox_decoded[0];
    uint8_t fc = g_ioprox_decoded[1];
    uint16_t cn = (uint16_t)((g_ioprox_decoded[2] << 8) | g_ioprox_decoded[3]);
    sprintf(result,
            "V%u  FC: %u  Card: %u\n"
            "Hex: %02X%02X%02X%02X",
            version, fc, cn,
            g_ioprox_decoded[0], g_ioprox_decoded[1],
            g_ioprox_decoded[2], g_ioprox_decoded[3]);
}

/*============================================================================*/
/* T5577 write support — modulation/bitrate/block layout ported from Flipper
 * protocol_io_prox_xsf_write_data (FSK2a, RF/64, 2 data blocks, GPLv3).
 *
 * The 64-bit encoded frame is NOT Flipper's standard IoProx framing: it is the
 * EXACT INVERSE of THIS file's ioprox_decode / ioprox_can_be_decoded, so that a
 * card read by the M1 decoder and written back here reads back identically on
 * M1. Frame layout the M1 decoder expects (bit offsets):
 *
 *   [0..7]   = 0x00                (data[0] == 0x00)
 *   [8]      = 1                   (framing)
 *   [9..16]  = uid[0]  version     (ioprox_decode reads bits 9)
 *   [17]     = 1                   (framing)
 *   [18..25] = uid[1]  facility    (reads bits 18)
 *   [26]     = 1                   (framing)
 *   [27..34] = uid[2]  code hi     (reads bits 27)
 *   [35]     = 1                   (framing)
 *   [36..43] = uid[3]  code lo     (reads bits 36)
 *   [44]     = 1                   (framing)
 *   [45..52] = checksum = 0xFF - (uid[0]+uid[1]+uid[2]+uid[3])   (validated at bit 45)
 *   [53]     = 1                   (framing)
 *   [62]     = 0                   (required 0)
 *   remaining bits (54..61, 63) = 0
 *
 * Decoded data (4 bytes) is sourced from tag->uid in M1's stored order, which
 * holds it verbatim for protocols <= 5 bytes.
 */
/*============================================================================*/
void protocol_ioprox_write_begin(void* protocol, void* data)
{
    LFRFID_TAG_INFO* tag_data = (LFRFID_TAG_INFO*)protocol;
    LFRFIDProgram*   write    = (LFRFIDProgram*)data;
    const uint8_t*   decoded  = tag_data->uid;
    uint8_t          encoded[IOPROX_ENCODED_SIZE];

    memset(encoded, 0, sizeof(encoded));

    /* byte 0 (bits 0..7) stays 0x00 from memset; framing bit at 8 = 1 */
    bit_lib_set_bit(encoded, 8, 1);

    /* Version (decode reads bits 9). */
    bit_lib_set_bits(encoded, 9, decoded[0], 8);
    bit_lib_set_bit(encoded, 17, 1);

    /* Facility (decode reads bits 18). */
    bit_lib_set_bits(encoded, 18, decoded[1], 8);
    bit_lib_set_bit(encoded, 26, 1);

    /* Code high (decode reads bits 27). */
    bit_lib_set_bits(encoded, 27, decoded[2], 8);
    bit_lib_set_bit(encoded, 35, 1);

    /* Code low (decode reads bits 36). */
    bit_lib_set_bits(encoded, 36, decoded[3], 8);
    bit_lib_set_bit(encoded, 44, 1);

    /* Checksum (decode validates at bits 45): 0xFF - sum of the 4 data bytes. */
    uint8_t checksum = (uint8_t)(0xFF - (decoded[0] + decoded[1] + decoded[2] + decoded[3]));
    bit_lib_set_bits(encoded, 45, checksum, 8);
    bit_lib_set_bit(encoded, 53, 1);

    /* bit 62 must be 0 (already 0 from memset). */

    if(write && write->type == LFRFIDProgramTypeT5577) {
        write->t5577.block_data[0] = T5577_MOD_FSK2a | T5577_BITRATE_RF_64 | T5577_TRANS_BL_1_2;
        write->t5577.block_data[1] = bit_lib_get_bits_32(encoded, 0, 32);
        write->t5577.block_data[2] = bit_lib_get_bits_32(encoded, 32, 32);
        write->t5577.max_blocks = 3;
    }
}

void protocol_ioprox_write_send(void* proto)
{
    (void)proto;
    t5577_execute_write(lfrfid_program, 0);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_ioprox = {
    .name = "IoProxXSF",
    .manufacturer = "Kantech",
    .data_size = IOPROX_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)ioprox_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)ioprox_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)ioprox_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = {
        .begin = (lfrfidProtocolWriteBegin)protocol_ioprox_write_begin,
        .send  = (lfrfidProtocolWriteSend)protocol_ioprox_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)ioprox_render_data,
};
