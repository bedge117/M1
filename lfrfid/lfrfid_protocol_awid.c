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
#include "t5577.h"
#include "bit_lib.h"

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
/* Encode: build the 96-bit FSK2a frame from decoded data                     */
/* Ported bit-exact from Flipper protocol_awid_encode (lib/lfrfid, GPLv3):    */
/*   preamble 0x01, then 22 groups of (3 data bits << 1 | odd parity) => 4b   */
/*============================================================================*/
static void awid_encode(const uint8_t *decoded_data, uint8_t *encoded_data)
{
    memset(encoded_data, 0, AWID_ENCODED_SIZE);

    /* preamble */
    bit_lib_set_bits(encoded_data, 0, 0x01, 8);

    for (size_t i = 0; i < 88 / 4; i++) {
        uint8_t value = (uint8_t)(bit_lib_get_bits(decoded_data, i * 3, 3) << 1);
        value |= bit_lib_test_parity_32(value, BitLibParityOdd);
        bit_lib_set_bits(encoded_data, 8 + i * 4, value, 4);
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
/* T5577 write support — ported from Flipper protocol_awid_write_data /
 * protocol_awid_encode (lib/lfrfid, GPLv3).
 *
 * AWID: FSK2a modulation, RF/50 bitrate, 3 data blocks (block 0 = config,
 * blocks 1..3 = 96-bit encoded frame). The decoded data (9 bytes) is sourced
 * from the g_awid_decoded static buffer — it cannot come from tag->uid because
 * the 66-bit / 9-byte payload does not fit in uid[5]. Before encoding, the
 * length byte is normalised and the payload is round-tripped through
 * encode/parity-strip/decode exactly as Flipper does, so the final T5577 image
 * is bit-exact.
 */
/*============================================================================*/
void protocol_awid_write_begin(void* protocol, void* data)
{
    (void)protocol;
    LFRFIDProgram* write = (LFRFIDProgram*)data;
    uint8_t        encoded[AWID_ENCODED_SIZE];

    memset(encoded, 0, sizeof(encoded));

    /* Fix incorrect length byte */
    if (g_awid_decoded[0] != 26 && g_awid_decoded[0] != 50 && g_awid_decoded[0] != 37 &&
        g_awid_decoded[0] != 34 && g_awid_decoded[0] != 36) {
        g_awid_decoded[0] = 26;
    }

    /* Correct protocol data by redecoding (encode -> strip parity -> decode) */
    awid_encode(g_awid_decoded, encoded);
    bit_lib_remove_bit_every_nth(encoded, 8, 88, 4);
    bit_lib_copy_bits(g_awid_decoded, 0, 66, encoded, 8);

    /* Final encode for T5577 */
    awid_encode(g_awid_decoded, encoded);

    if (write && write->type == LFRFIDProgramTypeT5577) {
        write->t5577.block_data[0] = T5577_MOD_FSK2a | T5577_BITRATE_RF_50 | T5577_TRANS_BL_1_3;
        write->t5577.block_data[1] = bit_lib_get_bits_32(encoded, 0, 32);
        write->t5577.block_data[2] = bit_lib_get_bits_32(encoded, 32, 32);
        write->t5577.block_data[3] = bit_lib_get_bits_32(encoded, 64, 32);
        write->t5577.max_blocks = 4;
    }
}

void protocol_awid_write_send(void* proto)
{
    (void)proto;
    t5577_execute_write(lfrfid_program, 0);
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
    .write = {
        .begin = (lfrfidProtocolWriteBegin)protocol_awid_write_begin,
        .send  = (lfrfidProtocolWriteSend)protocol_awid_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)awid_render_data,
};
