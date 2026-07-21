/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Viking protocol decoder
 *
 * Based on the Flipper Zero Viking protocol approach.
 * ASK/Manchester modulation, RF/32, 88-bit frame (64 encoded + 24 overlap).
 * Preamble: 0xF200 at bit 0 and bit 64 (16-bit at both positions)
 * Checksum: XOR of 8 bytes from bit 24 to bit 87 must equal 0xA8
 *
 * Copyright (C) Flipper Devices Inc. (GPLv3)
 * Modifications: Copyright (C) 2026 Monstatek
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
#include "lfrfid_manchester.h"
#include "lfrfid_bit_lib.h"
#include "t5577.h"
#include "bit_lib.h"

/***************************** V A R I A B L E S ******************************/

static manch_decoder_t g_viking_dec;
static uint8_t g_viking_decoded[VIKING_DECODED_SIZE];

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Viking validation: Flipper-exact preamble + XOR checksum                   */
/*============================================================================*/
static bool viking_can_be_decoded(const uint8_t *data)
{
    /* Check 16-bit preamble at bit 0 and bit 64 */
    if (bl_get_bits_16(data, 0, 16) != 0xF200)
        return false;
    if (bl_get_bits_16(data, 64, 16) != 0xF200)
        return false;

    /* XOR checksum: all 8 bytes from bit 24 to bit 87, XOR'd together,
     * must equal 0xA8 */
    uint8_t chk = 0;
    for (int i = 0; i < 8; i++)
        chk ^= bl_get_bits(data, 24 + i * 8, 8);
    if (chk != 0xA8)
        return false;

    return true;
}

/*============================================================================*/
static void viking_decode(const uint8_t *data, uint8_t *decoded)
{
    /* Card data is 4 bytes from bit 24 to bit 55 */
    for (int i = 0; i < VIKING_DECODED_SIZE; i++)
        decoded[i] = bl_get_bits(data, 24 + i * 8, 8);
}

/*============================================================================*/
static void viking_begin_impl(void)
{
    manch_init(&g_viking_dec, VIKING_HALF_BIT_US, VIKING_ENCODED_SIZE * 8);
    memset(g_viking_decoded, 0, sizeof(g_viking_decoded));
}

/*============================================================================*/
static bool viking_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    manch_feed_events(&g_viking_dec, evt, (uint8_t)size);

    if (manch_is_full(&g_viking_dec) &&
        viking_can_be_decoded(g_viking_dec.frame_buffer)) {
        viking_decode(g_viking_dec.frame_buffer, g_viking_decoded);
        memcpy(lfrfid_tag_info.uid, g_viking_decoded,
               min(sizeof(lfrfid_tag_info.uid), VIKING_DECODED_SIZE));
        lfrfid_tag_info.bitrate = 32;
        return true;
    }

    return false;
}

/*============================================================================*/
static uint8_t *viking_get_data(void *proto) { (void)proto; return g_viking_decoded; }

/*============================================================================*/
static void viking_render_data(void *proto, char *result)
{
    (void)proto;
    uint32_t card_id = ((uint32_t)g_viking_decoded[0] << 24) |
                       ((uint32_t)g_viking_decoded[1] << 16) |
                       ((uint32_t)g_viking_decoded[2] << 8) |
                       g_viking_decoded[3];
    sprintf(result,
            "Card: %lu\n"
            "Hex: %02X%02X%02X%02X",
            (unsigned long)card_id,
            g_viking_decoded[0], g_viking_decoded[1],
            g_viking_decoded[2], g_viking_decoded[3]);
}

/*============================================================================*/
/* T5577 write support — ported from Flipper protocol_viking_write_data /
 * protocol_viking_encoder_start (lib/lfrfid, GPLv3).
 *
 * Viking: Manchester modulation, RF/32 bitrate, 2 data blocks. The 64-bit
 * encoded frame is a 24-bit preamble (0xF2 0x00 0x00) + 32-bit card id at
 * bit 24 + 8-bit XOR checksum at bit 56. Decoded data (4 bytes) is sourced
 * from tag->uid, which holds the card id verbatim for protocols <= 5 bytes.
 */
/*============================================================================*/
void protocol_viking_write_begin(void* protocol, void* data)
{
    LFRFID_TAG_INFO* tag_data = (LFRFID_TAG_INFO*)protocol;
    LFRFIDProgram*   write    = (LFRFIDProgram*)data;
    const uint8_t*   decoded  = tag_data->uid;
    uint8_t          encoded[VIKING_ENCODED_SIZE];

    memset(encoded, 0, sizeof(encoded));

    /* Preamble: 0xF2 0x00 0x00 */
    bit_lib_set_bits(encoded, 0, 0b11110010, 8);
    bit_lib_set_bits(encoded, 8, 0b00000000, 8);
    bit_lib_set_bits(encoded, 16, 0b00000000, 8);

    /* Card id */
    bit_lib_copy_bits(encoded, 24, 32, decoded, 0);

    /* Checksum */
    uint32_t id = bit_lib_get_bits_32(decoded, 0, 32);
    uint8_t  checksum = ((id >> 24) & 0xFF) ^ ((id >> 16) & 0xFF) ^ ((id >> 8) & 0xFF) ^
                        (id & 0xFF) ^ 0xF2 ^ 0xA8;
    bit_lib_set_bits(encoded, 56, checksum, 8);

    if(write && write->type == LFRFIDProgramTypeT5577) {
        write->t5577.block_data[0] =
            T5577_MOD_MANCHESTER | T5577_BITRATE_RF_32 | T5577_TRANS_BL_1_2;
        write->t5577.block_data[1] = bit_lib_get_bits_32(encoded, 0, 32);
        write->t5577.block_data[2] = bit_lib_get_bits_32(encoded, 32, 32);
        write->t5577.max_blocks = 3;
    }
}

void protocol_viking_write_send(void* proto)
{
    (void)proto;
    t5577_execute_write(lfrfid_program, 0);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_viking = {
    .name = "Viking",
    .manufacturer = "Viking",
    .data_size = VIKING_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)viking_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)viking_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)viking_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write =
    {
        .begin = (lfrfidProtocolWriteBegin)protocol_viking_write_begin,
        .send  = (lfrfidProtocolWriteSend)protocol_viking_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)viking_render_data,
};
