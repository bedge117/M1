/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Idteck protocol decoder
 *
 * Based on the Flipper Zero Idteck protocol approach.
 * PSK1 modulation, 255us per bit, 64-bit frame.
 * Preamble: 0x49 0x44 0x54 0x4B ("IDTK")
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
#include "lfrfid_bit_lib.h"
#include "t5577.h"
#include "bit_lib.h"

/***************************** V A R I A B L E S ******************************/

/* Four parallel decode buffers for phase ambiguity (Flipper approach) */
static uint8_t g_idteck_encoded[IDTECK_ENCODED_SIZE];
static uint8_t g_idteck_neg_encoded[IDTECK_ENCODED_SIZE];
static uint8_t g_idteck_corrupted[IDTECK_ENCODED_SIZE];
static uint8_t g_idteck_neg_corrupted[IDTECK_ENCODED_SIZE];
static uint8_t g_idteck_decoded[IDTECK_DECODED_SIZE];

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Idteck preamble: 0x49 0x44 0x54 0x4B ("IDTK") in MSB-first byte order    */
/*============================================================================*/
static bool idteck_can_be_decoded(const uint8_t *data)
{
    if (data[0] != 0x49)
        return false;
    if (data[1] != 0x44)
        return false;
    if (data[2] != 0x54)
        return false;
    if (data[3] != 0x4B)
        return false;
    return true;
}

/*============================================================================*/
static bool idteck_feed_internal(bool polarity, uint32_t time, uint8_t *data)
{
    time += (IDTECK_US_PER_BIT / 2);
    size_t bit_count = time / IDTECK_US_PER_BIT;

    if (bit_count < IDTECK_ENCODED_BIT_SIZE) {
        for (size_t i = 0; i < bit_count; i++) {
            bl_push_bit(data, IDTECK_ENCODED_SIZE, polarity);
            if (idteck_can_be_decoded(data))
                return true;
        }
    }
    return false;
}

/*============================================================================*/
static void idteck_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, IDTECK_DECODED_SIZE);
    /* Data follows the 4-byte preamble: bytes 4-11 */
    memcpy(decoded, &encoded[4], IDTECK_DECODED_SIZE);
}

/*============================================================================*/
static void idteck_begin_impl(void)
{
    memset(g_idteck_encoded, 0, sizeof(g_idteck_encoded));
    memset(g_idteck_neg_encoded, 0, sizeof(g_idteck_neg_encoded));
    memset(g_idteck_corrupted, 0, sizeof(g_idteck_corrupted));
    memset(g_idteck_neg_corrupted, 0, sizeof(g_idteck_neg_corrupted));
    memset(g_idteck_decoded, 0, sizeof(g_idteck_decoded));
}

/*============================================================================*/
static bool idteck_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        bool level = (evt[i].edge != 0);
        uint32_t duration = evt[i].t_us;

        if (duration <= (IDTECK_US_PER_BIT / 2))
            continue;

        if (idteck_feed_internal(level, duration, g_idteck_encoded)) {
            idteck_decode(g_idteck_encoded, g_idteck_decoded);
            goto detected;
        }
        if (idteck_feed_internal(!level, duration, g_idteck_neg_encoded)) {
            idteck_decode(g_idteck_neg_encoded, g_idteck_decoded);
            goto detected;
        }

        if (duration > (IDTECK_US_PER_BIT / 4)) {
            uint32_t adjusted = duration;
            if (level)
                adjusted += 120;
            else if (adjusted > 120)
                adjusted -= 120;

            if (idteck_feed_internal(level, adjusted, g_idteck_corrupted)) {
                idteck_decode(g_idteck_corrupted, g_idteck_decoded);
                goto detected;
            }
            if (idteck_feed_internal(!level, adjusted, g_idteck_neg_corrupted)) {
                idteck_decode(g_idteck_neg_corrupted, g_idteck_decoded);
                goto detected;
            }
        }
    }
    return false;

detected:
    memcpy(lfrfid_tag_info.uid, g_idteck_decoded,
           min(sizeof(lfrfid_tag_info.uid), IDTECK_DECODED_SIZE));
    return true;
}

/*============================================================================*/
static uint8_t *idteck_get_data(void *proto) { (void)proto; return g_idteck_decoded; }

/*============================================================================*/
static void idteck_render_data(void *proto, char *result)
{
    (void)proto;
    sprintf(result,
            "Hex: %02X%02X%02X%02X %02X%02X%02X%02X",
            g_idteck_decoded[0], g_idteck_decoded[1],
            g_idteck_decoded[2], g_idteck_decoded[3],
            g_idteck_decoded[4], g_idteck_decoded[5],
            g_idteck_decoded[6], g_idteck_decoded[7]);
}

/*============================================================================*/
/* T5577 write support — ported from Flipper protocol_idteck_write_data /
 * protocol_idteck_encoder_start (lib/lfrfid, GPLv3).
 *
 * Idteck: PSK1 modulation, RF/32 bitrate, 2 data blocks. The 64-bit encoded
 * frame is the 32-bit "IDTK" preamble (0x49 0x44 0x54 0x4B) at bits 0..31 and
 * the 32-bit card payload at bits 32..63.
 *
 * DATA SOURCE: Idteck decoded is 8 bytes and does not fit tag->uid[5], so the
 * card payload is sourced from the module-local g_idteck_decoded buffer (the
 * same buffer get_data() returns). Its layout differs from Flipper's
 * protocol->data: Flipper stores [preamble(4)][card(4)] (card at bits 32..63),
 * while M1's decoder stores [card(4)][preamble(4)] (card at bits 0..31 —
 * idteck_decode copies encoded[4..11], i.e. the payload then the next repeated
 * preamble). The card is therefore read from g_idteck_decoded bits 0..31.
 *
 * Config word matches Flipper bit-exactly; all fields have named M1 constants.
 */
/*============================================================================*/
void protocol_idteck_write_begin(void* protocol, void* data)
{
    (void)protocol; /* Idteck decoded (8 bytes) does not fit tag->uid; use g_idteck_decoded */
    LFRFIDProgram* write   = (LFRFIDProgram*)data;
    const uint8_t* decoded = g_idteck_decoded;
    uint8_t        encoded[IDTECK_ENCODED_SIZE];

    memset(encoded, 0, sizeof(encoded));
    /* Preamble "IDTK" = 0x49 0x44 0x54 0x4B at bits 0..31 */
    encoded[0] = 0x49;
    encoded[1] = 0x44;
    encoded[2] = 0x54;
    encoded[3] = 0x4B;
    /* Card payload lives at g_idteck_decoded bits 0..31; place it at bits 32..63 */
    bit_lib_copy_bits(encoded, 32, 32, decoded, 0);

    if(write && write->type == LFRFIDProgramTypeT5577) {
        write->t5577.block_data[0] = T5577_MOD_PSK1 | T5577_BITRATE_RF_32 | T5577_TRANS_BL_1_2;
        write->t5577.block_data[1] = bit_lib_get_bits_32(encoded, 0, 32);
        write->t5577.block_data[2] = bit_lib_get_bits_32(encoded, 32, 32);
        write->t5577.max_blocks = 3;
    }
}

void protocol_idteck_write_send(void* proto)
{
    (void)proto;
    t5577_execute_write(lfrfid_program, 0);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_idteck = {
    .name = "Idteck",
    .manufacturer = "Idteck",
    .data_size = IDTECK_DECODED_SIZE,
    .features = LFRFIDFeaturePSK,
    .get_data = (lfrfidProtocolGetData)idteck_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)idteck_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)idteck_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = {
        .begin = (lfrfidProtocolWriteBegin)protocol_idteck_write_begin,
        .send  = (lfrfidProtocolWriteSend)protocol_idteck_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)idteck_render_data,
};
