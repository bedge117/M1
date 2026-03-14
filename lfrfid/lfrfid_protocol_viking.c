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
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)viking_render_data,
};
