/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Noralsy protocol decoder
 *
 * Based on the Flipper Zero Noralsy protocol approach.
 * ASK/Manchester modulation, RF/32, 96-bit frame.
 * Preamble: 12 bits = 0xBB0 (0b101110110000)
 * Checksum: XOR of 4-bit nibbles at specific positions
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

static manch_decoder_t g_noralsy_dec;
static uint8_t g_noralsy_decoded[NORALSY_DECODED_SIZE];

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Noralsy: 96-bit frame, Manchester encoded                                  */
/* Preamble: first 12 bits = 0b101110110000 = 0xBB0                          */
/* Checksum: XOR of nibbles at specific bit positions                         */
/*============================================================================*/
static bool noralsy_can_be_decoded(const manch_decoder_t *d)
{
    if (d->bit_count < 96)
        return false;

    /* Check 12-bit preamble: 0xBB0 */
    if (bl_get_bits_16(d->frame_buffer, 0, 12) != 0xBB0)
        return false;

    /* Checksum validation: XOR of 4-bit nibbles at specific positions
     * Two 4-bit checksums at bits 72-75 and 76-79
     * Compared against XOR of nibbles from data positions
     */
    uint8_t cs1 = 0;
    uint8_t cs2 = 0;

    /* Nibble positions for checksum 1 (even nibbles from data) */
    cs1 ^= bl_get_bits(d->frame_buffer, 12, 4);
    cs1 ^= bl_get_bits(d->frame_buffer, 20, 4);
    cs1 ^= bl_get_bits(d->frame_buffer, 28, 4);
    cs1 ^= bl_get_bits(d->frame_buffer, 36, 4);
    cs1 ^= bl_get_bits(d->frame_buffer, 44, 4);
    cs1 ^= bl_get_bits(d->frame_buffer, 52, 4);
    cs1 ^= bl_get_bits(d->frame_buffer, 60, 4);
    cs1 ^= bl_get_bits(d->frame_buffer, 68, 4);

    /* Nibble positions for checksum 2 (odd nibbles from data) */
    cs2 ^= bl_get_bits(d->frame_buffer, 16, 4);
    cs2 ^= bl_get_bits(d->frame_buffer, 24, 4);
    cs2 ^= bl_get_bits(d->frame_buffer, 32, 4);
    cs2 ^= bl_get_bits(d->frame_buffer, 40, 4);
    cs2 ^= bl_get_bits(d->frame_buffer, 48, 4);
    cs2 ^= bl_get_bits(d->frame_buffer, 56, 4);
    cs2 ^= bl_get_bits(d->frame_buffer, 64, 4);

    uint8_t stored_cs1 = bl_get_bits(d->frame_buffer, 72, 4);
    uint8_t stored_cs2 = bl_get_bits(d->frame_buffer, 76, 4);

    if (cs1 != stored_cs1 || cs2 != stored_cs2)
        return false;

    return true;
}

/*============================================================================*/
static void noralsy_begin_impl(void)
{
    manch_init(&g_noralsy_dec, NORALSY_HALF_BIT_US, 96);
    memset(g_noralsy_decoded, 0, sizeof(g_noralsy_decoded));
}

/*============================================================================*/
static bool noralsy_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    manch_feed_events(&g_noralsy_dec, evt, (uint8_t)size);

    if (noralsy_can_be_decoded(&g_noralsy_dec)) {
        /* Copy full frame as decoded data */
        memcpy(g_noralsy_decoded, g_noralsy_dec.frame_buffer, NORALSY_DECODED_SIZE);
        memcpy(lfrfid_tag_info.uid, g_noralsy_decoded,
               min(sizeof(lfrfid_tag_info.uid), NORALSY_DECODED_SIZE));
        lfrfid_tag_info.bitrate = 32;
        return true;
    }

    return false;
}

/*============================================================================*/
static uint8_t *noralsy_get_data(void *proto) { (void)proto; return g_noralsy_decoded; }

/*============================================================================*/
static void noralsy_render_data(void *proto, char *result)
{
    (void)proto;
    sprintf(result,
            "Noralsy\n"
            "Hex: %02X%02X%02X%02X%02X",
            g_noralsy_decoded[0], g_noralsy_decoded[1],
            g_noralsy_decoded[2], g_noralsy_decoded[3],
            g_noralsy_decoded[4]);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_noralsy = {
    .name = "Noralsy",
    .manufacturer = "Noralsy",
    .data_size = NORALSY_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)noralsy_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)noralsy_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)noralsy_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)noralsy_render_data,
};
