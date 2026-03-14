/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — GProxII (Guardall) protocol decoder
 *
 * Based on the Flipper Zero GProxII protocol approach.
 * ASK/BI-PHASE inverted modulation, RF/64, 96-bit frame.
 * Bi-phase inverted: short=bit 1, long=bit 0
 * Preamble: first 6 bits = 0b111110
 * After preamble: parity on every 5th bit (remove every 5th)
 * XOR bytes 1-8 against byte 0
 * Card length must be 26 or 36 bits
 * Wiegand parity check on card data
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

/***************************** V A R I A B L E S ******************************/

static uint8_t g_gproxii_encoded[GPROXII_ENCODED_SIZE];
static uint8_t g_gproxii_decoded[GPROXII_DECODED_SIZE];
static bool    g_gproxii_last_short;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Wiegand parity check for card data                                         */
/*============================================================================*/
static bool gproxii_wiegand_parity(uint64_t card_data, uint8_t card_len)
{
    if (card_len == 26) {
        /* Even parity on bits 25..14, odd parity on bits 12..1 */
        uint32_t even_bits = (uint32_t)((card_data >> 13) & 0x1FFF);
        uint32_t odd_bits  = (uint32_t)(card_data & 0x1FFF);
        bool ep = bl_test_parity_32(even_bits, 13);
        bool op = bl_test_parity_32(odd_bits, 13);
        return (!ep && op);
    } else if (card_len == 36) {
        /* Even parity on bits 35..19, odd parity on bits 17..1 */
        uint32_t even_bits = (uint32_t)((card_data >> 18) & 0x3FFFF);
        uint32_t odd_bits  = (uint32_t)(card_data & 0x3FFFF);
        bool ep = bl_test_parity_32(even_bits, 18);
        bool op = bl_test_parity_32(odd_bits, 18);
        return (!ep && op);
    }
    return false;
}

/*============================================================================*/
/* GProxII validation: Flipper-exact preamble + XOR + parity                  */
/*============================================================================*/
static bool gproxii_can_be_decoded(const uint8_t *data)
{
    /* Preamble: first 6 bits must be 0b111110 */
    if (bl_get_bits(data, 0, 6) != 0x3E)
        return false;

    /* Remove parity bits (every 5th bit after preamble) to get clean data.
     * After the 6-bit preamble, there are 90 data+parity bits.
     * Every 5th bit is parity, so 90 / 5 = 18 parity bits removed,
     * leaving 72 data bits = 9 bytes. */
    uint8_t clean[12];
    memset(clean, 0, sizeof(clean));

    /* Copy bits starting after preamble */
    bl_copy_bits(clean, 0, 90, data, 6);

    /* Verify and remove every 5th bit (parity) */
    for (int group = 0; group < 18; group++) {
        /* Each group is 5 bits: 4 data + 1 parity */
        size_t base = (size_t)(group * 5);
        uint8_t parity = 0;
        for (int j = 0; j < 5; j++)
            parity ^= bl_get_bit(clean, base + (size_t)j) ? 1 : 0;
        if (parity != 0)
            return false;
    }

    /* Remove parity bits to get 72 clean bits */
    uint8_t stripped[9];
    memset(stripped, 0, sizeof(stripped));
    size_t dst_bit = 0;
    for (int group = 0; group < 18; group++) {
        size_t base = (size_t)(group * 5);
        for (int j = 0; j < 4; j++) {
            bl_set_bit(stripped, dst_bit, bl_get_bit(clean, base + (size_t)j));
            dst_bit++;
        }
    }

    /* XOR bytes 1-8 against byte 0 */
    uint8_t xor_val = stripped[0];
    uint8_t result[9];
    result[0] = stripped[0];
    for (int i = 1; i < 9; i++)
        result[i] = stripped[i] ^ xor_val;

    /* Card length is in first byte after XOR: must be 26 or 36 */
    uint8_t card_len = result[1];
    if (card_len != 26 && card_len != 36)
        return false;

    /* Extract card data bits: bytes 2+ contain the card data */
    uint64_t card_data = 0;
    for (int i = 0; i < card_len && i < 56; i++) {
        bool bit = (result[2 + i / 8] >> (7 - (i % 8))) & 1;
        card_data = (card_data << 1) | (bit ? 1 : 0);
    }

    /* Wiegand parity check */
    if (!gproxii_wiegand_parity(card_data, card_len))
        return false;

    /* Validation passed — store result into decoded buffer */
    memcpy((void *)g_gproxii_decoded, result, min(sizeof(g_gproxii_decoded), sizeof(result)));

    return true;
}

/*============================================================================*/
static void gproxii_begin_impl(void)
{
    memset(g_gproxii_encoded, 0, sizeof(g_gproxii_encoded));
    memset(g_gproxii_decoded, 0, sizeof(g_gproxii_decoded));
    g_gproxii_last_short = false;
}

/*============================================================================*/
/* Bi-phase inverted decoder: short=bit 1, long=bit 0                         */
/*============================================================================*/
static bool gproxii_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        uint16_t dur = evt[i].t_us;

        if (dur >= (GPROXII_SHORT_US - GPROXII_JITTER_US) &&
            dur <= (GPROXII_SHORT_US + GPROXII_JITTER_US)) {
            /* SHORT pulse */
            if (!g_gproxii_last_short) {
                g_gproxii_last_short = true;
            } else {
                /* Two consecutive shorts = bit 1 (inverted bi-phase) */
                bl_push_bit(g_gproxii_encoded, GPROXII_ENCODED_SIZE, true);
                g_gproxii_last_short = false;

                if (gproxii_can_be_decoded(g_gproxii_encoded)) {
                    memcpy(lfrfid_tag_info.uid, g_gproxii_decoded,
                           min(sizeof(lfrfid_tag_info.uid), GPROXII_DECODED_SIZE));
                    lfrfid_tag_info.bitrate = 64;
                    return true;
                }
            }
        } else if (dur >= (GPROXII_LONG_US - GPROXII_JITTER_US) &&
                   dur <= (GPROXII_LONG_US + GPROXII_JITTER_US)) {
            /* LONG pulse */
            if (!g_gproxii_last_short) {
                /* One long = bit 0 (inverted bi-phase) */
                bl_push_bit(g_gproxii_encoded, GPROXII_ENCODED_SIZE, false);

                if (gproxii_can_be_decoded(g_gproxii_encoded)) {
                    memcpy(lfrfid_tag_info.uid, g_gproxii_decoded,
                           min(sizeof(lfrfid_tag_info.uid), GPROXII_DECODED_SIZE));
                    lfrfid_tag_info.bitrate = 64;
                    return true;
                }
            } else {
                /* Long after a short = reset short tracking */
                g_gproxii_last_short = false;
            }
        } else {
            /* Out of range — reset short tracking */
            g_gproxii_last_short = false;
        }
    }

    return false;
}

/*============================================================================*/
static uint8_t *gproxii_get_data(void *proto) { (void)proto; return g_gproxii_decoded; }

/*============================================================================*/
static void gproxii_render_data(void *proto, char *result)
{
    (void)proto;
    uint8_t card_len = g_gproxii_decoded[1];
    /* Extract FC and Card from Wiegand data */
    if (card_len == 26) {
        uint32_t raw = ((uint32_t)g_gproxii_decoded[2] << 24) |
                       ((uint32_t)g_gproxii_decoded[3] << 16) |
                       ((uint32_t)g_gproxii_decoded[4] << 8) |
                       g_gproxii_decoded[5];
        raw >>= 6; /* right-align 26 bits */
        uint16_t fc = (uint16_t)((raw >> 17) & 0xFF);
        uint16_t card = (uint16_t)((raw >> 1) & 0xFFFF);
        sprintf(result, "FC:%u Card:%u\nLen:26", fc, card);
    } else if (card_len == 36) {
        sprintf(result,
                "GProxII 36-bit\n"
                "Hex: %02X%02X%02X%02X%02X",
                g_gproxii_decoded[2], g_gproxii_decoded[3],
                g_gproxii_decoded[4], g_gproxii_decoded[5],
                g_gproxii_decoded[6]);
    } else {
        sprintf(result,
                "GProxII\n"
                "Hex: %02X%02X%02X%02X",
                g_gproxii_decoded[0], g_gproxii_decoded[1],
                g_gproxii_decoded[2], g_gproxii_decoded[3]);
    }
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_gproxii = {
    .name = "GProxII",
    .manufacturer = "Guardall",
    .data_size = GPROXII_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)gproxii_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)gproxii_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)gproxii_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)gproxii_render_data,
};
