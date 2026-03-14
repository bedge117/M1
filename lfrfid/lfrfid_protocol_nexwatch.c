/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Nexwatch protocol decoder
 *
 * Based on the Flipper Zero Nexwatch protocol approach.
 * PSK1 modulation, 255us per bit, 96-bit frame.
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

/***************************** C O N S T A N T S ******************************/

static const uint8_t nexwatch_hex_2_id[] = {
    31, 27, 23, 19, 15, 11, 7, 3, 30, 26, 22, 18, 14, 10, 6, 2,
    29, 25, 21, 17, 13, 9,  5, 1, 28, 24, 20, 16, 12, 8,  4, 0
};

/***************************** V A R I A B L E S ******************************/

/* Four parallel decode buffers for phase ambiguity (Flipper approach) */
static uint8_t g_nex_encoded[NEXWATCH_ENCODED_DATA_SIZE];
static uint8_t g_nex_neg_encoded[NEXWATCH_ENCODED_DATA_SIZE];
static uint8_t g_nex_corrupted[NEXWATCH_ENCODED_DATA_SIZE];
static uint8_t g_nex_neg_corrupted[NEXWATCH_ENCODED_DATA_SIZE];
static uint8_t g_nex_decoded[NEXWATCH_DECODED_SIZE];

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Parity: XOR all nibbles then swap bit positions                            */
/*============================================================================*/
static uint8_t nexwatch_parity_swap(uint8_t p)
{
    return (uint8_t)(((p >> 3) & 1) |
                     (((p >> 1) & 1) << 1) |
                     (((p >> 2) & 1) << 2) |
                     ((p & 1) << 3));
}

static uint8_t nexwatch_parity(const uint8_t hex[5])
{
    uint8_t p = 0;
    for (int i = 0; i < 5; i++) {
        p ^= (hex[i] >> 4) & 0x0F;
        p ^= hex[i] & 0x0F;
    }
    return nexwatch_parity_swap(p);
}

/*============================================================================*/
/* Nexwatch preamble: 0x56 at byte 0, reserved word = 0 at bits 8-39         */
/* Parity check on data nibbles                                               */
/*============================================================================*/
static bool nexwatch_can_be_decoded(const uint8_t *data)
{
    /* Preamble: 0x56 (01010110) */
    if (bl_get_bits(data, 0, 8) != 0x56)
        return false;

    /* Reserved word: bits 8-39 must all be zero */
    if (bl_get_bits_32(data, 8, 32) != 0)
        return false;

    /* Extract 5 hex bytes from bits 40-79 for parity check */
    uint8_t hex[5];
    for (int i = 0; i < 5; i++)
        hex[i] = bl_get_bits(data, 40 + i * 8, 8);
    hex[4] &= 0xF0;

    /* Parity nibble at bits 76-79 */
    uint8_t parity = bl_get_bits(data, 76, 4);
    if (nexwatch_parity(hex) != parity)
        return false;

    return true;
}

/*============================================================================*/
static bool nex_feed_internal(bool polarity, uint32_t time, uint8_t *data)
{
    time += (NEXWATCH_US_PER_BIT / 2);
    size_t bit_count = time / NEXWATCH_US_PER_BIT;

    if (bit_count < NEXWATCH_ENCODED_BIT_SIZE) {
        for (size_t i = 0; i < bit_count; i++) {
            bl_push_bit(data, NEXWATCH_ENCODED_DATA_SIZE, polarity);
            if (nexwatch_can_be_decoded(data))
                return true;
        }
    }
    return false;
}

/*============================================================================*/
/* Reverse a nibble (4 bits)                                                  */
/*============================================================================*/
static uint8_t nexwatch_reverse_nibble(uint8_t n)
{
    return (uint8_t)(((n >> 3) & 1) |
                     (((n >> 2) & 1) << 1) |
                     (((n >> 1) & 1) << 2) |
                     ((n & 1) << 3));
}

/*============================================================================*/
/* Reverse a full byte                                                        */
/*============================================================================*/
static uint8_t nexwatch_reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

/*============================================================================*/
static void nexwatch_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, NEXWATCH_DECODED_SIZE);

    /* Descramble 32-bit ID from bits 40-71 using lookup table */
    uint32_t scrambled = bl_get_bits_32(encoded, 40, 32);
    uint32_t id = 0;
    for (int i = 0; i < 32; i++) {
        if (scrambled & (1UL << (31 - i)))
            id |= (1UL << nexwatch_hex_2_id[i]);
    }

    /* Bytes 0-3: 32-bit descrambled ID (big-endian) */
    decoded[0] = (uint8_t)(id >> 24);
    decoded[1] = (uint8_t)(id >> 16);
    decoded[2] = (uint8_t)(id >> 8);
    decoded[3] = (uint8_t)(id);

    /* Bytes 4-6: 24-bit check from bits 72-95 */
    decoded[4] = bl_get_bits(encoded, 72, 8);
    decoded[5] = bl_get_bits(encoded, 80, 8);
    decoded[6] = bl_get_bits(encoded, 88, 8);

    /* Byte 7: parity nibble from bits 76-79 */
    decoded[7] = bl_get_bits(encoded, 76, 4);
}

/*============================================================================*/
static void nexwatch_begin_impl(void)
{
    memset(g_nex_encoded, 0, sizeof(g_nex_encoded));
    memset(g_nex_neg_encoded, 0, sizeof(g_nex_neg_encoded));
    memset(g_nex_corrupted, 0, sizeof(g_nex_corrupted));
    memset(g_nex_neg_corrupted, 0, sizeof(g_nex_neg_corrupted));
    memset(g_nex_decoded, 0, sizeof(g_nex_decoded));
}

/*============================================================================*/
static bool nexwatch_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        bool level = (evt[i].edge != 0);
        uint32_t duration = evt[i].t_us;

        if (duration <= (NEXWATCH_US_PER_BIT / 2))
            continue;

        if (nex_feed_internal(level, duration, g_nex_encoded)) {
            nexwatch_decode(g_nex_encoded, g_nex_decoded);
            goto detected;
        }
        if (nex_feed_internal(!level, duration, g_nex_neg_encoded)) {
            nexwatch_decode(g_nex_neg_encoded, g_nex_decoded);
            goto detected;
        }

        if (duration > (NEXWATCH_US_PER_BIT / 4)) {
            uint32_t adjusted = duration;
            if (level)
                adjusted += 120;
            else if (adjusted > 120)
                adjusted -= 120;

            if (nex_feed_internal(level, adjusted, g_nex_corrupted)) {
                nexwatch_decode(g_nex_corrupted, g_nex_decoded);
                goto detected;
            }
            if (nex_feed_internal(!level, adjusted, g_nex_neg_corrupted)) {
                nexwatch_decode(g_nex_neg_corrupted, g_nex_decoded);
                goto detected;
            }
        }
    }
    return false;

detected:
    memcpy(lfrfid_tag_info.uid, g_nex_decoded,
           min(sizeof(lfrfid_tag_info.uid), NEXWATCH_DECODED_SIZE));
    return true;
}

/*============================================================================*/
static uint8_t *nexwatch_get_data(void *proto) { (void)proto; return g_nex_decoded; }

/*============================================================================*/
static void nexwatch_render_data(void *proto, char *result)
{
    (void)proto;

    /* Reconstruct 32-bit ID from decoded bytes 0-3 */
    uint32_t id = ((uint32_t)g_nex_decoded[0] << 24) |
                  ((uint32_t)g_nex_decoded[1] << 16) |
                  ((uint32_t)g_nex_decoded[2] << 8) |
                  g_nex_decoded[3];

    /* Detect magic type from checksum */
    uint8_t parity_nibble = g_nex_decoded[7] & 0x0F;
    uint8_t a = (uint8_t)(((id >> 24) & 0xFF) - ((id >> 16) & 0xFF) -
                ((id >> 8) & 0xFF) - (id & 0xFF) -
                0x00 - (nexwatch_reverse_nibble(parity_nibble) >> 4));
    uint8_t magic = nexwatch_reverse_byte(a);

    const char *type_str = "Unknown";
    if (magic == 0xBE) type_str = "Quadrakey";
    else if (magic == 0x88) type_str = "Nexkey";
    else if (magic == 0x86) type_str = "Honeywell";

    sprintf(result,
            "Type: %s\n"
            "ID: %lu\n"
            "Hex: %02X%02X%02X%02X",
            type_str,
            (unsigned long)id,
            g_nex_decoded[0], g_nex_decoded[1],
            g_nex_decoded[2], g_nex_decoded[3]);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_nexwatch = {
    .name = "Nexwatch",
    .manufacturer = "Honeywell",
    .data_size = NEXWATCH_DECODED_SIZE,
    .features = LFRFIDFeaturePSK,
    .get_data = (lfrfidProtocolGetData)nexwatch_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)nexwatch_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)nexwatch_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)nexwatch_render_data,
};
