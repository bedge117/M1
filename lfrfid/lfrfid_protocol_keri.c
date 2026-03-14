/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Keri protocol decoder
 *
 * Based on the Flipper Zero Keri protocol approach.
 * PSK1 modulation, 255us per bit, 64-bit frame.
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

/* Four parallel decode buffers for phase ambiguity (Flipper approach) */
static uint8_t g_keri_encoded[KERI_ENCODED_DATA_SIZE];
static uint8_t g_keri_neg_encoded[KERI_ENCODED_DATA_SIZE];
static uint8_t g_keri_corrupted[KERI_ENCODED_DATA_SIZE];
static uint8_t g_keri_neg_corrupted[KERI_ENCODED_DATA_SIZE];
static uint8_t g_keri_decoded[KERI_DECODED_SIZE];

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

static inline void keri_bit_push(uint8_t *data, size_t data_size, bool bit)
{
    for (size_t i = 0; i < data_size - 1; i++)
        data[i] = (uint8_t)((data[i] << 1) | (data[i + 1] >> 7));
    data[data_size - 1] = (uint8_t)((data[data_size - 1] << 1) | (bit ? 1 : 0));
}

static inline bool keri_bit_get(const uint8_t *data, size_t bit_index)
{
    return (data[bit_index / 8] >> (7 - (bit_index % 8))) & 1;
}

/*============================================================================*/
/* Keri preamble: 11100000 00000001 (16 bits) at start                       */
/*============================================================================*/
static bool keri_can_be_decoded(const uint8_t *data)
{
    /* Check preamble: E001 pattern at bit offset 0 */
    if (data[0] != 0xE0)
        return false;
    if (data[1] != 0x01)
        return false;
    /* Verify frame wraps: preamble also at bit 64 */
    if (data[8] != 0xE0)
        return false;
    return true;
}

/*============================================================================*/
static bool keri_feed_internal(bool polarity, uint32_t time, uint8_t *data)
{
    time += (KERI_US_PER_BIT / 2);
    size_t bit_count = time / KERI_US_PER_BIT;

    if (bit_count < KERI_ENCODED_BIT_SIZE) {
        for (size_t i = 0; i < bit_count; i++) {
            keri_bit_push(data, KERI_ENCODED_DATA_SIZE, polarity);
            if (keri_can_be_decoded(data))
                return true;
        }
    }
    return false;
}

/*============================================================================*/
static void keri_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, KERI_DECODED_SIZE);
    /* Extract 32-bit internal ID from encoded data */
    decoded[0] = encoded[2];
    decoded[1] = encoded[3];
    decoded[2] = encoded[4];
    decoded[3] = encoded[5];
}

/*============================================================================*/
static void keri_begin_impl(void)
{
    memset(g_keri_encoded, 0, sizeof(g_keri_encoded));
    memset(g_keri_neg_encoded, 0, sizeof(g_keri_neg_encoded));
    memset(g_keri_corrupted, 0, sizeof(g_keri_corrupted));
    memset(g_keri_neg_corrupted, 0, sizeof(g_keri_neg_corrupted));
    memset(g_keri_decoded, 0, sizeof(g_keri_decoded));
}

/*============================================================================*/
static bool keri_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        bool level = (evt[i].edge != 0);
        uint32_t duration = evt[i].t_us;

        if (duration <= (KERI_US_PER_BIT / 2))
            continue;

        if (keri_feed_internal(level, duration, g_keri_encoded)) {
            keri_decode(g_keri_encoded, g_keri_decoded);
            goto detected;
        }
        if (keri_feed_internal(!level, duration, g_keri_neg_encoded)) {
            keri_decode(g_keri_neg_encoded, g_keri_decoded);
            goto detected;
        }

        if (duration > (KERI_US_PER_BIT / 4)) {
            uint32_t adjusted = duration;
            if (level)
                adjusted += 120;
            else if (adjusted > 120)
                adjusted -= 120;

            if (keri_feed_internal(level, adjusted, g_keri_corrupted)) {
                keri_decode(g_keri_corrupted, g_keri_decoded);
                goto detected;
            }
            if (keri_feed_internal(!level, adjusted, g_keri_neg_corrupted)) {
                keri_decode(g_keri_neg_corrupted, g_keri_decoded);
                goto detected;
            }
        }
    }
    return false;

detected:
    memcpy(lfrfid_tag_info.uid, g_keri_decoded,
           min(sizeof(lfrfid_tag_info.uid), KERI_DECODED_SIZE));
    return true;
}

/*============================================================================*/
static uint8_t *keri_get_data(void *proto) { (void)proto; return g_keri_decoded; }

/*============================================================================*/
static void keri_render_data(void *proto, char *result)
{
    (void)proto;
    uint32_t internal_id = ((uint32_t)g_keri_decoded[0] << 24) |
                           ((uint32_t)g_keri_decoded[1] << 16) |
                           ((uint32_t)g_keri_decoded[2] << 8) |
                           g_keri_decoded[3];
    sprintf(result,
            "ID: %lu\n"
            "Hex: %02X%02X%02X%02X",
            (unsigned long)internal_id,
            g_keri_decoded[0], g_keri_decoded[1],
            g_keri_decoded[2], g_keri_decoded[3]);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_keri = {
    .name = "Keri",
    .manufacturer = "Keri",
    .data_size = KERI_DECODED_SIZE,
    .features = LFRFIDFeaturePSK,
    .get_data = (lfrfidProtocolGetData)keri_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)keri_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)keri_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)keri_render_data,
};
