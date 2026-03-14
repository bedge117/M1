/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Indala 26-bit PSK protocol
 *
 * Based on the Flipper Zero Indala26 protocol approach.
 * PSK1 modulation, 255us per bit, 64-bit encoded frame.
 * Preamble: 10100000 00000000 00000000 00000000 1
 *
 * Original project:
 * https://github.com/flipperdevices/flipperzero-firmware
 *
 * Copyright (C) Flipper Devices Inc.
 * Licensed under the GNU General Public License v3.0 (GPLv3).
 *
 * Modifications and additional implementation:
 * Copyright (C) 2026 Monstatek
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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

/*************************** D E F I N E S ************************************/

#define INDALA26_PREAMBLE_BIT_SIZE  33
#define INDALA26_DATA_BYTES         ((INDALA26_ENCODED_BIT_SIZE) / 8)  /* 8 */

/***************************** V A R I A B L E S ******************************/

/* Four parallel decode buffers to handle phase ambiguity (Flipper approach) */
static uint8_t g_indala_encoded[INDALA26_ENCODED_DATA_SIZE];
static uint8_t g_indala_neg_encoded[INDALA26_ENCODED_DATA_SIZE];
static uint8_t g_indala_corrupted[INDALA26_ENCODED_DATA_SIZE];
static uint8_t g_indala_neg_corrupted[INDALA26_ENCODED_DATA_SIZE];

static uint8_t g_indala_decoded[INDALA26_DECODED_DATA_SIZE];

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void    indala26_decoder_begin_impl(void);
static bool    indala26_decoder_execute_impl(void *proto, uint16_t size);
static uint8_t *indala26_get_data(void *proto);
static void    indala26_render_data(void *proto, char *result);

/************************* B I T   H E L P E R S *****************************/

static inline void bit_push(uint8_t *data, size_t data_size, bool bit)
{
    for (size_t i = 0; i < data_size - 1; i++)
        data[i] = (uint8_t)((data[i] << 1) | (data[i + 1] >> 7));
    data[data_size - 1] = (uint8_t)((data[data_size - 1] << 1) | (bit ? 1 : 0));
}

static inline bool bit_get(const uint8_t *data, size_t bit_index)
{
    return (data[bit_index / 8] >> (7 - (bit_index % 8))) & 1;
}

static inline void bit_set(uint8_t *data, size_t bit_index, bool value)
{
    if (value)
        data[bit_index / 8] |= (uint8_t)(1 << (7 - (bit_index % 8)));
    else
        data[bit_index / 8] &= (uint8_t)~(1 << (7 - (bit_index % 8)));
}

static inline void bit_copy(uint8_t *dst, size_t dst_bit,
                             const uint8_t *src, size_t src_bit, size_t count)
{
    for (size_t i = 0; i < count; i++)
        bit_set(dst, dst_bit + i, bit_get(src, src_bit + i));
}

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
 * @brief Check preamble at a given bit offset
 * Preamble: 10100000 00000000 00000000 00000000 1 (33 bits)
 */
/*============================================================================*/
static bool indala26_check_preamble(const uint8_t *data, size_t bit_offset)
{
    /* Byte-aligned check for the first 32 bits of preamble */
    size_t byte_idx = bit_offset / 8;
    if (*(uint32_t *)&data[byte_idx] != 0x000000A0u)
        return false;
    /* Bit 32 of preamble must be 1 */
    if (!bit_get(data, bit_offset + 32))
        return false;
    return true;
}

/*============================================================================*/
/**
 * @brief Check if encoded data contains valid Indala26 frame
 *
 * Requires preamble at offset 0 and offset 64, plus specific zero bits.
 */
/*============================================================================*/
static bool indala26_can_be_decoded(const uint8_t *data)
{
    if (!indala26_check_preamble(data, 0))
        return false;
    if (!indala26_check_preamble(data, 64))
        return false;
    /* Bits 60 and 61 must be 0 */
    if (bit_get(data, 61))
        return false;
    if (bit_get(data, 60))
        return false;
    return true;
}

/*============================================================================*/
/**
 * @brief Feed a PSK pulse into one of the decode buffers
 *
 * Counts the number of bit periods in the duration and pushes the
 * polarity for each bit period.
 */
/*============================================================================*/
static bool indala26_feed_internal(bool polarity, uint32_t time, uint8_t *data)
{
    time += (INDALA26_US_PER_BIT / 2);

    size_t bit_count = time / INDALA26_US_PER_BIT;

    if (bit_count < INDALA26_ENCODED_BIT_SIZE)
    {
        for (size_t i = 0; i < bit_count; i++)
        {
            bit_push(data, INDALA26_ENCODED_DATA_SIZE, polarity);
            if (indala26_can_be_decoded(data))
                return true;
        }
    }

    return false;
}

/*============================================================================*/
/**
 * @brief Extract decoded data from encoded frame
 *
 * Copies specific bit ranges from encoded data (after preamble)
 * to the decoded output, matching Flipper's bit layout.
 */
/*============================================================================*/
static void indala26_save_decoded(uint8_t *decoded, const uint8_t *encoded)
{
    memset(decoded, 0, INDALA26_DECODED_DATA_SIZE);
    bit_copy(decoded, 0, encoded, 33, 22);
    bit_copy(decoded, 22, encoded, 55, 5);
    bit_copy(decoded, 27, encoded, 62, 2);
}

/*============================================================================*/
/**
 * @brief Initialize decoder state
 */
/*============================================================================*/
static void indala26_decoder_begin_impl(void)
{
    memset(g_indala_encoded, 0, sizeof(g_indala_encoded));
    memset(g_indala_neg_encoded, 0, sizeof(g_indala_neg_encoded));
    memset(g_indala_corrupted, 0, sizeof(g_indala_corrupted));
    memset(g_indala_neg_corrupted, 0, sizeof(g_indala_neg_corrupted));
    memset(g_indala_decoded, 0, sizeof(g_indala_decoded));
}

/*============================================================================*/
/**
 * @brief Feed edge events into the Indala26 PSK decoder
 *
 * PSK decoding: each edge event gives a polarity (level) and duration.
 * We feed each event into 4 parallel buffers (positive, negative,
 * corrupted-positive, corrupted-negative) to handle phase ambiguity.
 * This matches the Flipper Zero approach.
 */
/*============================================================================*/
static bool indala26_decoder_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++)
    {
        bool level = (evt[i].edge != 0);
        uint32_t duration = evt[i].t_us;

        if (duration <= (INDALA26_US_PER_BIT / 2))
            continue;

        /* Normal polarity */
        if (indala26_feed_internal(level, duration, g_indala_encoded))
        {
            indala26_save_decoded(g_indala_decoded, g_indala_encoded);
            goto detected;
        }
        /* Inverted polarity */
        if (indala26_feed_internal(!level, duration, g_indala_neg_encoded))
        {
            indala26_save_decoded(g_indala_decoded, g_indala_neg_encoded);
            goto detected;
        }

        /* Corrupted phase (shifted timing) */
        if (duration > (INDALA26_US_PER_BIT / 4))
        {
            uint32_t adjusted = duration;
            if (level)
                adjusted += 120;
            else if (adjusted > 120)
                adjusted -= 120;

            if (indala26_feed_internal(level, adjusted, g_indala_corrupted))
            {
                indala26_save_decoded(g_indala_decoded, g_indala_corrupted);
                goto detected;
            }
            if (indala26_feed_internal(!level, adjusted, g_indala_neg_corrupted))
            {
                indala26_save_decoded(g_indala_decoded, g_indala_neg_corrupted);
                goto detected;
            }
        }
    }

    return false;

detected:
    /* Copy to lfrfid_tag_info.uid for system compatibility */
    memcpy(lfrfid_tag_info.uid, g_indala_decoded,
           min(sizeof(lfrfid_tag_info.uid), INDALA26_DECODED_DATA_SIZE));
    return true;
}

/*============================================================================*/
/**
 * @brief Return pointer to decoded card data
 */
/*============================================================================*/
static uint8_t *indala26_get_data(void *proto)
{
    (void)proto;
    return g_indala_decoded;
}

/*============================================================================*/
/**
 * @brief Extract facility code from decoded data (Flipper layout)
 */
/*============================================================================*/
static uint8_t indala26_get_fc(const uint8_t *data)
{
    uint8_t fc = 0;
    fc = (uint8_t)((fc << 1) | bit_get(data, 24));
    fc = (uint8_t)((fc << 1) | bit_get(data, 16));
    fc = (uint8_t)((fc << 1) | bit_get(data, 11));
    fc = (uint8_t)((fc << 1) | bit_get(data, 14));
    fc = (uint8_t)((fc << 1) | bit_get(data, 15));
    fc = (uint8_t)((fc << 1) | bit_get(data, 20));
    fc = (uint8_t)((fc << 1) | bit_get(data, 6));
    fc = (uint8_t)((fc << 1) | bit_get(data, 25));
    return fc;
}

/*============================================================================*/
/**
 * @brief Extract card number from decoded data (Flipper layout)
 */
/*============================================================================*/
static uint16_t indala26_get_cn(const uint8_t *data)
{
    uint16_t cn = 0;
    cn = (uint16_t)((cn << 1) | bit_get(data, 9));
    cn = (uint16_t)((cn << 1) | bit_get(data, 12));
    cn = (uint16_t)((cn << 1) | bit_get(data, 10));
    cn = (uint16_t)((cn << 1) | bit_get(data, 7));
    cn = (uint16_t)((cn << 1) | bit_get(data, 19));
    cn = (uint16_t)((cn << 1) | bit_get(data, 3));
    cn = (uint16_t)((cn << 1) | bit_get(data, 2));
    cn = (uint16_t)((cn << 1) | bit_get(data, 18));
    cn = (uint16_t)((cn << 1) | bit_get(data, 13));
    cn = (uint16_t)((cn << 1) | bit_get(data, 0));
    cn = (uint16_t)((cn << 1) | bit_get(data, 4));
    cn = (uint16_t)((cn << 1) | bit_get(data, 21));
    cn = (uint16_t)((cn << 1) | bit_get(data, 23));
    cn = (uint16_t)((cn << 1) | bit_get(data, 26));
    cn = (uint16_t)((cn << 1) | bit_get(data, 17));
    cn = (uint16_t)((cn << 1) | bit_get(data, 8));
    return cn;
}

/*============================================================================*/
/**
 * @brief Render card data for display
 */
/*============================================================================*/
static void indala26_render_data(void *proto, char *result)
{
    (void)proto;
    uint8_t fc = indala26_get_fc(g_indala_decoded);
    uint16_t cn = indala26_get_cn(g_indala_decoded);

    sprintf(result,
            "FC: %u  Card: %u\n"
            "Hex: %02X%02X%02X%02X",
            fc, cn,
            g_indala_decoded[0], g_indala_decoded[1],
            g_indala_decoded[2], g_indala_decoded[3]);
}

/*============================================================================*/
/**
 * @brief Protocol definition table entry
 */
/*============================================================================*/
const LFRFIDProtocolBase protocol_indala26 = {
    .name = "Indala26",
    .manufacturer = "Motorola",
    .data_size = INDALA26_DECODED_DATA_SIZE,
    .features = LFRFIDFeaturePSK,
    .get_data = (lfrfidProtocolGetData)indala26_get_data,
    .decoder =
    {
        .begin   = (lfrfidProtocolDecoderBegin)indala26_decoder_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)indala26_decoder_execute_impl,
    },
    .encoder =
    {
        .begin = NULL,
        .send  = NULL,
    },
    .write =
    {
        .begin = NULL,
        .send  = NULL,
    },
    .render_data = (lfrfidProtocolRenderData)indala26_render_data,
};
