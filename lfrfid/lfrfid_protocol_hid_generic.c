/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — HID Generic (any HID Prox format)
 *
 * Based on the Flipper Zero HID Generic protocol approach.
 * Accepts any HID Proximity card regardless of bit format
 * (26, 34, 35, 37-bit, etc.) by checking only for valid
 * preamble + Manchester encoding.
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

#define HID_GEN_ENCODED_BITS  (HID_GENERIC_ENCODED_SIZE * 8)  /* 104 */

/***************************** V A R I A B L E S ******************************/

/* Decoded card data — full 44 bits in 6 bytes */
static uint8_t g_hid_decoded[HID_GENERIC_DECODED_SIZE];

/* 104-bit shift register for encoded stream */
static uint8_t g_hid_encoded[HID_GENERIC_ENCODED_SIZE];

/* Own FSK demod state (separate from H10301's instances) */
static fsk_symbol_state_t g_hid_sym_st;
static fsk_bit_state_t    g_hid_bit_st;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void    hid_generic_decoder_begin_impl(void);
static bool    hid_generic_decoder_execute_impl(void *proto, uint16_t size);
static bool    hid_generic_encoder_begin_impl(void *proto);
static void    hid_generic_encoder_send_impl(void *proto);
static uint8_t *hid_generic_get_data(void *proto);
static void    hid_generic_render_data(void *proto, char *result);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
 * @brief Push one bit into the 104-bit shift register (MSB-first)
 */
/*============================================================================*/
static void hid_push_bit(uint8_t *buf, size_t buf_size, uint8_t bit)
{
    for (size_t i = 0; i < buf_size - 1; i++)
        buf[i] = (uint8_t)((buf[i] << 1) | (buf[i + 1] >> 7));
    buf[buf_size - 1] = (uint8_t)((buf[buf_size - 1] << 1) | (bit & 1));
}


/*============================================================================*/
/**
 * @brief Check if the 104-bit window contains a valid HID frame
 *
 * Follows the Flipper Zero approach:
 *  1) Preamble 0x1D at byte[0] and byte[12]
 *  2) All 11 data bytes contain valid Manchester pairs (no 00 or 11)
 */
/*============================================================================*/
static bool hid_can_be_decoded(const uint8_t *data)
{
    /* Check preambles */
    if (data[0] != HID_GENERIC_PREAMBLE)
        return false;
    if (data[HID_GENERIC_ENCODED_SIZE - 1] != HID_GENERIC_PREAMBLE)
        return false;

    /* Check Manchester encoding for bytes 1..11 */
    for (size_t i = 1; i <= HID_GENERIC_DATA_BYTES; i++)
    {
        for (size_t n = 0; n < 4; n++)
        {
            uint8_t pair = (data[i] >> (n * 2)) & 0x03;
            if (pair == 0x03 || pair == 0x00)
                return false;
        }
    }

    return true;
}


/*============================================================================*/
/**
 * @brief Manchester-decode 88 encoded bits → 44 decoded bits
 *
 * Processes bytes 1..11 of the encoded frame.
 * Within each byte, bit pairs are processed MSB-first (n=3 → n=0).
 * Manchester convention: 01 = '0', 10 = '1'.
 */
/*============================================================================*/
static void hid_manchester_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, HID_GENERIC_DECODED_SIZE);
    int out_bit = 0;

    for (size_t i = 1; i <= HID_GENERIC_DATA_BYTES; i++)
    {
        for (int n = 3; n >= 0; n--)
        {
            uint8_t pair = (encoded[i] >> (n * 2)) & 0x03;
            uint8_t bit = (pair == 0x02) ? 1 : 0;  /* 10 = 1, 01 = 0 */

            if (bit)
                decoded[out_bit / 8] |= (uint8_t)(1 << (7 - (out_bit % 8)));
            out_bit++;
        }
    }
}


/*============================================================================*/
/**
 * @brief Determine the HID protocol bit size from decoded data
 *
 * Follows the Flipper Zero approach: count leading zero bits
 * to determine the actual credential length.
 */
/*============================================================================*/
static uint8_t hid_decode_protocol_size(const uint8_t *decoded)
{
    /* Scan from MSB for the first '1' bit */
    for (size_t bit = 0; bit < 6; bit++)
    {
        if (decoded[bit / 8] & (1 << (7 - (bit % 8))))
            return (uint8_t)(44 - bit - 1);
    }

    /* Check bit 6 */
    if (!(decoded[0] & 0x02))
        return 37;

    /* Walk remaining bits */
    size_t bit_index = 7;
    uint8_t size = 36;
    while (bit_index < 44 && size >= 26)
    {
        if (decoded[bit_index / 8] & (1 << (7 - (bit_index % 8))))
            return size;
        size--;
        bit_index++;
    }

    return 0;  /* unknown */
}


/*============================================================================*/
/**
 * @brief Initialize decoder state
 */
/*============================================================================*/
static void hid_generic_decoder_begin_impl(void)
{
    memset(g_hid_encoded, 0, sizeof(g_hid_encoded));
    memset(g_hid_decoded, 0, sizeof(g_hid_decoded));
    fsk_symbol_state_init(&g_hid_sym_st);
    fsk_bit_state_init(&g_hid_bit_st);
}


/*============================================================================*/
/**
 * @brief Feed edge events into the HID Generic decoder
 *
 * Reuses the same FSK demodulation chain as H10301 (fsk_symbol_feed +
 * fsk_bit_feed) but with its own state instances and a wider 104-bit
 * shift register. Validation only requires preamble + valid Manchester.
 */
/*============================================================================*/
static bool hid_generic_decoder_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++)
    {
        uint8_t symbol;
        if (fsk_symbol_feed(&g_hid_sym_st, &evt[i], &symbol))
        {
            uint8_t bit;
            if (fsk_bit_feed(&g_hid_bit_st, symbol, &bit))
            {
                hid_push_bit(g_hid_encoded, HID_GENERIC_ENCODED_SIZE, bit);

                if (hid_can_be_decoded(g_hid_encoded))
                {
                    hid_manchester_decode(g_hid_encoded, g_hid_decoded);

                    /* Copy to lfrfid_tag_info.uid for system compatibility */
                    memcpy(lfrfid_tag_info.uid, g_hid_decoded,
                           sizeof(lfrfid_tag_info.uid));

                    return true;
                }
            }
        }
    }

    return false;
}


/*============================================================================*/
/**
 * @brief Manchester-encode 44 decoded bits back to 88 encoded bits (11 bytes)
 *
 * Reverse of hid_manchester_decode(). Convention: 0→01, 1→10.
 */
/*============================================================================*/
static void hid_manchester_encode(const uint8_t *decoded, uint8_t *encoded_data)
{
    int in_bit = 0;

    for (size_t i = 0; i < HID_GENERIC_DATA_BYTES; i++)
    {
        uint8_t byte_val = 0;
        for (int n = 3; n >= 0; n--)
        {
            uint8_t bit = (decoded[in_bit / 8] >> (7 - (in_bit % 8))) & 1;
            uint8_t pair = bit ? 0x02 : 0x01;  /* 1→10, 0→01 */
            byte_val |= (uint8_t)(pair << (n * 2));
            in_bit++;
        }
        encoded_data[i] = byte_val;
    }
}


/*============================================================================*/
/**
 * @brief Convert raw FSK2a bitstream to GPIO waveform steps
 *
 * Same approach as h10301_raw96_to_wave but parameterized for any bit count.
 * FSK2a: bit 0 = 6 cycles @ 64us, bit 1 = 5 cycles @ 80us.
 */
/*============================================================================*/
#define HID_EMUL_HALF_ONE_US   40
#define HID_EMUL_HALF_ZERO_US  32
#define HID_EMUL_PERIOD_ONE_US  78   /* 80-2 */
#define HID_EMUL_PERIOD_ZERO_US 62   /* 64-2 */

static int hid_fsk2a_to_wave(const uint8_t *raw_data, int total_bits,
                              uint8_t gpio_pin,
                              Encoded_Data_t *steps, size_t max_steps,
                              size_t *out_step_count)
{
    size_t idx = 0;
    uint32_t bsrr_set   = 1u << gpio_pin;
    uint32_t bsrr_reset = 1u << (gpio_pin + 16);

    for (int bit = 0; bit < total_bits; ++bit)
    {
        int byte_pos    = bit >> 3;
        int bit_in_byte = 7 - (bit & 7);
        uint8_t b = (raw_data[byte_pos] >> bit_in_byte) & 1u;

        uint16_t half_us, rest_us;
        int repeat;

        if (b == 0) {
            half_us = HID_EMUL_HALF_ZERO_US;
            rest_us = (uint16_t)(HID_EMUL_PERIOD_ZERO_US - half_us);
            repeat  = 6;
        } else {
            half_us = HID_EMUL_HALF_ONE_US;
            rest_us = (uint16_t)(HID_EMUL_PERIOD_ONE_US - half_us);
            repeat  = 5;
        }

        for (int i = 0; i < repeat; ++i)
        {
            if (idx >= max_steps) goto overflow;
            steps[idx].bsrr    = bsrr_set;
            steps[idx].time_us = half_us;
            idx++;

            if (idx >= max_steps) goto overflow;
            steps[idx].bsrr    = bsrr_reset;
            steps[idx].time_us = rest_us;
            idx++;
        }
    }

    if (out_step_count) *out_step_count = idx;
    return 0;

overflow:
    if (out_step_count) *out_step_count = idx;
    return -1;
}


/*============================================================================*/
/**
 * @brief Build emulation waveform from decoded HID Generic data
 *
 * Re-encodes the 44 decoded bits into a 104-bit FSK2a frame
 * (0x1D preamble + 88 Manchester bits + 0x1D preamble),
 * then converts to GPIO waveform steps.
 */
/*============================================================================*/
static bool hid_generic_encoder_begin_impl(void *proto)
{
    uint8_t frame[HID_GENERIC_ENCODED_SIZE];

    /* Build the 104-bit FSK2a frame from full 6-byte decoded data.
     * We use g_hid_decoded (local static, populated during decode)
     * instead of lfrfid_tag_info.uid which is only 5 bytes and
     * would lose the last 4 bits of the 44-bit credential. */
    frame[0] = HID_GENERIC_PREAMBLE;   /* 0x1D */
    hid_manchester_encode(g_hid_decoded, &frame[1]);
    frame[HID_GENERIC_ENCODED_SIZE - 1] = HID_GENERIC_PREAMBLE;  /* 0x1D */

    hid_fsk2a_to_wave(frame, HID_GEN_ENCODED_BITS, 2,
                       lfrfid_encoded_data.data, ENCODED_DATA_MAX,
                       (size_t *)&lfrfid_encoded_data.length);
    return true;
}


/*============================================================================*/
/**
 * @brief Start FSK2a emulation output
 */
/*============================================================================*/
static void hid_generic_encoder_send_impl(void *proto)
{
    (void)proto;
    lfrfid_encoded_data.index = 0;
    lfrfid_emul_hw_init();
}


/*============================================================================*/
/**
 * @brief Return pointer to decoded card data
 */
/*============================================================================*/
static uint8_t *hid_generic_get_data(void *proto)
{
    (void)proto;
    return g_hid_decoded;
}


/*============================================================================*/
/**
 * @brief Render card data for display
 *
 * Shows the protocol bit size and hex data.
 */
/*============================================================================*/
static void hid_generic_render_data(void *proto, char *result)
{
    (void)proto;
    uint8_t prot_size = hid_decode_protocol_size(g_hid_decoded);

    if (prot_size == 0)
    {
        sprintf(result,
                "Hex: %02X%02X%02X%02X%02X%X",
                g_hid_decoded[0], g_hid_decoded[1], g_hid_decoded[2],
                g_hid_decoded[3], g_hid_decoded[4], g_hid_decoded[5] >> 4);
    }
    else
    {
        sprintf(result,
                "%u-bit HID Proximity\n"
                "Hex: %02X%02X%02X%02X%02X%X",
                prot_size,
                g_hid_decoded[0], g_hid_decoded[1], g_hid_decoded[2],
                g_hid_decoded[3], g_hid_decoded[4], g_hid_decoded[5] >> 4);
    }
}


/*============================================================================*/
/**
 * @brief Protocol definition table entry
 */
/*============================================================================*/
const LFRFIDProtocolBase protocol_hid_generic = {
    .name = "HID Generic",
    .manufacturer = "HID",
    .data_size = HID_GENERIC_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)hid_generic_get_data,
    .decoder =
    {
        .begin   = (lfrfidProtocolDecoderBegin)hid_generic_decoder_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)hid_generic_decoder_execute_impl,
    },
    .encoder =
    {
        .begin = (lfrfidProtocolEncoderBegin)hid_generic_encoder_begin_impl,
        .send  = (lfrfidProtocolEncoderSend)hid_generic_encoder_send_impl,
    },
    .write =
    {
        .begin = NULL,
        .send  = NULL,
    },
    .render_data = (lfrfidProtocolRenderData)hid_generic_render_data,
};
