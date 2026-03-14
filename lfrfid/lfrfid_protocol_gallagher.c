/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Gallagher protocol decoder
 *
 * Based on the Flipper Zero Gallagher protocol approach.
 * ASK/Manchester modulation, RF/32, 112-bit frame (96 encoded + 16 overlap).
 * Preamble: 0x7FEA at bit 0 and bit 96
 * CRC-8: poly=0x07, init=0x2C, data bytes extracted at every 9th bit interval
 * Descramble with 256-byte lookup table
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

static manch_decoder_t g_gallagher_dec;
static uint8_t g_gallagher_decoded[GALLAGHER_DECODED_SIZE];

/* Gallagher descramble table (256 entries) */
static const uint8_t gallagher_descramble_table[256] = {
    0x2f,0x6e,0xdd,0xdf,0x1d,0x0f,0xb0,0x76,0xad,0xaf,0x7f,0xbb,0x77,0x85,0x11,
    0x6d,0xf4,0xd2,0x84,0x42,0xeb,0xf7,0x34,0x55,0x4a,0x3a,0x10,0x71,0xe7,0xa1,
    0x62,0x1a,0x3e,0x4c,0x14,0xd3,0x5e,0xb2,0x7d,0x56,0xbc,0x27,0x82,0x60,0xe3,
    0xae,0x1f,0x9b,0xaa,0x2b,0x95,0x49,0x73,0xe1,0x92,0x79,0x91,0x38,0x6c,0x19,
    0x0e,0xa9,0xe2,0x8d,0x66,0xc7,0x5a,0xf5,0x1c,0x80,0x99,0xbe,0x4e,0x41,0xf0,
    0xe8,0xa6,0x20,0xab,0x87,0xc8,0x1e,0xa0,0x59,0x7b,0x0c,0xc3,0x3c,0x61,0xcc,
    0x40,0x9e,0x06,0x52,0x1b,0x32,0x8c,0x12,0x93,0xbf,0xef,0x3b,0x25,0x0d,0xc2,
    0x88,0xd1,0xe0,0x07,0x2d,0x70,0xc6,0x29,0x6a,0x4d,0x47,0x26,0xa3,0xe4,0x8b,
    0xf6,0x97,0x2c,0x5d,0x3d,0xd7,0x96,0x28,0x02,0x08,0x30,0xa7,0x22,0xc9,0x65,
    0xf8,0xb7,0xb4,0x8a,0xca,0xb9,0xf2,0xd0,0x17,0xff,0x46,0xfb,0x9a,0xba,0x8f,
    0xb6,0x69,0x68,0x8e,0x21,0x6f,0xc4,0xcb,0xb3,0xce,0x51,0xd4,0x81,0x00,0x2e,
    0x9c,0x74,0x63,0x45,0xd9,0x16,0x35,0x5f,0xed,0x78,0x9f,0x01,0x48,0x04,0xc1,
    0x33,0xd6,0x4f,0x94,0xde,0x31,0x9d,0x0a,0xac,0x18,0x4b,0xcd,0x98,0xb8,0x37,
    0xa2,0x83,0xec,0x03,0xd8,0xda,0xe5,0x7a,0x6b,0x53,0xd5,0x15,0xa4,0x43,0xe9,
    0x90,0x67,0x58,0xc0,0xa5,0xfa,0x2a,0xb1,0x75,0x50,0x39,0x5c,0xe6,0xdc,0x89,
    0xfc,0xcf,0xfe,0xf9,0x57,0x54,0x64,0xa8,0xee,0x23,0x0b,0xf1,0xea,0xfd,0xdb,
    0xbd,0x09,0xb5,0x5b,0x05,0x86,0x13,0xf3,0x24,0xc5,0x3f,0x44,0x72,0x7c,0x7e,
    0x36
};

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Gallagher validation: Flipper-exact preamble + CRC-8 check                 */
/*============================================================================*/
static bool gallagher_can_be_decoded(const uint8_t *data)
{
    /* Check 16-bit preamble at bit 0 and bit 96 */
    /* 0b0111111111101010 = 0x7FEA */
    if (bl_get_bits_16(data, 0, 16) != 0x7FEA)
        return false;
    if (bl_get_bits_16(data, 96, 16) != 0x7FEA)
        return false;

    /* Extract 8 data bytes from positions 16+(9*i) for i=0..7
     * (every 9th bit after preamble is a separator, extract 8 bits between) */
    uint8_t checksum_arr[8];
    for (int i = 0; i < 8; i++)
        checksum_arr[i] = bl_get_bits(data, 16 + (9 * i), 8);

    /* CRC byte at position 16+(9*8) = bit 88, 8 bits */
    uint8_t crc_byte = bl_get_bits(data, 16 + (9 * 8), 8);

    /* CRC-8: poly=0x07, init=0x2C, refin=false, refout=false, xorout=0x00 */
    uint8_t calc_crc = bl_crc8(checksum_arr, 8, 0x07, 0x2C, false, false, 0x00);

    if (calc_crc != crc_byte)
        return false;

    return true;
}

/*============================================================================*/
static void gallagher_decode(const uint8_t *data, uint8_t *decoded)
{
    /* Work on a copy starting at bit 16, remove every 9th bit (separator),
     * then descramble */
    uint8_t temp[12];
    memset(temp, 0, sizeof(temp));

    /* Copy bits 16..95 (80 bits) into temp starting at bit 0 */
    bl_copy_bits(temp, 0, 80, data, 16);

    /* Remove every 9th bit: out of 80 bits, 72 remain (= 9 bytes) */
    bl_remove_bit_every_nth(temp, 0, 80, 9);

    /* First 8 bytes after removing separators are the scrambled data */
    uint8_t encoded_data[10];
    memset(encoded_data, 0, sizeof(encoded_data));
    for (int i = 0; i < 8; i++)
        encoded_data[i] = bl_get_bits(temp, i * 8, 8);

    /* Descramble each byte */
    for (int i = 0; i < 8; i++)
        encoded_data[i] = gallagher_descramble_table[encoded_data[i]];

    /* Extract fields per Flipper:
     * region(4 bits), issue_level(4 bits), FC(24 bits), card_number(32 bits) */
    uint8_t region = (encoded_data[5] & 0x1E) >> 1;
    uint8_t issue_level = encoded_data[9] & 0x0F;  /* note: encoded_data[9] is 0 here */
    uint32_t fc = (uint32_t)((encoded_data[7] & 0x0F) << 12) |
                  (uint32_t)(encoded_data[3] << 4) |
                  (uint32_t)((encoded_data[9] >> 4) & 0x0F);  /* encoded_data[9] is 0 */
    uint32_t card = (uint32_t)(encoded_data[2] << 16) |
                    (uint32_t)((encoded_data[6] & 0x1F) << 11) |
                    (uint32_t)(encoded_data[4] << 3) |
                    (uint32_t)((encoded_data[5] & 0xE0) >> 5);

    /* Store decoded: region(4), issue(4), FC(24), card(32) = 8 bytes */
    memset(decoded, 0, GALLAGHER_DECODED_SIZE);
    decoded[0] = (region << 4) | issue_level;
    decoded[1] = (uint8_t)((fc >> 16) & 0xFF);
    decoded[2] = (uint8_t)((fc >> 8) & 0xFF);
    decoded[3] = (uint8_t)(fc & 0xFF);
    decoded[4] = (uint8_t)((card >> 24) & 0xFF);
    decoded[5] = (uint8_t)((card >> 16) & 0xFF);
    decoded[6] = (uint8_t)((card >> 8) & 0xFF);
    decoded[7] = (uint8_t)(card & 0xFF);
}

/*============================================================================*/
static void gallagher_begin_impl(void)
{
    manch_init(&g_gallagher_dec, GALLAGHER_HALF_BIT_US,
               GALLAGHER_ENCODED_SIZE * 8);
    memset(g_gallagher_decoded, 0, sizeof(g_gallagher_decoded));
}

/*============================================================================*/
static bool gallagher_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    manch_feed_events(&g_gallagher_dec, evt, (uint8_t)size);

    if (manch_is_full(&g_gallagher_dec) &&
        gallagher_can_be_decoded(g_gallagher_dec.frame_buffer)) {
        gallagher_decode(g_gallagher_dec.frame_buffer, g_gallagher_decoded);
        memcpy(lfrfid_tag_info.uid, g_gallagher_decoded,
               min(sizeof(lfrfid_tag_info.uid), GALLAGHER_DECODED_SIZE));
        lfrfid_tag_info.bitrate = 32;
        return true;
    }

    return false;
}

/*============================================================================*/
static uint8_t *gallagher_get_data(void *proto) { (void)proto; return g_gallagher_decoded; }

/*============================================================================*/
static void gallagher_render_data(void *proto, char *result)
{
    (void)proto;
    uint8_t region = (g_gallagher_decoded[0] >> 4) & 0x0F;
    uint8_t issue = g_gallagher_decoded[0] & 0x0F;
    uint32_t fc = ((uint32_t)g_gallagher_decoded[1] << 16) |
                  ((uint32_t)g_gallagher_decoded[2] << 8) |
                  g_gallagher_decoded[3];
    uint32_t card = ((uint32_t)g_gallagher_decoded[4] << 24) |
                    ((uint32_t)g_gallagher_decoded[5] << 16) |
                    ((uint32_t)g_gallagher_decoded[6] << 8) |
                    g_gallagher_decoded[7];
    sprintf(result,
            "R:%u IL:%u FC:%lu\n"
            "Card: %lu",
            region, issue,
            (unsigned long)fc, (unsigned long)card);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_gallagher = {
    .name = "Gallagher",
    .manufacturer = "Gallagher",
    .data_size = GALLAGHER_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)gallagher_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)gallagher_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)gallagher_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)gallagher_render_data,
};
