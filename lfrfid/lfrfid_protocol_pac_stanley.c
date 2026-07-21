/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — PAC/Stanley protocol decoder
 *
 * Based on the Flipper Zero PAC/Stanley protocol approach.
 * Direct modulation, 256us per cycle, 128-bit frame.
 *
 * Copyright (C) Flipper Devices Inc. (GPLv3)
 * Modifications: Copyright (C) 2026 Monstatek
 */
/*************************** I N C L U D E S **********************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "app_freertos.h"
#include "cmsis_os.h"
#include "main.h"
#include "uiView.h"

#include "lfrfid.h"
#include "lfrfid_bit_lib.h"
#include "t5577.h"
#include "bit_lib.h"

/***************************** C O N S T A N T S ******************************/

#define PAC_DATA_START_INDEX  39   /* 8 + (3 * 10) + 1 */
#define PAC_BYTE_LENGTH       10

/***************************** V A R I A B L E S ******************************/

static uint8_t g_pac_encoded[PAC_STANLEY_ENCODED_SIZE];
static uint8_t g_pac_decoded[PAC_STANLEY_DECODED_SIZE];
static bool    g_pac_got_preamble;
static bool    g_pac_inverted;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Reverse a byte (bit-reverse)                                               */
/*============================================================================*/
static uint8_t pac_reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

/*============================================================================*/
/* Validation: preamble, structure, parity, and checksum                      */
/*============================================================================*/
static bool pac_can_be_decoded(const uint8_t *data)
{
    /* Preamble: byte[0] = 0xFF */
    if (bl_get_bits(data, 0, 8) != 0xFF)
        return false;

    /* Bit 8 = 0, bit 9 = 0, bit 10 = 1 */
    if (bl_get_bit(data, 8))
        return false;
    if (bl_get_bit(data, 9))
        return false;
    if (!bl_get_bit(data, 10))
        return false;

    /* Bits 11-18 = 0x02 */
    if (bl_get_bits(data, 11, 8) != 0x02)
        return false;

    /* Wrap preamble: byte[16] = 0xFF */
    if (bl_get_bits(data, 128, 8) != 0xFF)
        return false;

    /* Checksum and parity validation */
    uint8_t checksum = 0;
    uint8_t stripped_byte = 0;

    for (int idx = 0; idx < 9; idx++) {
        uint8_t byte_val = pac_reverse_byte(
            bl_get_bits(data, PAC_DATA_START_INDEX + (PAC_BYTE_LENGTH * idx), 8));
        stripped_byte = byte_val & 0x7F;

        /* Check odd parity: MSB of reversed byte is parity bit */
        uint8_t parity_bit = (byte_val >> 7) & 1;
        uint8_t calc_parity = bl_test_parity_32((uint32_t)stripped_byte, 7) ? 1 : 0;
        if (parity_bit != calc_parity)
            return false;

        if (idx < 8)
            checksum ^= stripped_byte;
    }

    /* Last stripped_byte (idx=8) is the checksum byte */
    if (stripped_byte != checksum)
        return false;

    return true;
}

/*============================================================================*/
static void pac_decode(const uint8_t *encoded, uint8_t *decoded)
{
    memset(decoded, 0, PAC_STANLEY_DECODED_SIZE);

    /* Extract 8 data bytes (7-bit each after removing parity) */
    uint8_t card_data[8];
    for (int idx = 0; idx < 8; idx++) {
        uint8_t byte_val = pac_reverse_byte(
            bl_get_bits(encoded, PAC_DATA_START_INDEX + (PAC_BYTE_LENGTH * idx), 8));
        card_data[idx] = byte_val & 0x7F;  /* strip parity */
    }

    /* Pack 8 x 7-bit ASCII digits into 4 decoded bytes (hex conversion) */
    for (int i = 0; i < 4; i++) {
        uint8_t hi = 0, lo = 0;

        /* Convert ASCII digit to hex nibble */
        if (card_data[i * 2] >= '0' && card_data[i * 2] <= '9')
            hi = card_data[i * 2] - '0';
        else if (card_data[i * 2] >= 'A' && card_data[i * 2] <= 'F')
            hi = card_data[i * 2] - 'A' + 10;

        if (card_data[i * 2 + 1] >= '0' && card_data[i * 2 + 1] <= '9')
            lo = card_data[i * 2 + 1] - '0';
        else if (card_data[i * 2 + 1] >= 'A' && card_data[i * 2 + 1] <= 'F')
            lo = card_data[i * 2 + 1] - 'A' + 10;

        decoded[i] = (uint8_t)((hi << 4) | lo);
    }
}

/*============================================================================*/
static void pac_begin_impl(void)
{
    memset(g_pac_encoded, 0, sizeof(g_pac_encoded));
    memset(g_pac_decoded, 0, sizeof(g_pac_decoded));
    g_pac_got_preamble = false;
    g_pac_inverted = false;
}

/*============================================================================*/
static bool pac_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        bool level = (evt[i].edge != 0);
        uint32_t dur = evt[i].t_us;

        /* Ignore extremely long pulses */
        if (dur > 4000)
            continue;

        uint8_t pulses = (uint8_t)roundf((float)dur / (float)PAC_STANLEY_US_PER_CYCLE);

        if (pulses >= 9 && !g_pac_got_preamble) {
            /* First long pulse: preamble detected (8 high bits) */
            pulses = 8;
            g_pac_got_preamble = true;
            g_pac_inverted = !level;
        } else if (pulses >= 9 && g_pac_got_preamble) {
            /* Second long pulse after preamble: reset */
            g_pac_got_preamble = false;
            continue;
        } else if (pulses == 0 && dur > 60) {
            /* Short pulse that rounded to 0 but is real: treat as 1 */
            pulses = 1;
        }

        if (pulses) {
            for (uint8_t j = 0; j < pulses; j++)
                bl_push_bit(g_pac_encoded, PAC_STANLEY_ENCODED_SIZE,
                            level ^ g_pac_inverted);

            if (pac_can_be_decoded(g_pac_encoded)) {
                pac_decode(g_pac_encoded, g_pac_decoded);
                memcpy(lfrfid_tag_info.uid, g_pac_decoded,
                       min(sizeof(lfrfid_tag_info.uid), PAC_STANLEY_DECODED_SIZE));
                return true;
            }
        }
    }
    return false;
}

/*============================================================================*/
static uint8_t *pac_get_data(void *proto) { (void)proto; return g_pac_decoded; }

/*============================================================================*/
static void pac_render_data(void *proto, char *result)
{
    (void)proto;
    uint32_t card_id = ((uint32_t)g_pac_decoded[0] << 24) |
                       ((uint32_t)g_pac_decoded[1] << 16) |
                       ((uint32_t)g_pac_decoded[2] << 8) |
                       g_pac_decoded[3];
    sprintf(result,
            "ID: %lu\n"
            "Hex: %02X%02X%02X%02X",
            (unsigned long)card_id,
            g_pac_decoded[0], g_pac_decoded[1],
            g_pac_decoded[2], g_pac_decoded[3]);
}

/*============================================================================*/
/* T5577 write support — ported from Flipper protocol_pac_stanley_write_data /
 * protocol_pac_stanley_encoder_start (lib/lfrfid, GPLv3).
 *
 * PAC/Stanley: DIRECT modulation, RF/32 bitrate, 4 data blocks. The 128-bit
 * encoded frame is: 0xFF mark/stop preamble, start bit + reflected STX, then 8
 * ASCII-hex card-id bytes ("20" + 8 hex chars of the 4 decoded bytes) plus a
 * checksum byte, each carried as reversed 7-bit + odd-parity in a 10-bit cell
 * (start/stop framing bits pre-seeded by the 0x40>> pattern). Decoded data
 * (4 bytes) is sourced from tag->uid, which holds it verbatim.
 */
/*============================================================================*/

/* Flipper toolbox uint8_to_hex_chars: 4 src bytes -> 8 uppercase hex chars */
static void pac_uint8_to_hex_chars(const uint8_t* src, uint8_t* target, int length)
{
    static const char hex[] = "0123456789ABCDEF";
    for (int i = 0; i < length; i += 2) {
        target[i]     = (uint8_t)hex[src[i / 2] >> 4];
        target[i + 1] = (uint8_t)hex[src[i / 2] & 0x0F];
    }
}

/* Port of protocol_pac_stanley_encoder_start: build 128-bit frame from 4 bytes */
static void pac_encode(const uint8_t* decoded, uint8_t* encoded)
{
    uint8_t idbytes[10];
    idbytes[0] = '2';
    idbytes[1] = '0';
    pac_uint8_to_hex_chars(decoded, &idbytes[2], 8);

    /* Insert start and stop bits (10-bit cell framing pattern) */
    for (size_t i = 0; i < 16; i++)
        encoded[i] = (uint8_t)(0x40 >> ((i + 3) % 5 * 2));

    encoded[0] = 0xFF; /* mark + stop */
    encoded[1] = 0x20; /* start + reflect8(STX) */

    uint8_t checksum = 0;
    for (size_t i = 2; i < 13; i++) {
        uint8_t shift = (uint8_t)(7 - (i + 3) % 4 * 2);
        uint8_t index = (uint8_t)(i + (i - 1) / 4);

        uint16_t pattern;
        if (i < 12) {
            pattern = bit_lib_reverse_8_fast(idbytes[i - 2]);
            pattern |= bit_lib_test_parity_32(pattern, BitLibParityOdd);
            if (i > 3) checksum ^= idbytes[i - 2];
        } else {
            pattern = (uint16_t)((bit_lib_reverse_8_fast(checksum) & 0xFE) |
                                 (bit_lib_test_parity_32(checksum, BitLibParityOdd)));
        }
        pattern = (uint16_t)(pattern << shift);

        encoded[index]     |= (uint8_t)(pattern >> 8 & 0xFF);
        encoded[index + 1] |= (uint8_t)(pattern & 0xFF);
    }
}

void protocol_pac_stanley_write_begin(void* protocol, void* data)
{
    LFRFID_TAG_INFO* tag_data = (LFRFID_TAG_INFO*)protocol;
    LFRFIDProgram*   write    = (LFRFIDProgram*)data;
    const uint8_t*   decoded  = tag_data->uid;
    uint8_t          encoded[PAC_STANLEY_ENCODED_SIZE];

    memset(encoded, 0, sizeof(encoded));
    pac_encode(decoded, encoded);

    if(write && write->type == LFRFIDProgramTypeT5577) {
        /* DIRECT modulation, RF/32 bitrate, 4 data blocks */
        write->t5577.block_data[0] = T5577_MOD_DIRECT | T5577_BITRATE_RF_32 | T5577_TRANS_BL_1_4;
        write->t5577.block_data[1] = bit_lib_get_bits_32(encoded, 0, 32);
        write->t5577.block_data[2] = bit_lib_get_bits_32(encoded, 32, 32);
        write->t5577.block_data[3] = bit_lib_get_bits_32(encoded, 64, 32);
        write->t5577.block_data[4] = bit_lib_get_bits_32(encoded, 96, 32);
        write->t5577.max_blocks = 5;
    }
}

void protocol_pac_stanley_write_send(void* proto)
{
    (void)proto;
    t5577_execute_write(lfrfid_program, 0);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_pac_stanley = {
    .name = "PAC/Stanley",
    .manufacturer = "PAC",
    .data_size = PAC_STANLEY_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)pac_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)pac_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)pac_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = {
        .begin = (lfrfidProtocolWriteBegin)protocol_pac_stanley_write_begin,
        .send  = (lfrfidProtocolWriteSend)protocol_pac_stanley_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)pac_render_data,
};
