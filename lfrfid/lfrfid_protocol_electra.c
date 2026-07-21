/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Electra protocol decoder
 *
 * Based on the Flipper Zero Electra protocol approach.
 * ASK/Manchester modulation, RF/64 (256us short, 512us long).
 * Uses dual 64-bit shift registers: base_data and epilogue.
 * Validates EM4100 header on base, row/column parity, epilogue filler.
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
#include "t5577.h"
#include "bit_lib.h"

/***************************** D E F I N E S **********************************/

/* Manchester timing: RF/64 = 256us short, 512us long */
#define ELECTRA_SHORT_MIN   156
#define ELECTRA_SHORT_MAX   356
#define ELECTRA_LONG_MIN    412
#define ELECTRA_LONG_MAX    612

/* Encoder layout (mirrors Flipper protocol_electra.c) */
#define ELECTRA_DECODED_BASE_DATA_SIZE          5   /* 5 EM4100 base bytes            */
#define ELECTRA_DECODED_EPILOGUE_START_POS      5   /* epilogue bytes at decoded[5..7]*/
#define ELECTRA_ENCODED_EPILOGUE_SIZE           8   /* epilogue word = 8 bytes        */
#define ELECTRA_EM_COLUMN_COUNT                 4
#define ELECTRA_EM_ROW_COUNT                    10
#define ELECTRA_EM_BITS_PER_ROW_COUNT           5   /* 4 data + 1 row parity          */

/***************************** V A R I A B L E S ******************************/

static uint64_t g_electra_base;
static uint64_t g_electra_epilogue;
static uint16_t g_electra_bit_count;
static bool g_electra_last_short;
static bool g_electra_last_edge;  /* false=fall, true=rise */
static uint8_t g_electra_decoded[ELECTRA_DECODED_SIZE];

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Shift a new bit through the dual registers                                 */
/* carry = MSB of epilogue goes to LSB of base_data                           */
/*============================================================================*/
static void electra_push_bit(bool bit)
{
    bool carry = (g_electra_epilogue >> 63) & 1;
    g_electra_base = (g_electra_base << 1) | (carry ? 1ULL : 0ULL);
    g_electra_epilogue = (g_electra_epilogue << 1) | (bit ? 1ULL : 0ULL);

    if (g_electra_bit_count < 128)
        g_electra_bit_count++;
}

/*============================================================================*/
/* Validate: Flipper Zero Electra logic                                       */
/*============================================================================*/
static bool electra_can_be_decoded(void)
{
    if (g_electra_bit_count < 128)
        return false;

    /* Base data must have EM4100 header: 9 consecutive 1s at MSB (bits 63-55) */
    uint16_t header = (uint16_t)(g_electra_base >> 55);
    if (header != 0x01FF)
        return false;

    /* Stop bit: bit 0 of base must be 0 */
    if (g_electra_base & 1)
        return false;

    /* Epilogue must NOT have EM4100 header (distinguishes from plain EM4100) */
    uint16_t epi_header = (uint16_t)(g_electra_epilogue >> 55);
    if (epi_header == 0x01FF)
        return false;

    /* Row parity: 10 rows of 5 bits each (after 9-bit header), even parity per row */
    for (int row = 0; row < 10; row++) {
        int start = 55 - 1 - (row * 5);
        uint8_t count = 0;
        for (int b = 0; b < 5; b++) {
            if ((g_electra_base >> (start - b)) & 1)
                count++;
        }
        if (count & 1)
            return false;
    }

    /* Column parity: 4 columns, even parity */
    for (int col = 0; col < 4; col++) {
        uint8_t count = 0;
        for (int row = 0; row < 11; row++) { /* 10 data rows + 1 parity row */
            int bit_pos = 55 - 1 - (row * 5) - col;
            if (bit_pos >= 0 && ((g_electra_base >> bit_pos) & 1))
                count++;
        }
        if (count & 1)
            return false;
    }

    /* Epilogue filler check: all bytes except last 3 must be same value */
    uint8_t epi_bytes[8];
    for (int i = 0; i < 8; i++)
        epi_bytes[i] = (uint8_t)(g_electra_epilogue >> (56 - i * 8));

    uint8_t filler = epi_bytes[0];
    for (int i = 1; i < 5; i++) {
        if (epi_bytes[i] != filler)
            return false;
    }

    return true;
}

/*============================================================================*/
/* Extract decoded data: 5 bytes from base + 3 bytes from epilogue           */
/*============================================================================*/
static void electra_decode(void)
{
    memset(g_electra_decoded, 0, ELECTRA_DECODED_SIZE);

    /* Base data: extract 10 nibbles (each row has 4 data + 1 parity) -> 5 bytes */
    for (int row = 0; row < 10; row++) {
        int start = 55 - 1 - (row * 5);
        uint8_t nibble = 0;
        for (int b = 0; b < 4; b++) {
            nibble = (uint8_t)((nibble << 1) | ((g_electra_base >> (start - b)) & 1));
        }
        if (row & 1)
            g_electra_decoded[row / 2] |= nibble;
        else
            g_electra_decoded[row / 2] = (uint8_t)(nibble << 4);
    }

    /* Epilogue: last 3 bytes */
    g_electra_decoded[5] = (uint8_t)(g_electra_epilogue >> 16);
    g_electra_decoded[6] = (uint8_t)(g_electra_epilogue >> 8);
    g_electra_decoded[7] = (uint8_t)(g_electra_epilogue);
}

/*============================================================================*/
static void electra_begin_impl(void)
{
    g_electra_base = 0;
    g_electra_epilogue = 0;
    g_electra_bit_count = 0;
    g_electra_last_short = false;
    g_electra_last_edge = false;
    memset(g_electra_decoded, 0, sizeof(g_electra_decoded));
}

/*============================================================================*/
static bool electra_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (uint16_t i = 0; i < size; i++) {
        uint16_t dur = evt[i].t_us;
        bool is_rise = (evt[i].edge == 1);

        if (dur >= ELECTRA_SHORT_MIN && dur <= ELECTRA_SHORT_MAX) {
            /* Short pulse */
            if (!g_electra_last_short) {
                g_electra_last_short = true;
                g_electra_last_edge = is_rise;
            } else {
                /* Two consecutive shorts = one Manchester bit */
                /* The bit value depends on the edge transition */
                bool bit = !g_electra_last_edge; /* fall->rise = 1, rise->fall = 0 */
                electra_push_bit(bit);
                g_electra_last_short = false;

                if (electra_can_be_decoded()) {
                    electra_decode();
                    memcpy(lfrfid_tag_info.uid, g_electra_decoded,
                           min(sizeof(lfrfid_tag_info.uid), ELECTRA_DECODED_SIZE));
                    lfrfid_tag_info.bitrate = 64;
                    return true;
                }
            }
        } else if (dur >= ELECTRA_LONG_MIN && dur <= ELECTRA_LONG_MAX) {
            /* Long pulse = one Manchester bit */
            g_electra_last_short = false;
            /* Bit value: rising edge = 1, falling edge = 0 */
            electra_push_bit(is_rise);

            if (electra_can_be_decoded()) {
                electra_decode();
                memcpy(lfrfid_tag_info.uid, g_electra_decoded,
                       min(sizeof(lfrfid_tag_info.uid), ELECTRA_DECODED_SIZE));
                lfrfid_tag_info.bitrate = 64;
                return true;
            }
        } else {
            /* Out of range: reset */
            g_electra_last_short = false;
        }
    }

    return false;
}

/*============================================================================*/
static uint8_t *electra_get_data(void *proto) { (void)proto; return g_electra_decoded; }

/*============================================================================*/
static void electra_render_data(void *proto, char *result)
{
    (void)proto;
    sprintf(result,
            "Electra\n"
            "Hex: %02X%02X%02X%02X%02X"
            " E: %02X%02X%02X",
            g_electra_decoded[0], g_electra_decoded[1], g_electra_decoded[2],
            g_electra_decoded[3], g_electra_decoded[4],
            g_electra_decoded[5], g_electra_decoded[6], g_electra_decoded[7]);
}

/*============================================================================*/
/* T5577 write support — ported from Flipper protocol_electra_write_data /
 * protocol_electra_encoder_start (lib/lfrfid, GPLv3).
 *
 * Electra: ASK/Manchester modulation, RF/64 bitrate, 5 data blocks. The frame
 * is a 64-bit EM4100 base word (9-bit header, 10 rows of 4 data + 1 row parity,
 * 4 column-parity bits, 1 stop bit) plus a 64-bit Electra epilogue word (2
 * meaningful bytes + 6 filler bytes). Block 0 is config; blocks 1-2 carry the
 * base word, blocks 3-4 carry the Electra-specific epilogue word — matching
 * Flipper's block layout and 5-block count exactly.
 *
 * Payload is sourced from the module's full 8-byte decoded buffer
 * g_electra_decoded (decoded[0..4] = base, decoded[5..7] = epilogue); it does
 * not fit tag->uid (5 bytes).
 */
/*============================================================================*/

/* One EM4100 nibble: 4 data bits (MSB first) + 1 even row-parity bit,
 * appended to the base word. Mirrors Flipper's em_write_nibble(). */
static void electra_em_write_nibble(bool low_nibble, uint8_t data, uint64_t* encoded_base_data)
{
    uint8_t parity_sum = 0;
    uint8_t start = low_nibble ? 0 : 4;

    for(int8_t i = (int8_t)(start + 3); i >= (int8_t)start; i--) {
        parity_sum += (data >> i) & 1;
        *encoded_base_data = (*encoded_base_data << 1) | ((data >> i) & 1);
    }

    *encoded_base_data = (*encoded_base_data << 1) | ((parity_sum % 2) & 1);
}

/* Build the 64-bit base word and 64-bit epilogue word from the decoded buffer.
 * Mirrors Flipper's protocol_electra_encoder_start(). */
static void electra_encode(const uint8_t* decoded, uint64_t* out_base, uint64_t* out_epilogue)
{
    /* header: 9 leading 1 bits */
    uint64_t base = 0b111111111ULL;

    /* data: 10 nibbles (2 per base byte), each with a row-parity bit */
    for(uint8_t i = 0; i < ELECTRA_DECODED_BASE_DATA_SIZE; i++) {
        electra_em_write_nibble(false, decoded[i], &base);
        electra_em_write_nibble(true, decoded[i], &base);
    }

    /* column parity: 4 bits */
    for(uint8_t c = 0; c < ELECTRA_EM_COLUMN_COUNT; c++) {
        uint8_t parity_sum = 0;
        for(uint8_t i = 1; i <= ELECTRA_EM_ROW_COUNT; i++) {
            uint8_t parity_bit =
                (base >> (i * ELECTRA_EM_BITS_PER_ROW_COUNT - 1)) & 1;
            parity_sum += parity_bit;
        }
        base = (base << 1) | ((parity_sum % 2) & 1);
    }

    /* stop bit */
    base = (base << 1) | 0;

    /* epilogue: 2 meaningful bytes then 6 filler bytes (decoded[7] repeated) */
    uint64_t epilogue = decoded[ELECTRA_DECODED_EPILOGUE_START_POS];
    epilogue <<= 8;
    epilogue |= decoded[ELECTRA_DECODED_EPILOGUE_START_POS + 1];
    for(uint8_t i = 2; i < ELECTRA_ENCODED_EPILOGUE_SIZE; i++) {
        epilogue <<= 8;
        epilogue |= decoded[ELECTRA_DECODED_EPILOGUE_START_POS + 2];
    }

    *out_base = base;
    *out_epilogue = epilogue;
}

/*============================================================================*/
void protocol_electra_write_begin(void* protocol, void* data)
{
    (void)protocol; /* LFRFID_TAG_INFO* unused: 8-byte data (>uid) from g_electra_decoded */
    LFRFIDProgram* write   = (LFRFIDProgram*)data;
    const uint8_t* decoded = g_electra_decoded;
    uint8_t        encoded[ELECTRA_ENCODED_SIZE];
    uint64_t       base = 0, epilogue = 0;

    memset(encoded, 0, sizeof(encoded));

    electra_encode(decoded, &base, &epilogue);
    bit_lib_num_to_bytes_be(base, 8, &encoded[0]);      /* bits 0..63   */
    bit_lib_num_to_bytes_be(epilogue, 8, &encoded[8]);  /* bits 64..127 */

    if(write && write->type == LFRFIDProgramTypeT5577) {
        write->t5577.block_data[0] =
            T5577_MOD_MANCHESTER | T5577_BITRATE_RF_64 | T5577_TRANS_BL_1_4;
        write->t5577.block_data[1] = bit_lib_get_bits_32(encoded, 0, 32);   /* base  hi */
        write->t5577.block_data[2] = bit_lib_get_bits_32(encoded, 32, 32);  /* base  lo */
        write->t5577.block_data[3] = bit_lib_get_bits_32(encoded, 64, 32);  /* epi   hi */
        write->t5577.block_data[4] = bit_lib_get_bits_32(encoded, 96, 32);  /* epi   lo */
        write->t5577.max_blocks = 5;
    }
}

/*============================================================================*/
void protocol_electra_write_send(void* proto)
{
    (void)proto;
    t5577_execute_write(lfrfid_program, 0);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_electra = {
    .name = "Electra",
    .manufacturer = "Electra",
    .data_size = ELECTRA_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)electra_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)electra_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)electra_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = {
        .begin = (lfrfidProtocolWriteBegin)protocol_electra_write_begin,
        .send  = (lfrfidProtocolWriteSend)protocol_electra_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)electra_render_data,
};
