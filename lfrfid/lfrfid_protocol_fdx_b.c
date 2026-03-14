/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — FDX-B (ISO 11784/11785) animal tag decoder
 *
 * Based on the Flipper Zero FDX-B protocol approach.
 * ASK/Differential biphase modulation, RF/32, 128-bit frame.
 * Preamble: 11 bits = 0b00000000001
 * Control bits: every 9th bit after preamble must be 1
 * CRC-16/CCITT over extracted data bits
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

/***************************** D E F I N E S **********************************/

/* Bi-phase timing: RF/32 = 128us short, 256us long */
#define FDXB_SHORT_MIN   68
#define FDXB_SHORT_MAX   188
#define FDXB_LONG_MIN    196
#define FDXB_LONG_MAX    316

/* 128 data bits + 16 overlap for second preamble check = 144 bits = 18 bytes */
#define FDXB_FRAME_BITS  144
#define FDXB_FRAME_BYTES FDX_B_ENCODED_SIZE

/***************************** V A R I A B L E S ******************************/

static uint8_t g_fdxb_frame[FDXB_FRAME_BYTES];
static uint16_t g_fdxb_bit_count;
static bool g_fdxb_last_short;
static uint8_t g_fdxb_decoded[FDX_B_DECODED_SIZE];

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Push a bit into the bi-phase frame buffer (MSB shift register)             */
/*============================================================================*/
static void fdxb_push_bit(bool bit)
{
    bl_push_bit(g_fdxb_frame, FDXB_FRAME_BYTES, bit);
    if (g_fdxb_bit_count < FDXB_FRAME_BITS)
        g_fdxb_bit_count++;
}

/*============================================================================*/
/* CRC-16/CCITT: poly=0x1021, init=0x0000                                    */
/*============================================================================*/
static uint16_t fdxb_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc <<= 1;
        }
    }
    return crc;
}

/*============================================================================*/
/* Validate the 144-bit frame per Flipper Zero logic                          */
/*============================================================================*/
static bool fdxb_can_be_decoded(void)
{
    if (g_fdxb_bit_count < FDXB_FRAME_BITS)
        return false;

    /* Check 11-bit preamble at position 0: 0b00000000001 */
    if (bl_get_bits_16(g_fdxb_frame, 0, 11) != 0x001)
        return false;

    /* Check 11-bit preamble at position 128 (second frame start) */
    if (bl_get_bits_16(g_fdxb_frame, 128, 11) != 0x001)
        return false;

    /* Check control bits: every 9th bit starting at bit 11 must be 1 */
    /* 11 control bits total covering the 128-bit frame */
    for (int i = 0; i < 11; i++) {
        uint16_t pos = (uint16_t)(11 + (i * 9) + 8);
        if (pos >= 128)
            break;
        if (!bl_get_bit(g_fdxb_frame, pos))
            return false;
    }

    /* Extract data bits (skip preamble at bit 0-10 and control bits every 9th)
     * Data layout: after 11-bit preamble, groups of 8 data + 1 control
     * Extract 8 bytes of data = 64 data bits, then CRC is next 16 data bits
     */
    uint8_t extracted[10]; /* enough for data + CRC */
    memset(extracted, 0, sizeof(extracted));
    int out_bit = 0;

    for (int row = 0; row < 11; row++) {
        for (int col = 0; col < 8; col++) {
            uint16_t src = (uint16_t)(11 + row * 9 + col);
            if (src >= 128)
                break;
            bl_set_bit(extracted, (size_t)out_bit, bl_get_bit(g_fdxb_frame, src));
            out_bit++;
        }
    }

    /* out_bit should be ~88 data bits (11 rows * 8 cols, capped by frame)
     * First 64 bits = data, next 16 bits = CRC stored in stream */
    if (out_bit < 80)
        return false;

    /* CRC-16 over first 8 bytes (64 data bits) */
    uint16_t computed_crc = fdxb_crc16(extracted, 8);
    uint16_t stored_crc = (uint16_t)((extracted[8] << 8) | extracted[9]);

    if (computed_crc != stored_crc)
        return false;

    return true;
}

/*============================================================================*/
/* Decode: extract the 11 decoded bytes from the validated frame              */
/*============================================================================*/
static void fdxb_decode(void)
{
    memset(g_fdxb_decoded, 0, FDX_B_DECODED_SIZE);

    /* Extract all data bits (skip preamble and control bits) */
    int out_bit = 0;
    for (int row = 0; row < 13; row++) {
        for (int col = 0; col < 8; col++) {
            uint16_t src = (uint16_t)(11 + row * 9 + col);
            if (src >= 128)
                goto done;
            if (out_bit >= FDX_B_DECODED_SIZE * 8)
                goto done;
            bl_set_bit(g_fdxb_decoded, (size_t)out_bit,
                       bl_get_bit(g_fdxb_frame, src));
            out_bit++;
        }
    }
done:
    (void)0;
}

/*============================================================================*/
static void fdxb_begin_impl(void)
{
    memset(g_fdxb_frame, 0, sizeof(g_fdxb_frame));
    g_fdxb_bit_count = 0;
    g_fdxb_last_short = false;
    memset(g_fdxb_decoded, 0, sizeof(g_fdxb_decoded));
}

/*============================================================================*/
static bool fdxb_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (uint16_t i = 0; i < size; i++) {
        uint16_t dur = evt[i].t_us;

        if (dur >= FDXB_SHORT_MIN && dur <= FDXB_SHORT_MAX) {
            /* Short pulse */
            if (!g_fdxb_last_short) {
                g_fdxb_last_short = true;
            } else {
                /* Two shorts = bit 0 */
                fdxb_push_bit(false);
                g_fdxb_last_short = false;

                if (fdxb_can_be_decoded()) {
                    fdxb_decode();
                    memcpy(lfrfid_tag_info.uid, g_fdxb_decoded,
                           min(sizeof(lfrfid_tag_info.uid), FDX_B_DECODED_SIZE));
                    lfrfid_tag_info.bitrate = 32;
                    return true;
                }
            }
        } else if (dur >= FDXB_LONG_MIN && dur <= FDXB_LONG_MAX) {
            /* Long pulse */
            if (!g_fdxb_last_short) {
                /* One long = bit 1 */
                fdxb_push_bit(true);

                if (fdxb_can_be_decoded()) {
                    fdxb_decode();
                    memcpy(lfrfid_tag_info.uid, g_fdxb_decoded,
                           min(sizeof(lfrfid_tag_info.uid), FDX_B_DECODED_SIZE));
                    lfrfid_tag_info.bitrate = 32;
                    return true;
                }
            } else {
                /* Long after short = invalid, reset short flag */
                g_fdxb_last_short = false;
            }
        } else {
            /* Out of range: reset short flag */
            g_fdxb_last_short = false;
        }
    }

    return false;
}

/*============================================================================*/
static uint8_t *fdxb_get_data(void *proto) { (void)proto; return g_fdxb_decoded; }

/*============================================================================*/
static void fdxb_render_data(void *proto, char *result)
{
    (void)proto;

    /* FDX-B decoded layout (LSB first within each field per ISO 11784):
     * Bits 0-37:  National code (38 bits)
     * Bits 38-47: Country code (10 bits)
     * Bit 48:     Data block flag
     * Bit 49:     Animal flag
     */
    uint64_t national = bl_get_bits_64(g_fdxb_decoded, 0, 38);
    uint16_t country = bl_get_bits_16(g_fdxb_decoded, 38, 10);

    sprintf(result,
            "Country: %u\n"
            "Nat: %02X%02X%02X%02X%02X",
            country,
            g_fdxb_decoded[0], g_fdxb_decoded[1], g_fdxb_decoded[2],
            g_fdxb_decoded[3], g_fdxb_decoded[4]);
    (void)national;
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_fdx_b = {
    .name = "FDX-B",
    .manufacturer = "ISO 11784",
    .data_size = FDX_B_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)fdxb_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)fdxb_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)fdxb_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)fdxb_render_data,
};
