/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Jablotron protocol decoder
 *
 * Based on the Flipper Zero Jablotron protocol approach.
 * ASK/BI-PHASE modulation, RF/64, 80-bit frame (64 encoded + 16 overlap).
 * Two shorts = bit 0, one long = bit 1. Edge polarity ignored.
 * Preamble: 16 bits of all 1s (0xFFFF) at bit 0 and bit 64
 * Checksum: sum of 5 bytes at bits 16-55, XOR 0x3A, compare with byte at bits 56-63
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

static uint8_t g_jab_encoded[JABLOTRON_ENCODED_SIZE];
static uint8_t g_jab_decoded[JABLOTRON_DECODED_SIZE];
static bool    g_jab_last_short;

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Jablotron checksum: sum of 5 bytes at bits 16-55, XOR 0x3A                 */
/*============================================================================*/
static uint8_t jablotron_checksum(const uint8_t *data)
{
    uint8_t chk = 0;
    for (int i = 0; i < 5; i++)
        chk += bl_get_bits(data, 16 + i * 8, 8);
    return chk ^ 0x3A;
}

/*============================================================================*/
/* Jablotron validation: Flipper-exact preamble + checksum                    */
/*============================================================================*/
static bool jablotron_can_be_decoded(const uint8_t *data)
{
    /* Check 16 bits of all 1s at bit 0 and bit 64 */
    if (bl_get_bits_16(data, 0, 16) != 0xFFFF)
        return false;
    if (bl_get_bits_16(data, 64, 16) != 0xFFFF)
        return false;

    /* Checksum byte at bits 56-63 must match computed checksum */
    uint8_t expected = bl_get_bits(data, 56, 8);
    uint8_t computed = jablotron_checksum(data);
    if (computed != expected)
        return false;

    return true;
}

/*============================================================================*/
static void jablotron_decode(const uint8_t *data, uint8_t *decoded)
{
    /* Decoded data is 5 bytes from bit 16 to bit 55 */
    for (int i = 0; i < JABLOTRON_DECODED_SIZE; i++)
        decoded[i] = bl_get_bits(data, 16 + i * 8, 8);
}

/*============================================================================*/
static void jablotron_begin_impl(void)
{
    memset(g_jab_encoded, 0, sizeof(g_jab_encoded));
    memset(g_jab_decoded, 0, sizeof(g_jab_decoded));
    g_jab_last_short = false;
}

/*============================================================================*/
/* Bi-phase decoder: two shorts = bit 0, one long = bit 1                     */
/* Edge polarity is ignored for bit extraction                                */
/*============================================================================*/
static bool jablotron_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    for (int i = 0; i < size; i++) {
        uint16_t dur = evt[i].t_us;

        if (dur >= (JABLOTRON_SHORT_US - JABLOTRON_JITTER_US) &&
            dur <= (JABLOTRON_SHORT_US + JABLOTRON_JITTER_US)) {
            /* SHORT pulse */
            if (!g_jab_last_short) {
                g_jab_last_short = true;
            } else {
                /* Two consecutive shorts = bit 0 */
                bl_push_bit(g_jab_encoded, JABLOTRON_ENCODED_SIZE, false);
                g_jab_last_short = false;

                if (jablotron_can_be_decoded(g_jab_encoded)) {
                    jablotron_decode(g_jab_encoded, g_jab_decoded);
                    memcpy(lfrfid_tag_info.uid, g_jab_decoded,
                           min(sizeof(lfrfid_tag_info.uid), JABLOTRON_DECODED_SIZE));
                    lfrfid_tag_info.bitrate = 64;
                    return true;
                }
            }
        } else if (dur >= (JABLOTRON_LONG_US - JABLOTRON_JITTER_US) &&
                   dur <= (JABLOTRON_LONG_US + JABLOTRON_JITTER_US)) {
            /* LONG pulse */
            if (!g_jab_last_short) {
                /* One long = bit 1 */
                bl_push_bit(g_jab_encoded, JABLOTRON_ENCODED_SIZE, true);

                if (jablotron_can_be_decoded(g_jab_encoded)) {
                    jablotron_decode(g_jab_encoded, g_jab_decoded);
                    memcpy(lfrfid_tag_info.uid, g_jab_decoded,
                           min(sizeof(lfrfid_tag_info.uid), JABLOTRON_DECODED_SIZE));
                    lfrfid_tag_info.bitrate = 64;
                    return true;
                }
            } else {
                /* Long after a short = reset short tracking */
                g_jab_last_short = false;
            }
        } else {
            /* Out of range — reset short tracking */
            g_jab_last_short = false;
        }
    }

    return false;
}

/*============================================================================*/
static uint8_t *jablotron_get_data(void *proto) { (void)proto; return g_jab_decoded; }

/*============================================================================*/
static void jablotron_render_data(void *proto, char *result)
{
    (void)proto;
    uint64_t code = 0;
    for (int i = 0; i < JABLOTRON_DECODED_SIZE; i++)
        code = (code << 8) | g_jab_decoded[i];
    sprintf(result,
            "CN: %02X%02X%02X%02X%02X\n"
            "Hex: %02X%02X%02X%02X%02X",
            g_jab_decoded[0], g_jab_decoded[1],
            g_jab_decoded[2], g_jab_decoded[3],
            g_jab_decoded[4],
            g_jab_decoded[0], g_jab_decoded[1],
            g_jab_decoded[2], g_jab_decoded[3],
            g_jab_decoded[4]);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_jablotron = {
    .name = "Jablotron",
    .manufacturer = "Jablotron",
    .data_size = JABLOTRON_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)jablotron_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)jablotron_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)jablotron_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)jablotron_render_data,
};
