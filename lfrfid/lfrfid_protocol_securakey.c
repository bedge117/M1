/* See COPYING.txt for license details. */

/*
 * LF RFID (125 kHz) — Securakey protocol decoder
 *
 * Based on the Flipper Zero Securakey protocol approach.
 * ASK/Manchester modulation, RF/40, 96-bit frame (RKKT) or 64-bit (RKKTH).
 * Preamble: 19 bits with multiple format patterns.
 * Parity: every 9th bit starting at bit 2 must be 0.
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

/* 19-bit preamble patterns */
#define SECURAKEY_PRE_RKKTH_PLAIN  0x3FE00  /* 0b0111111111000000000 = plaintext RKKTH */
#define SECURAKEY_PRE_RKKT_26      0x3FE5A  /* 0b0111111111001011010 = 26-bit RKKT */
#define SECURAKEY_PRE_RKKT_32      0x3FE60  /* 0b0111111111001100000 = 32-bit RKKT */

/***************************** V A R I A B L E S ******************************/

static manch_decoder_t g_securakey_dec;
static uint8_t g_securakey_decoded[SECURAKEY_DECODED_SIZE];

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/* Securakey: 96-bit frame (RKKT) or 64-bit (RKKTH)                          */
/* 19-bit preamble, parity every 9th bit from bit 2                          */
/*============================================================================*/
static bool securakey_can_be_decoded(const manch_decoder_t *d)
{
    if (d->bit_count < 96)
        return false;

    /* Extract 19-bit preamble */
    uint32_t preamble = bl_get_bits_32(d->frame_buffer, 0, 19);

    bool valid_preamble = false;
    int data_bits = 0;

    if (preamble == SECURAKEY_PRE_RKKTH_PLAIN) {
        valid_preamble = true;
        data_bits = 54;  /* RKKTH: 64 - 19 preamble + some structure */
    } else if (preamble == SECURAKEY_PRE_RKKT_26) {
        valid_preamble = true;
        data_bits = 90;  /* 26-bit RKKT */
    } else if (preamble == SECURAKEY_PRE_RKKT_32) {
        valid_preamble = true;
        data_bits = 90;  /* 32-bit RKKT */
    }

    if (!valid_preamble)
        return false;

    /* Parity check: every 9th bit starting at bit 2 must be 0 */
    int check_bits = (preamble == SECURAKEY_PRE_RKKTH_PLAIN) ? 54 : 90;
    for (int pos = 2; pos < 2 + check_bits; pos += 9) {
        if ((size_t)pos < 96 && bl_get_bit(d->frame_buffer, (size_t)pos))
            return false;
    }

    (void)data_bits;
    return true;
}

/*============================================================================*/
/* Decode: extract 48 bits (6 bytes) of data                                  */
/*============================================================================*/
static void securakey_decode(const manch_decoder_t *d)
{
    memset(g_securakey_decoded, 0, SECURAKEY_DECODED_SIZE);

    /* Extract data bits after preamble, skipping parity bits (every 9th from bit 2) */
    int out_bit = 0;
    for (int pos = 19; pos < 96 && out_bit < SECURAKEY_DECODED_SIZE * 8; pos++) {
        /* Skip parity positions (every 9th bit starting at bit 2) */
        if (pos >= 2 && ((pos - 2) % 9) == 0)
            continue;

        bl_set_bit(g_securakey_decoded, (size_t)out_bit,
                   bl_get_bit(d->frame_buffer, (size_t)pos));
        out_bit++;
    }
}

/*============================================================================*/
static void securakey_begin_impl(void)
{
    manch_init(&g_securakey_dec, SECURAKEY_HALF_BIT_US, 96);
    memset(g_securakey_decoded, 0, sizeof(g_securakey_decoded));
}

/*============================================================================*/
static bool securakey_execute_impl(void *proto, uint16_t size)
{
    lfrfid_evt_t *evt = (lfrfid_evt_t *)proto;

    manch_feed_events(&g_securakey_dec, evt, (uint8_t)size);

    if (securakey_can_be_decoded(&g_securakey_dec)) {
        securakey_decode(&g_securakey_dec);
        memcpy(lfrfid_tag_info.uid, g_securakey_decoded,
               min(sizeof(lfrfid_tag_info.uid), SECURAKEY_DECODED_SIZE));
        lfrfid_tag_info.bitrate = 40;
        return true;
    }

    return false;
}

/*============================================================================*/
static uint8_t *securakey_get_data(void *proto) { (void)proto; return g_securakey_decoded; }

/*============================================================================*/
static void securakey_render_data(void *proto, char *result)
{
    (void)proto;
    sprintf(result,
            "Securakey\n"
            "Hex: %02X%02X%02X%02X%02X%02X",
            g_securakey_decoded[0], g_securakey_decoded[1],
            g_securakey_decoded[2], g_securakey_decoded[3],
            g_securakey_decoded[4], g_securakey_decoded[5]);
}

/*============================================================================*/
const LFRFIDProtocolBase protocol_securakey = {
    .name = "Securakey",
    .manufacturer = "Securakey",
    .data_size = SECURAKEY_DECODED_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)securakey_get_data,
    .decoder = {
        .begin   = (lfrfidProtocolDecoderBegin)securakey_begin_impl,
        .execute = (lfrfidProtocolDecoderExecute)securakey_execute_impl,
    },
    .encoder = { .begin = NULL, .send = NULL },
    .write   = { .begin = NULL, .send = NULL },
    .render_data = (lfrfidProtocolRenderData)securakey_render_data,
};
