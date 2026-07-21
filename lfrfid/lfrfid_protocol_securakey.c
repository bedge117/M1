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
#include "t5577.h"
#include "bit_lib.h"

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
/* T5577 write support — EXACT INVERSE of this file's securakey_decode().
 *
 * The clone requirement is round-trip fidelity on M1 itself: a card captured by
 * securakey_execute_impl()/securakey_decode() and then written by this encoder
 * must decode back to the identical g_securakey_decoded. It is therefore NOT a
 * port of Flipper's structured encoder_start (whose protocol->data layout does
 * not match M1's decoder); it reconstructs the raw on-wire frame by inverting
 * precisely what securakey_decode() did.
 *
 * securakey_decode() reads frame bits 19..95, skipping the Always-0 "parity"
 * slots at every position where (pos-2)%9==0 (i.e. pos 20,29,38,47,56,65,74,83,
 * 92), copying the survivors in order into 48 output bits and stopping at 48.
 * The resulting position map (frame_pos -> decoded_bit) is:
 *     out0        <- frame pos 19               (lone bit before first parity slot)
 *     out1 .. 40  <- frame pos 21 .. 65         (five regular 8-data + 1-parity groups)
 *     out41 .. 47 <- frame pos 66 .. 72         (final 7-bit tail; decode caps at 48)
 *
 * The inverse re-inserts the 19-bit preamble and the Always-0 parity slots
 * (mirroring the GProxII port's use of bit_lib_add_parity to rebuild a frame):
 *     - preamble (bits 0..18)  = 0x3FE60  (RKKT pattern the decoder recognises)
 *     - out0                   -> bit_lib_copy_bits at pos 19  (parity slot 20 stays 0)
 *     - out1..40               -> bit_lib_add_parity: 5 groups of 8 data + 1 zero
 *                                 parity, laid out at pos 21..65
 *     - out41..47              -> bit_lib_copy_bits at pos 66..72
 * All parity slots (20,29,38,47,56,65 via add_parity; 74,83,92 via memset) are 0,
 * matching what securakey_can_be_decoded() expects.
 *
 * Frame is always the full 96 bits (3 data blocks). M1's decoder requires
 * bit_count >= 96 to decode at all, so a 64-bit RKKTH-style 2-block write could
 * never read back on M1; the single 96-bit path is the only round-trippable form
 * and the true inverse of securakey_decode() (which has no 64-bit path).
 *
 * Data source is the full 6-byte decoded buffer g_securakey_decoded (48 bits >
 * 5 bytes, so tag->uid is not used).
 */
/*============================================================================*/
#define SECURAKEY_ENC_PREAMBLE 0x3FE60u /* 0b0111111111001100000 (19-bit RKKT) */

static void securakey_build_frame(const uint8_t *decoded, uint8_t *encoded)
{
    memset(encoded, 0, SECURAKEY_ENCODED_SIZE);

    /* 19-bit preamble at frame bits 0..18 (MSB-first). */
    bit_lib_set_bits(encoded, 0, (uint8_t)(SECURAKEY_ENC_PREAMBLE >> 11), 8);  /* bits 0..7  */
    bit_lib_set_bits(encoded, 8, (uint8_t)(SECURAKEY_ENC_PREAMBLE >> 3), 8);   /* bits 8..15 */
    bit_lib_set_bits(encoded, 16, (uint8_t)(SECURAKEY_ENC_PREAMBLE & 0x7), 3); /* bits 16..18 */

    /* Inverse of securakey_decode's parity-strip (see block comment above). */
    bit_lib_copy_bits(encoded, 19, 1, decoded, 0);   /* out0        -> pos 19        */
    bit_lib_add_parity(
        decoded, 1, encoded, 21, 40, 9, BitLibParityAlways0); /* out1..40 -> pos 21..65 */
    bit_lib_copy_bits(encoded, 66, 7, decoded, 41);  /* out41..47   -> pos 66..72    */
}

void securakey_write_begin(void* protocol, void* data)
{
    LFRFID_TAG_INFO* tag_data = (LFRFID_TAG_INFO*)protocol;
    LFRFIDProgram*   write    = (LFRFIDProgram*)data;
    uint8_t          encoded[SECURAKEY_ENCODED_SIZE];

    (void)tag_data;

    securakey_build_frame(g_securakey_decoded, encoded);

    if(write && write->type == LFRFIDProgramTypeT5577) {
        /* Manchester | RF/40, 96-bit frame = 3 data blocks (block 0 config). */
        write->t5577.block_data[0] =
            T5577_MOD_MANCHESTER | T5577_BITRATE_RF_40 | T5577_TRANS_BL_1_3;
        write->t5577.block_data[1] = bit_lib_get_bits_32(encoded, 0, 32);
        write->t5577.block_data[2] = bit_lib_get_bits_32(encoded, 32, 32);
        write->t5577.block_data[3] = bit_lib_get_bits_32(encoded, 64, 32);
        write->t5577.max_blocks = 4;
    }
}

void securakey_write_send(void* proto)
{
    (void)proto;
    t5577_execute_write(lfrfid_program, 0);
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
    .write   = {
        .begin = (lfrfidProtocolWriteBegin)securakey_write_begin,
        .send  = (lfrfidProtocolWriteSend)securakey_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)securakey_render_data,
};
