/* See COPYING.txt for license details. */

/*
 * picopass_listener.c — PicoPass / iCLASS card emulation
 *
 * Emulates a PicoPass card using the ST25R3916 in NFC-V mode.
 * Uses rfalTransceiveBlockingTxRx with manual CRC for PicoPass framing.
 *
 * The emulation state machine responds to reader commands:
 *   ACTALL → SOF
 *   IDENTIFY → anti-collision CSN
 *   SELECT → real CSN
 *   READCHECK → ePurse (challenge)
 *   CHECK → verify MAC, return chip response
 *   READ → return block data
 *
 * Based on the Flipper Zero picopass_listener by @bettse.
 */

#include "picopass_listener.h"
#include "picopass_cipher.h"
#include "common/nfc_ctx.h"
#include "rfal_rf.h"
#include "rfal_nfcv.h"
#include "rfal_nfc.h"
#include "st25r3916.h"
#include "st25r3916_com.h"
#include "logger.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════
 * Emulation state
 * ═══════════════════════════════════════════════════════════════════ */

typedef enum {
    PP_EMU_STATE_IDLE = 0,
    PP_EMU_STATE_SELECTED,
    PP_EMU_STATE_AUTHED,
    PP_EMU_STATE_HALT,
} PpEmuState_t;

static struct {
    PicopassBlock blocks[PICOPASS_MAX_APP_LIMIT];
    uint8_t       block_count;
    uint8_t       csn[8];          /* Block 0 */
    uint8_t       anti_csn[8];     /* Rotated CSN for anti-collision */
    uint8_t       cfg[8];          /* Block 1 */
    uint8_t       epurse[8];       /* Block 2 */
    uint8_t       div_key[8];      /* Derived from Kd in block 3 */
    bool          secured;
    PpEmuState_t  state;
    bool          running;
} s_emu;

/* TX/RX buffers for listener */
static uint8_t s_emu_tx[40];
static uint8_t s_emu_rx[40];

/* ═══════════════════════════════════════════════════════════════════
 * Anti-collision CSN generation
 * ═══════════════════════════════════════════════════════════════════ */

static void compute_anti_csn(const uint8_t csn[8], uint8_t anti_csn[8])
{
    for (int i = 0; i < 8; i++) {
        anti_csn[i] = (uint8_t)((csn[i] >> 3) | (csn[(i + 1) % 8] << 5));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * TX helper — send response with optional PicoPass CRC
 * ═══════════════════════════════════════════════════════════════════ */

static bool pp_emu_tx(const uint8_t *data, uint16_t len, bool with_crc)
{
    uint16_t frame_len = len;
    if (frame_len > sizeof(s_emu_tx) - 2) return false;

    memcpy(s_emu_tx, data, len);

    if (with_crc) {
        uint8_t crc[2];
        picopass_crc(s_emu_tx, frame_len, crc);
        s_emu_tx[frame_len++] = crc[0];
        s_emu_tx[frame_len++] = crc[1];
    }

    uint32_t flags = RFAL_TXRX_FLAGS_CRC_TX_MANUAL
                   | RFAL_TXRX_FLAGS_CRC_RX_KEEP
                   | RFAL_TXRX_FLAGS_CRC_RX_MANUAL;

    ReturnCode err = rfalTransceiveBlockingTx(
        s_emu_tx, frame_len, NULL, 0, NULL, flags,
        rfalConvMsTo1fc(5));

    return (err == RFAL_ERR_NONE || err == RFAL_ERR_LINK_LOSS);
}

/* ═══════════════════════════════════════════════════════════════════
 * Command handlers
 * ═══════════════════════════════════════════════════════════════════ */

static bool handle_actall(void)
{
    /* Respond with SOF only — the ST25R3916 sends this automatically
     * when we don't transmit any data. Just reset state. */
    s_emu.state = PP_EMU_STATE_IDLE;
    return true;
}

static bool handle_identify(void)
{
    /* Return anti-collision CSN + CRC */
    return pp_emu_tx(s_emu.anti_csn, 8, true);
}

static bool handle_select(const uint8_t *rx, uint16_t rx_len)
{
    /* SELECT: CMD(1) + ACSN(8) + CRC(2) = 11 bytes */
    if (rx_len < 9) return false;

    /* Check if the ACSN matches ours */
    if (memcmp(&rx[1], s_emu.anti_csn, 8) != 0) {
        return false; /* Not addressed to us */
    }

    s_emu.state = PP_EMU_STATE_SELECTED;
    /* Return real CSN + CRC */
    return pp_emu_tx(s_emu.csn, 8, true);
}

static bool handle_read(const uint8_t *rx, uint16_t rx_len)
{
    /* READ: CMD(1) + ADDR(1) + CRC(2) = 4 bytes */
    if (rx_len < 2) return false;
    uint8_t block_num = rx[1];

    if (block_num >= s_emu.block_count) {
        return false;
    }

    /* Key blocks always return 0xFF */
    if (block_num == PICOPASS_KD_BLOCK || block_num == PICOPASS_KC_BLOCK) {
        uint8_t ff_block[8];
        memset(ff_block, 0xFF, 8);
        return pp_emu_tx(ff_block, 8, true);
    }

    return pp_emu_tx(s_emu.blocks[block_num].data, 8, true);
}

static bool handle_readcheck(const uint8_t *rx, uint16_t rx_len)
{
    /* READCHECK: CMD(1) + ADDR(1), NO CRC in request or response */
    if (rx_len < 2) return false;
    uint8_t block_num = rx[1];

    if (block_num >= s_emu.block_count) return false;

    /* Return block data WITHOUT CRC */
    return pp_emu_tx(s_emu.blocks[block_num].data, 8, false);
}

static bool handle_check(const uint8_t *rx, uint16_t rx_len)
{
    /* CHECK: CMD(1) + NR(4) + MAC(4) = 9 bytes, no CRC */
    if (rx_len < 9) return false;

    /* Build CCNR: ePurse(8) + NR(4) */
    uint8_t ccnr[12];
    memcpy(ccnr, s_emu.epurse, 8);
    memcpy(&ccnr[8], &rx[1], 4); /* NR from reader */

    /* Verify reader MAC */
    uint8_t expected_mac[4];
    picopass_iclass_calc_mac(ccnr, s_emu.div_key, expected_mac);

    if (memcmp(expected_mac, &rx[5], 4) != 0) {
        platformLog("[PP-EMU] CHECK: MAC mismatch\r\n");
        s_emu.state = PP_EMU_STATE_IDLE;
        return false;
    }

    /* Compute tag response MAC */
    uint8_t tag_mac[4];
    picopass_iclass_calc_tag_mac(ccnr, s_emu.div_key, tag_mac);

    s_emu.state = PP_EMU_STATE_AUTHED;
    platformLog("[PP-EMU] CHECK: auth OK\r\n");

    /* Return chip response (4 bytes, no CRC) */
    return pp_emu_tx(tag_mac, 4, false);
}

static bool handle_halt(void)
{
    s_emu.state = PP_EMU_STATE_HALT;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════ */

bool picopass_listener_init(void)
{
    nfc_run_ctx_t *c = nfc_ctx_get();
    if (!c || !c->dump.has_dump || !c->dump.data) {
        platformLog("[PP-EMU] No dump data for emulation\r\n");
        return false;
    }
    if (c->head.family != M1NFC_FAM_ICLASS) {
        platformLog("[PP-EMU] Not a PicoPass card\r\n");
        return false;
    }

    /* Load blocks from dump */
    memset(&s_emu, 0, sizeof(s_emu));
    s_emu.block_count = (uint8_t)c->dump.unit_count;
    if (s_emu.block_count > PICOPASS_MAX_APP_LIMIT)
        s_emu.block_count = PICOPASS_MAX_APP_LIMIT;

    for (uint8_t i = 0; i < s_emu.block_count; i++) {
        memcpy(s_emu.blocks[i].data, &c->dump.data[i * 8], 8);
    }

    /* Cache key fields */
    memcpy(s_emu.csn, s_emu.blocks[PICOPASS_CSN_BLOCK].data, 8);
    memcpy(s_emu.cfg, s_emu.blocks[PICOPASS_CFG_BLOCK].data, 8);
    memcpy(s_emu.epurse, s_emu.blocks[PICOPASS_EPURSE_BLOCK].data, 8);

    /* Compute anti-collision CSN */
    compute_anti_csn(s_emu.csn, s_emu.anti_csn);

    /* Check security mode */
    uint8_t fuses = s_emu.cfg[7];
    s_emu.secured = ((fuses & PICOPASS_FUSE_CRYPT10) == PICOPASS_FUSE_CRYPT1 ||
                     (fuses & PICOPASS_FUSE_CRYPT10) == PICOPASS_FUSE_CRYPT10);

    /* If we have a diversified key stored (from the read), use it.
     * The key is stored in the PicoPass dump context — block 3 (Kd) is
     * always 0xFF in dumps, so we use the div_key saved during read. */
    /* For now, use zeros — the reader must authenticate with us,
     * and we need the key from the original read. This will be
     * populated when file load parses the key. */
    memset(s_emu.div_key, 0, 8);

    s_emu.state = PP_EMU_STATE_IDLE;
    s_emu.running = true;

    /* Initialize RFAL in NFC-V mode for listening */
    ReturnCode err = rfalNfcInitialize();
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP-EMU] RFAL init failed: %d\r\n", err);
        return false;
    }

    /* Configure for NFC-V (ISO15693) */
    err = rfalNfcvPollerInitialize();
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP-EMU] NFC-V init failed: %d\r\n", err);
        return false;
    }

    platformLog("[PP-EMU] Ready. CSN=%02X%02X%02X%02X%02X%02X%02X%02X blocks=%u secured=%d\r\n",
                s_emu.csn[0], s_emu.csn[1], s_emu.csn[2], s_emu.csn[3],
                s_emu.csn[4], s_emu.csn[5], s_emu.csn[6], s_emu.csn[7],
                s_emu.block_count, s_emu.secured);

    return true;
}

void picopass_listener_process(void)
{
    if (!s_emu.running) return;

    /* Poll for incoming data — in transparent mode we receive raw frames.
     * For now, use blocking RX with short timeout to poll. */
    uint16_t rx_len = 0;
    uint32_t flags = RFAL_TXRX_FLAGS_CRC_TX_MANUAL
                   | RFAL_TXRX_FLAGS_CRC_RX_KEEP
                   | RFAL_TXRX_FLAGS_CRC_RX_MANUAL;

    ReturnCode err = rfalTransceiveBlockingTxRx(
        NULL, 0, s_emu_rx, sizeof(s_emu_rx), &rx_len,
        flags, rfalConvMsTo1fc(50));

    if (err != RFAL_ERR_NONE || rx_len == 0) {
        return; /* No data — continue polling */
    }

    /* Dispatch command */
    uint8_t cmd = s_emu_rx[0];

    switch (cmd) {
    case PICOPASS_CMD_ACTALL:
        handle_actall();
        break;

    case PICOPASS_CMD_IDENTIFY:
        /* IDENTIFY (1 byte) vs READ (4 bytes) — distinguished by length */
        if (rx_len <= 3) {
            handle_identify();
        } else {
            handle_read(s_emu_rx, rx_len);
        }
        break;

    case PICOPASS_CMD_SELECT:
        handle_select(s_emu_rx, rx_len);
        break;

    case PICOPASS_CMD_READCHECK_KD:
    case PICOPASS_CMD_READCHECK_KC:
        handle_readcheck(s_emu_rx, rx_len);
        break;

    case PICOPASS_CMD_CHECK:
        handle_check(s_emu_rx, rx_len);
        break;

    case PICOPASS_CMD_READ4:
        /* READ4: return 4 consecutive blocks */
        if (rx_len >= 2) {
            uint8_t start = s_emu_rx[1];
            uint8_t buf[32];
            for (int i = 0; i < 4 && (start + i) < s_emu.block_count; i++) {
                uint8_t blk = start + (uint8_t)i;
                if (blk == PICOPASS_KD_BLOCK || blk == PICOPASS_KC_BLOCK)
                    memset(&buf[i * 8], 0xFF, 8);
                else
                    memcpy(&buf[i * 8], s_emu.blocks[blk].data, 8);
            }
            pp_emu_tx(buf, 32, true);
        }
        break;

    case PICOPASS_CMD_UPDATE:
        /* UPDATE: CMD(1) + ADDR(1) + DATA(8) + SIGN(4)/CRC(2) */
        if (rx_len >= 10) {
            uint8_t block_num = s_emu_rx[1];
            if (block_num < s_emu.block_count &&
                block_num != PICOPASS_CSN_BLOCK) {
                memcpy(s_emu.blocks[block_num].data, &s_emu_rx[2], 8);
                /* Update cached fields */
                if (block_num == PICOPASS_EPURSE_BLOCK)
                    memcpy(s_emu.epurse, &s_emu_rx[2], 8);
                /* Respond with the written data + CRC */
                pp_emu_tx(s_emu.blocks[block_num].data, 8, true);
            }
        }
        break;

    case PICOPASS_CMD_HALT:
        handle_halt();
        break;

    default:
        /* Unknown command — ignore */
        break;
    }
}

void picopass_listener_stop(void)
{
    s_emu.running = false;
    s_emu.state = PP_EMU_STATE_IDLE;
    platformLog("[PP-EMU] Stopped\r\n");
}
