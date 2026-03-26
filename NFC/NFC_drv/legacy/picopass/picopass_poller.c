/* See COPYING.txt for license details. */

/*
 * picopass_poller.c — PicoPass / iCLASS card reader
 *
 * Full read sequence for PicoPass cards over the ST25R3916 using RFAL.
 * PicoPass uses ISO15693 physical layer but with proprietary commands
 * and a different CRC (init=0xE012, no final inversion).
 *
 * Key reference: Flipper Zero picopass app by @bettse,
 *                Proxmark3 RRG iclass.c
 */

#include "picopass_poller.h"
#include "picopass_cipher.h"
#include "picopass_keys.h"
#include "rfal_rf.h"
#include "rfal_nfcv.h"
#include "st25r3916.h"
#include "st25r3916_com.h"
#include "logger.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════
 * Raw transceive — PicoPass over ISO15693 PHY
 *
 * PicoPass frame format differs from standard ISO15693:
 *   - No FLAGS byte (RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL required)
 *   - CRC uses init=0xE012, no final inversion (not standard 0xFFFF)
 *   - CRC covers payload bytes AFTER the command byte, not the cmd itself
 *   - Some commands (ACTALL, READCHECK, CHECK) have no CRC at all
 *
 * Reference: bettse/picopass (Flipper Zero), Proxmark3 RRG iclass.c
 * ═══════════════════════════════════════════════════════════════════ */

#define PP_TX_BUF_SIZE  20U
#define PP_RX_BUF_SIZE  40U

/* FWT: 100000 fc (~7.4ms) — matches bettse's PICOPASS_POLLER_FWT_FC */
#define PP_FWT_FC       (100000U)

/* Flags for all PicoPass transceive: bypass ISO15693 FLAGS byte auto-adapt,
 * disable hardware CRC (we handle it in software with PicoPass init value),
 * keep CRC in RX buffer so we can verify it ourselves. */
#define PP_TRX_FLAGS    ( (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL    \
                        | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP      \
                        | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_MANUAL    \
                        | (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL \
                        | (uint32_t)RFAL_TXRX_FLAGS_AGC_ON )

/**
 * Raw PicoPass transceive. Sends tx_buf as-is (caller must include
 * any CRC bytes). Returns raw response including any CRC.
 */
static ReturnCode pp_raw_trx(const uint8_t *tx, uint16_t tx_len,
                             uint8_t *rx, uint16_t rx_size, uint16_t *rx_len)
{
    *rx_len = 0;
    return rfalTransceiveBlockingTxRx(
        (uint8_t *)tx, tx_len, rx, rx_size, rx_len, PP_TRX_FLAGS, PP_FWT_FC);
}

/**
 * Send a PicoPass command with CRC over the payload (after cmd byte).
 * Frame on wire: CMD(1) + payload(N) + CRC(2)
 * CRC covers payload only, NOT the command byte.
 */
static ReturnCode pp_send_with_crc(uint8_t cmd, const uint8_t *payload,
                                   uint16_t payload_len,
                                   uint8_t *rx, uint16_t rx_size,
                                   uint16_t *rx_len)
{
    uint8_t frame[PP_TX_BUF_SIZE];
    uint16_t pos = 0;

    if (1 + payload_len + 2 > PP_TX_BUF_SIZE) return RFAL_ERR_PARAM;

    frame[pos++] = cmd;
    if (payload_len > 0) {
        memcpy(&frame[pos], payload, payload_len);
        pos += payload_len;
    }

    /* CRC over payload only (everything after the command byte) */
    uint8_t crc[2];
    picopass_crc(payload, payload_len, crc);
    frame[pos++] = crc[0];
    frame[pos++] = crc[1];

    return pp_raw_trx(frame, pos, rx, rx_size, rx_len);
}

/**
 * Verify PicoPass CRC on a received response buffer.
 * CRC in response covers all data bytes (the card includes data + CRC).
 */
static bool pp_verify_rx_crc(const uint8_t *rx, uint16_t rx_len)
{
    if (rx_len < 3) return false;
    return picopass_crc_check(rx, rx_len);
}

/* ═══════════════════════════════════════════════════════════════════
 * PicoPass command implementations
 * ═══════════════════════════════════════════════════════════════════ */

static ReturnCode pp_cmd_actall(void)
{
    uint8_t cmd = PICOPASS_CMD_ACTALL;
    uint8_t rx[4];
    uint16_t rx_len = 0;

    /* ACTALL: 1 byte command, no CRC. Card responds with SOF only.
     * ST25R3916 reports SOF-only as incomplete/framing/CRC error. */
    ReturnCode err = pp_raw_trx(&cmd, 1, rx, sizeof(rx), &rx_len);
    if (err == RFAL_ERR_NONE || err == RFAL_ERR_INCOMPLETE_BYTE ||
        err == RFAL_ERR_CRC || err == RFAL_ERR_FRAMING ||
        err == RFAL_ERR_TIMEOUT) {
        return RFAL_ERR_NONE;
    }
    return err;
}

static ReturnCode pp_cmd_identify(uint8_t anti_csn[8])
{
    uint8_t cmd = PICOPASS_CMD_IDENTIFY;
    uint8_t rx[12]; /* ACSN(8) + CRC(2) */
    uint16_t rx_len = 0;

    /* IDENTIFY: just the command byte, no payload, no CRC on TX.
     * Response: ACSN(8) + CRC(2) */
    ReturnCode err = pp_raw_trx(&cmd, 1, rx, sizeof(rx), &rx_len);
    if (err != RFAL_ERR_NONE) return err;
    if (rx_len < 10) return RFAL_ERR_PROTO;
    if (!pp_verify_rx_crc(rx, 10)) return RFAL_ERR_CRC;

    memcpy(anti_csn, rx, 8);
    return RFAL_ERR_NONE;
}

static ReturnCode pp_cmd_select(const uint8_t anti_csn[8], uint8_t csn[8])
{
    uint8_t rx[12]; /* CSN(8) + CRC(2) */
    uint16_t rx_len = 0;

    /* SELECT: CMD + ACSN(8), CRC over ACSN only (not CMD).
     * Response: CSN(8) + CRC(2) */
    ReturnCode err = pp_send_with_crc(PICOPASS_CMD_SELECT, anti_csn, 8,
                                      rx, sizeof(rx), &rx_len);
    if (err != RFAL_ERR_NONE) return err;
    if (rx_len < 10) return RFAL_ERR_PROTO;
    if (!pp_verify_rx_crc(rx, 10)) return RFAL_ERR_CRC;

    memcpy(csn, rx, 8);
    return RFAL_ERR_NONE;
}

static ReturnCode pp_cmd_read_block(uint8_t block_num, uint8_t data[8])
{
    uint8_t rx[12]; /* DATA(8) + CRC(2) */
    uint16_t rx_len = 0;

    /* READ: CMD(0x0C) + block_num(1) + CRC(2), CRC over block_num only.
     * Same command byte as IDENTIFY — distinguished by having a payload.
     * Response: DATA(8) + CRC(2) */
    ReturnCode err = pp_send_with_crc(PICOPASS_CMD_READ, &block_num, 1,
                                      rx, sizeof(rx), &rx_len);
    if (err != RFAL_ERR_NONE) return err;
    if (rx_len < 10) return RFAL_ERR_PROTO;
    if (!pp_verify_rx_crc(rx, 10)) return RFAL_ERR_CRC;

    memcpy(data, rx, 8);
    return RFAL_ERR_NONE;
}

static ReturnCode pp_cmd_readcheck_kd(uint8_t block_num, uint8_t data[8])
{
    /* READCHECK_KD: no CRC in request OR response */
    uint8_t tx[2];
    uint8_t rx[8];
    uint16_t rx_len = 0;

    tx[0] = PICOPASS_CMD_READCHECK_KD;
    tx[1] = block_num;

    ReturnCode err = pp_raw_trx(tx, 2, rx, sizeof(rx), &rx_len);
    if (err != RFAL_ERR_NONE) return err;
    if (rx_len < 8) return RFAL_ERR_PROTO;

    memcpy(data, rx, 8);
    return RFAL_ERR_NONE;
}

static ReturnCode pp_cmd_check(const uint8_t nr[4], const uint8_t mac[4],
                               uint8_t chip_response[4])
{
    /* CHECK: CMD + NR(4) + MAC(4), no CRC. Response: 4 bytes, no CRC. */
    uint8_t tx[9];
    uint8_t rx[4];
    uint16_t rx_len = 0;

    tx[0] = PICOPASS_CMD_CHECK;
    memcpy(&tx[1], nr, 4);
    memcpy(&tx[5], mac, 4);

    ReturnCode err = pp_raw_trx(tx, 9, rx, sizeof(rx), &rx_len);
    if (err != RFAL_ERR_NONE) return err;
    if (rx_len < 4) return RFAL_ERR_PROTO;

    memcpy(chip_response, rx, 4);
    return RFAL_ERR_NONE;
}

/* ═══════════════════════════════════════════════════════════════════
 * Authentication with key dictionary
 * ═══════════════════════════════════════════════════════════════════ */

static bool pp_try_auth_with_div(const uint8_t *div_key_in, uint8_t div_key_out[8])
{
    uint8_t div_key[8];
    memcpy(div_key, div_key_in, 8);

    /* Step 2: READCHECK_KD on block 2 to get challenge (CCNR) */
    uint8_t ccnr_data[8];
    ReturnCode err = pp_cmd_readcheck_kd(PICOPASS_EPURSE_BLOCK, ccnr_data);
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] READCHECK_KD failed: %d\r\n", err);
        return false;
    }

    /* Step 3: Build CCNR = ePurse(8) + NR(4) where NR = 0 */
    uint8_t ccnr[12];
    memcpy(ccnr, ccnr_data, 8);
    memset(&ccnr[8], 0, 4); /* NR = 0000 */

    /* Step 4: Calculate reader MAC */
    uint8_t reader_mac[4];
    picopass_iclass_calc_mac(ccnr, div_key, reader_mac);

    /* Step 5: Send CHECK with NR + MAC */
    uint8_t nr[4] = {0, 0, 0, 0};
    uint8_t chip_resp[4];
    err = pp_cmd_check(nr, reader_mac, chip_resp);
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] CHECK failed: %d\r\n", err);
        return false;
    }

    /* Step 6: Verify chip response (optional — we trust it if CHECK didn't error) */
    memcpy(div_key_out, div_key, 8);
    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 * Public API: Full card read
 * ═══════════════════════════════════════════════════════════════════ */

PicopassReadResult picopass_poller_read(const rfalNfcDevice *dev, PicopassData *out)
{
    ReturnCode err;
    memset(out, 0, sizeof(*out));

    platformLog("[PP] Starting PicoPass read...\r\n");

    /* ─── ACTALL ─── */
    err = pp_cmd_actall();
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] ACTALL failed: %d\r\n", err);
        return PICOPASS_READ_ERR_NO_CARD;
    }

    /* ─── IDENTIFY ─── */
    uint8_t anti_csn[8];
    err = pp_cmd_identify(anti_csn);
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] IDENTIFY failed: %d\r\n", err);
        return PICOPASS_READ_ERR_NO_CARD;
    }
    platformLog("[PP] Anti-collision CSN found\r\n");

    /* ─── SELECT ─── */
    err = pp_cmd_select(anti_csn, out->csn);
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] SELECT failed: %d\r\n", err);
        return PICOPASS_READ_ERR_SELECT;
    }
    memcpy(out->blocks[PICOPASS_CSN_BLOCK].data, out->csn, 8);
    platformLog("[PP] Selected CSN: %02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                out->csn[0], out->csn[1], out->csn[2], out->csn[3],
                out->csn[4], out->csn[5], out->csn[6], out->csn[7]);

    /* ─── READ config blocks (1, 2, 5) ─── */
    err = pp_cmd_read_block(PICOPASS_CFG_BLOCK, out->cfg);
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] Read config failed: %d\r\n", err);
        return PICOPASS_READ_ERR_READ;
    }
    memcpy(out->blocks[PICOPASS_CFG_BLOCK].data, out->cfg, 8);

    err = pp_cmd_read_block(PICOPASS_EPURSE_BLOCK, out->epurse);
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] Read ePurse failed: %d\r\n", err);
        return PICOPASS_READ_ERR_READ;
    }
    memcpy(out->blocks[PICOPASS_EPURSE_BLOCK].data, out->epurse, 8);

    err = pp_cmd_read_block(PICOPASS_AIA_BLOCK, out->aia);
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] Read AIA failed: %d\r\n", err);
        return PICOPASS_READ_ERR_READ;
    }
    memcpy(out->blocks[PICOPASS_AIA_BLOCK].data, out->aia, 8);

    /* ─── Determine security mode ─── */
    uint8_t fuses = out->cfg[7];
    uint8_t app_limit = out->cfg[0];
    if (app_limit > PICOPASS_MAX_APP_LIMIT) app_limit = PICOPASS_MAX_APP_LIMIT;
    out->block_count = app_limit;

    /* Check if AIA indicates legacy (0xFF...FF) */
    uint8_t aia_ff[8];
    memset(aia_ff, 0xFF, 8);
    out->legacy = (memcmp(out->aia, aia_ff, 8) == 0);

    if ((fuses & PICOPASS_FUSE_CRYPT10) == 0x00 ||
        (fuses & PICOPASS_FUSE_CRYPT10) == PICOPASS_FUSE_CRYPT0) {
        /* Non-secured or auth-disabled */
        out->secured = false;
        platformLog("[PP] Card is NOT secured (fuses=0x%02X)\r\n", fuses);
    } else {
        out->secured = true;
        platformLog("[PP] Card is SECURED (fuses=0x%02X), attempting auth...\r\n", fuses);

        /* Try all known keys — standard diversification first, then elite */
        out->auth_success = false;

        /* Pass 1: Standard key diversification */
        for (int k = 0; k < PICOPASS_NUM_KEYS && !out->auth_success; k++) {
            uint8_t div_key[8];
            picopass_iclass_calc_div_key(out->csn, picopass_key_list[k], div_key);
            platformLog("[PP] Trying standard key %d...\r\n", k);
            if (pp_try_auth_with_div(div_key, out->div_key)) {
                out->auth_success = true;
                platformLog("[PP] Auth SUCCESS with standard key %d\r\n", k);
                break;
            }
            /* After failed auth, card goes to HALT — need re-select */
            pp_cmd_actall();
            pp_cmd_identify(anti_csn);
            pp_cmd_select(anti_csn, out->csn);
        }

        /* Pass 2: Elite key diversification */
        for (int k = 0; k < PICOPASS_NUM_KEYS && !out->auth_success; k++) {
            uint8_t div_key[8];
            picopass_iclass_calc_div_key_elite(out->csn, picopass_key_list[k], div_key);
            platformLog("[PP] Trying elite key %d...\r\n", k);
            if (pp_try_auth_with_div(div_key, out->div_key)) {
                out->auth_success = true;
                platformLog("[PP] Auth SUCCESS with elite key %d\r\n", k);
                break;
            }
            pp_cmd_actall();
            pp_cmd_identify(anti_csn);
            pp_cmd_select(anti_csn, out->csn);
        }

        if (!out->auth_success) {
            platformLog("[PP] Auth FAILED with all known keys (standard + elite)\r\n");
            return PICOPASS_READ_ERR_AUTH;
        }
    }

    /* ─── READ all application blocks ─── */
    platformLog("[PP] Reading %u blocks...\r\n", app_limit);
    for (uint8_t blk = 0; blk < app_limit; blk++) {
        /* Skip key blocks — always return 0xFF */
        if (blk == PICOPASS_KD_BLOCK || blk == PICOPASS_KC_BLOCK) {
            memset(out->blocks[blk].data, 0xFF, 8);
            continue;
        }
        /* Skip blocks we already read */
        if (blk <= PICOPASS_CSN_BLOCK || blk == PICOPASS_CFG_BLOCK ||
            blk == PICOPASS_EPURSE_BLOCK || blk == PICOPASS_AIA_BLOCK) {
            continue; /* Already populated */
        }

        err = pp_cmd_read_block(blk, out->blocks[blk].data);
        if (err != RFAL_ERR_NONE) {
            platformLog("[PP] Read block %u failed: %d\r\n", blk, err);
            /* Don't fail — just skip this block */
        }
    }

    platformLog("[PP] PicoPass read complete: %u blocks, secured=%d, legacy=%d\r\n",
                app_limit, out->secured, out->legacy);
    return PICOPASS_READ_OK;
}

/* ═══════════════════════════════════════════════════════════════════
 * Write a single block (UPDATE command)
 * ═══════════════════════════════════════════════════════════════════ */

PicopassReadResult picopass_poller_write_block(uint8_t block_num,
                                               const uint8_t data[8],
                                               const uint8_t div_key[8])
{
    if (block_num == PICOPASS_CSN_BLOCK) {
        platformLog("[PP] Cannot write CSN (block 0)\r\n");
        return PICOPASS_READ_ERR_READ;
    }

    /* UPDATE: CMD(1) + ADDR(1) + DATA(8) + MAC(4), no CRC on TX per bettse.
     * Response: DATA(8) + CRC(2) */
    uint8_t payload[13]; /* ADDR(1) + DATA(8) + MAC(4) */
    uint8_t rx[12];
    uint16_t rx_len = 0;

    payload[0] = block_num;
    memcpy(&payload[1], data, 8);

    /* Compute update MAC */
    uint8_t mac[4];
    picopass_iclass_calc_update_mac(block_num, data, div_key, mac);
    memcpy(&payload[9], mac, 4);

    /* UPDATE has no CRC on TX — send raw CMD + payload */
    uint8_t tx[14];
    tx[0] = PICOPASS_CMD_UPDATE;
    memcpy(&tx[1], payload, 13);

    ReturnCode err = pp_raw_trx(tx, 14, rx, sizeof(rx), &rx_len);
    if (err != RFAL_ERR_NONE) {
        platformLog("[PP] UPDATE block %u failed: %d\r\n", block_num, err);
        return PICOPASS_READ_ERR_READ;
    }

    /* Verify response contains the written data */
    if (rx_len >= 10 && picopass_crc_check(rx, 10)) {
        if (memcmp(rx, data, 8) == 0) {
            platformLog("[PP] Block %u written OK\r\n", block_num);
            return PICOPASS_READ_OK;
        }
    }

    platformLog("[PP] Block %u write verify failed\r\n", block_num);
    return PICOPASS_READ_ERR_READ;
}
