/* See COPYING.txt for license details. */

/*
 * picopass.h — PicoPass / iCLASS protocol definitions
 *
 * Based on the PicoPass 2KS datasheet and Flipper Zero Picopass app (bettse).
 * Ported to M1 platform (ST25R3916 + RFAL).
 */

#ifndef PICOPASS_H_
#define PICOPASS_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ───────── PicoPass command bytes ───────── */
#define PICOPASS_CMD_ACTALL          0x0AU
#define PICOPASS_CMD_ACT             0x8EU
#define PICOPASS_CMD_IDENTIFY        0x0CU   /* also READ (distinguished by length) */
#define PICOPASS_CMD_READ            0x0CU
#define PICOPASS_CMD_READ4           0x06U
#define PICOPASS_CMD_SELECT          0x81U
#define PICOPASS_CMD_READCHECK_KD    0x88U
#define PICOPASS_CMD_READCHECK_KC    0x18U
#define PICOPASS_CMD_CHECK           0x05U
#define PICOPASS_CMD_UPDATE          0x87U
#define PICOPASS_CMD_DETECT          0x0FU
#define PICOPASS_CMD_HALT            0x00U
#define PICOPASS_CMD_PAGESEL         0x84U

/* ───────── Block layout ───────── */
#define PICOPASS_BLOCK_LEN           8U
#define PICOPASS_MAX_APP_LIMIT       32U
#define PICOPASS_CSN_BLOCK           0U
#define PICOPASS_CFG_BLOCK           1U
#define PICOPASS_EPURSE_BLOCK        2U
#define PICOPASS_KD_BLOCK            3U
#define PICOPASS_KC_BLOCK            4U
#define PICOPASS_AIA_BLOCK           5U
#define PICOPASS_PACS_CFG_BLOCK      6U

/* Config byte 7 fuse flags */
#define PICOPASS_FUSE_PERS           0x80U
#define PICOPASS_FUSE_CRYPT10        0x18U
#define PICOPASS_FUSE_CRYPT1         0x10U
#define PICOPASS_FUSE_CRYPT0         0x08U
#define PICOPASS_FUSE_RA             0x04U

/* ───────── CRC ───────── */
#define PICOPASS_CRC_INIT            0xE012U
#define PICOPASS_CRC_POLY            0x8408U

/* ───────── Data structures ───────── */

typedef struct {
    uint8_t data[PICOPASS_BLOCK_LEN];
} PicopassBlock;

typedef struct {
    PicopassBlock blocks[PICOPASS_MAX_APP_LIMIT];
    uint8_t       block_count;       /* app_limit from config */
    uint8_t       csn[8];            /* Block 0 — Card Serial Number */
    uint8_t       cfg[8];            /* Block 1 — Configuration */
    uint8_t       epurse[8];         /* Block 2 — ePurse / challenge */
    uint8_t       aia[8];            /* Block 5 — Application Issuer Area */
    bool          secured;           /* true if CRYPT1|CRYPT0 indicate auth required */
    bool          auth_success;      /* true if CHECK passed */
    bool          legacy;            /* true if AIA == FF...FF (not SE) */
    uint8_t       div_key[8];        /* Diversified key (if auth succeeded) */
} PicopassData;

/* Key type selection */
typedef enum {
    PICOPASS_KEY_KD = 0,
    PICOPASS_KEY_KC = 1,
} PicopassKeyType;

/* ───────── CRC functions ───────── */

static inline void picopass_crc(const uint8_t *data, uint16_t len, uint8_t crc_out[2])
{
    uint16_t crc = PICOPASS_CRC_INIT;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1U)
                crc = (crc >> 1) ^ PICOPASS_CRC_POLY;
            else
                crc >>= 1;
        }
    }
    crc_out[0] = (uint8_t)(crc & 0xFFU);
    crc_out[1] = (uint8_t)((crc >> 8) & 0xFFU);
}

static inline bool picopass_crc_check(const uint8_t *data, uint16_t len)
{
    /* data includes trailing 2-byte CRC */
    if (len < 3) return false;
    uint8_t calc[2];
    picopass_crc(data, len - 2, calc);
    return (calc[0] == data[len - 2]) && (calc[1] == data[len - 1]);
}

#endif /* PICOPASS_H_ */
