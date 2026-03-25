/* See COPYING.txt for license details. */

/*
 * picopass_poller.h — PicoPass / iCLASS card reader/writer
 *
 * Sends PicoPass commands over ISO15693/NFC-V transport via RFAL,
 * handles authentication, block reading, and block writing.
 */

#ifndef PICOPASS_POLLER_H_
#define PICOPASS_POLLER_H_

#include "picopass.h"
#include "rfal_nfc.h"

typedef enum {
    PICOPASS_READ_OK = 0,
    PICOPASS_READ_ERR_NO_CARD,
    PICOPASS_READ_ERR_SELECT,
    PICOPASS_READ_ERR_AUTH,
    PICOPASS_READ_ERR_READ,
    PICOPASS_READ_ERR_CRC,
    PICOPASS_READ_ERR_UNSECURED,
} PicopassReadResult;

/**
 * @brief Read a PicoPass / iCLASS card.
 *
 * Full sequence: ACTALL → IDENTIFY → SELECT → READ config →
 * authenticate (tries standard + elite keys) → READ all blocks.
 *
 * @param dev   The RFAL NFC device (from discovery, type NFCV)
 * @param out   Output data structure — filled on success
 * @return PICOPASS_READ_OK on success
 */
PicopassReadResult picopass_poller_read(const rfalNfcDevice *dev, PicopassData *out);

/**
 * @brief Write a single block to an already-selected and authenticated card.
 *
 * Must be called after picopass_poller_read() succeeds with auth.
 * Uses the UPDATE command with MAC signing.
 *
 * @param block_num  Block number to write (cannot write CSN block 0)
 * @param data       8 bytes of data to write
 * @param div_key    Diversified key (from successful auth)
 * @return PICOPASS_READ_OK on success
 */
PicopassReadResult picopass_poller_write_block(uint8_t block_num,
                                               const uint8_t data[8],
                                               const uint8_t div_key[8]);

#endif /* PICOPASS_POLLER_H_ */
