/* See COPYING.txt for license details. */

#ifndef NFC_POLLER_H_
#define NFC_POLLER_H_

#include <stdbool.h>
#include <stdint.h>
#include "rfal_nfc.h"

/**
 * @brief ReadIni - Initialize NFC poller (READ-ONLY mode)
 *
 * @retval true Initialization successful
 * @retval false Initialization failed
 */
bool ReadIni(void);

/**
 * @brief ReadCycle - Main NFC poller processing loop
 *
 * @retval None
 */
void ReadCycle(void);

/* --- Extern wrappers for static poller helpers --- */

/** Check if SAK indicates a MIFARE Classic variant */
bool nfc_poller_is_classic_sak(uint8_t sak);

/** Read all sectors of a MIFARE Classic card using key dictionary */
void nfc_poller_read_mfc(const rfalNfcDevice *dev);

/** Read Type 2 Tag (NTAG/Ultralight) pages */
void nfc_poller_read_t2t(const rfalNfcDevice *dev);

/** Send PWD_AUTH to NTAG/Ultralight; on success pack[2] is filled */
ReturnCode nfc_poller_pwd_auth(const uint8_t pwd[4], uint8_t pack[2]);

/** Send GET_VERSION to NTAG; fills rxBuf with 8-byte version */
ReturnCode nfc_poller_get_version(uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rcvLen);

#endif /* NFC_POLLER_H_ */