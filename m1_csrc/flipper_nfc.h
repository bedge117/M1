/* See COPYING.txt for license details. */

/*
 * flipper_nfc.h
 *
 * Flipper Zero .nfc file format parser
 *
 * M1 Project
 */

#ifndef FLIPPER_NFC_H_
#define FLIPPER_NFC_H_

#include "flipper_file.h"

#define FLIPPER_NFC_UID_MAX_LEN    10
#define FLIPPER_NFC_ATQA_LEN       2

typedef enum {
	FLIPPER_NFC_TYPE_ISO14443_3A = 0,
	FLIPPER_NFC_TYPE_ISO14443_3B,
	FLIPPER_NFC_TYPE_ISO14443_4A,
	FLIPPER_NFC_TYPE_NTAG,
	FLIPPER_NFC_TYPE_MIFARE_CLASSIC,
	FLIPPER_NFC_TYPE_MIFARE_DESFIRE,
	FLIPPER_NFC_TYPE_UNKNOWN
} flipper_nfc_type_t;

typedef struct {
	flipper_nfc_type_t type;
	uint8_t  uid[FLIPPER_NFC_UID_MAX_LEN];
	uint8_t  uid_len;
	uint8_t  atqa[FLIPPER_NFC_ATQA_LEN];
	uint8_t  sak;
	char     device_type[32];
} flipper_nfc_card_t;

/* Load a .nfc file (header only: UID, ATQA, SAK, device type) */
bool flipper_nfc_load(const char *path, flipper_nfc_card_t *out);

/* Save a .nfc file */
bool flipper_nfc_save(const char *path, const flipper_nfc_card_t *card);

/* Map Flipper NFC device type string to enum */
flipper_nfc_type_t flipper_nfc_parse_type(const char *type_str);

/* Load dump data from a Flipper .nfc file.
 * Parses "Page N:" and "Block N:" lines into the provided buffer.
 * Returns the number of units (pages or blocks) parsed.
 * unit_size is set to 4 for pages (T2T/NTAG) or 16 for blocks (Classic). */
uint16_t flipper_nfc_load_dump(const char *path,
                               uint8_t *dump_buf, uint16_t dump_buf_size,
                               uint8_t *valid_bits,
                               uint16_t *unit_size);

#endif /* FLIPPER_NFC_H_ */
