/* See COPYING.txt for license details. */

/*
 * picopass_keys.h — Known iCLASS master keys
 *
 * These are published research keys — see Garcia et al. "Exposing iClass
 * Key Diversification" (USENIX WOOT 2011).
 */

#ifndef PICOPASS_KEYS_H_
#define PICOPASS_KEYS_H_

#include <stdint.h>

/* HID iCLASS standard debit key (AA1 — Application Area 1) */
static const uint8_t picopass_iclass_key[8] = {
    0xAF, 0xA7, 0x85, 0xA7, 0xDA, 0xB3, 0x33, 0x78
};

/* Factory credit key */
static const uint8_t picopass_factory_credit_key[8] = {
    0x76, 0x65, 0x54, 0x43, 0x32, 0x21, 0x10, 0x00
};

/* Factory debit key */
static const uint8_t picopass_factory_debit_key[8] = {
    0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87
};

/* XiCE / XiCL key */
static const uint8_t picopass_xice_key[8] = {
    0x20, 0x20, 0x66, 0x66, 0x66, 0x66, 0x88, 0x88
};

/* XiCS key */
static const uint8_t picopass_xics_key[8] = {
    0x66, 0x66, 0x20, 0x20, 0x66, 0x66, 0x88, 0x88
};

/* Number of keys to try during dictionary attack */
#define PICOPASS_NUM_KEYS 5

static const uint8_t *picopass_key_list[PICOPASS_NUM_KEYS] = {
    picopass_iclass_key,
    picopass_factory_debit_key,
    picopass_factory_credit_key,
    picopass_xice_key,
    picopass_xics_key,
};

#endif /* PICOPASS_KEYS_H_ */
