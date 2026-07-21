/*
 * m1_amiibo.h — M1 glue for the vendored nfc3d amiibo crypto.
 *
 * Loads the amiibo master keys (key_retail.bin) from SD and re-signs 540-byte
 * NTAG215 amiibo dumps in place so they validate on a real console (Switch),
 * regardless of whether the source dump's HMACs matched its UID.
 */
#ifndef M1_AMIIBO_H
#define M1_AMIIBO_H

#include <stdint.h>
#include <stdbool.h>

/* Load the 160-byte amiibo master keys from an explicit SD path (FatFs).
 * No-op if already loaded. Returns true on success. */
bool m1_amiibo_load_keys(const char *path);

/* Try the standard SD key locations; returns true if keys are available. */
bool m1_amiibo_ensure_keys(void);

/* True if the master keys are loaded. */
bool m1_amiibo_is_loaded(void);

/* Re-sign a 540-byte NTAG215 amiibo dump in place: decrypt then re-encrypt,
 * regenerating both the tag and data HMACs for the UID contained in the dump.
 * Returns true if keys were loaded and applied, false otherwise (dump left
 * untouched). Only the first 520 bytes (the amiibo figure region) are modified;
 * the trailing config/PWD/PACK pages are preserved. */
bool m1_amiibo_resign(uint8_t *dump540);

/* Validate the crypto primitives (HMAC-SHA256 + AES-128) against known test
 * vectors. Returns true if both pass. Call once before trusting re-signing. */
bool m1_amiibo_selftest(void);

#endif /* M1_AMIIBO_H */
