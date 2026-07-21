/*
 * mbedtls/aes.h — minimal mbedTLS-compatible AES shim for the M1 amiibo module.
 *
 * Provides only the subset of the mbedTLS AES API that nfc3d/amiibo.c uses
 * (AES-128-CTR), backed by the vendored tiny-AES-c block cipher. This lets the
 * upstream nfc3d source compile byte-for-byte unmodified. Implemented in
 * mbedtls_shim.c.
 */
#ifndef M1_MBEDTLS_AES_SHIM_H
#define M1_MBEDTLS_AES_SHIM_H

#include <stddef.h>
#include <stdint.h>
#include "tiny_aes.h"

typedef struct {
    struct AES_ctx ctx;
} mbedtls_aes_context;

/* AES-128 encryption key schedule. keybits must be 128 (all amiibo uses). */
int mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits);

/* AES-CTR stream cipher, matching mbedTLS semantics exactly:
 *  - nonce_counter is the 16-byte big-endian counter, incremented per block
 *  - stream_block holds the current keystream block
 *  - nc_off is the byte offset within stream_block (0..15)
 */
int mbedtls_aes_crypt_ctr(mbedtls_aes_context *ctx,
                          size_t length,
                          size_t *nc_off,
                          unsigned char nonce_counter[16],
                          unsigned char stream_block[16],
                          const unsigned char *input,
                          unsigned char *output);

#endif /* M1_MBEDTLS_AES_SHIM_H */
