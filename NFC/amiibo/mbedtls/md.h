/*
 * mbedtls/md.h — minimal mbedTLS-compatible message-digest shim for the M1
 * amiibo module. Provides only the HMAC-SHA256 subset that nfc3d/drbg.c and
 * nfc3d/amiibo.c use, backed by the vendored Brad Conte SHA-256. Lets the
 * upstream nfc3d source compile byte-for-byte unmodified. Implemented in
 * mbedtls_shim.c.
 */
#ifndef M1_MBEDTLS_MD_SHIM_H
#define M1_MBEDTLS_MD_SHIM_H

#include <stddef.h>
#include <stdint.h>
#include "sha256.h"

typedef enum {
    MBEDTLS_MD_NONE = 0,
    MBEDTLS_MD_SHA256,
} mbedtls_md_type_t;

/* Opaque info token — only SHA-256 is supported, so the value is a marker. */
typedef int mbedtls_md_info_t;

/* HMAC-SHA256 streaming context. Held by value inside nfc3d_drbg_ctx, so the
 * full definition must be visible here. */
typedef struct {
    SHA256_CTX sha;        /* inner hash in progress */
    uint8_t    ipad[64];   /* key XOR 0x36 (saved so _reset needs no key)  */
    uint8_t    opad[64];   /* key XOR 0x5c                                  */
    int        ready;      /* setup() called */
} mbedtls_md_context_t;

void                    mbedtls_md_init(mbedtls_md_context_t *ctx);
const mbedtls_md_info_t *mbedtls_md_info_from_type(mbedtls_md_type_t md_type);
int  mbedtls_md_setup(mbedtls_md_context_t *ctx, const mbedtls_md_info_t *md_info, int hmac);
int  mbedtls_md_hmac_starts(mbedtls_md_context_t *ctx, const unsigned char *key, size_t keylen);
int  mbedtls_md_hmac_update(mbedtls_md_context_t *ctx, const unsigned char *input, size_t ilen);
int  mbedtls_md_hmac_finish(mbedtls_md_context_t *ctx, unsigned char *output);
int  mbedtls_md_hmac_reset(mbedtls_md_context_t *ctx);
void mbedtls_md_free(mbedtls_md_context_t *ctx);

/* One-shot HMAC-SHA256. */
int  mbedtls_md_hmac(const mbedtls_md_info_t *md_info,
                     const unsigned char *key, size_t keylen,
                     const unsigned char *input, size_t ilen,
                     unsigned char *output);

#endif /* M1_MBEDTLS_MD_SHIM_H */
