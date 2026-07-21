/*
 * mbedtls_shim.c — implements the tiny mbedTLS-compatible subset used by the
 * vendored nfc3d amiibo code, backed by tiny-AES-c (AES-128 block) and Brad
 * Conte's SHA-256. Keeping this shim lets the upstream nfc3d/*.c compile
 * byte-for-byte unmodified.
 *
 *  - AES: mbedtls_aes_setkey_enc + mbedtls_aes_crypt_ctr (CTR built here per
 *         mbedTLS semantics; tiny-AES supplies only the ECB block encrypt).
 *  - MD:  HMAC-SHA256 streaming + one-shot.
 */
#include <string.h>
#include "mbedtls/aes.h"
#include "mbedtls/md.h"
#include "tiny_aes.h"
#include "sha256.h"

/* ============================ AES-128-CTR ============================ */

int mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits)
{
    (void)keybits; /* amiibo is AES-128 only */
    AES_init_ctx(&ctx->ctx, key);
    return 0;
}

int mbedtls_aes_crypt_ctr(mbedtls_aes_context *ctx,
                          size_t length,
                          size_t *nc_off,
                          unsigned char nonce_counter[16],
                          unsigned char stream_block[16],
                          const unsigned char *input,
                          unsigned char *output)
{
    size_t n = *nc_off;

    while (length--) {
        if (n == 0) {
            memcpy(stream_block, nonce_counter, 16);
            AES_ECB_encrypt(&ctx->ctx, stream_block);
            /* increment the 128-bit counter, big-endian */
            for (int i = 16; i > 0; i--) {
                if (++nonce_counter[i - 1] != 0) {
                    break;
                }
            }
        }
        *output++ = (unsigned char)(*input++ ^ stream_block[n]);
        n = (n + 1) & 0x0F;
    }

    *nc_off = n;
    return 0;
}

/* ============================ HMAC-SHA256 ============================ */

#define HMAC_BLOCK 64

static const mbedtls_md_info_t s_sha256_info = MBEDTLS_MD_SHA256;

void mbedtls_md_init(mbedtls_md_context_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

const mbedtls_md_info_t *mbedtls_md_info_from_type(mbedtls_md_type_t md_type)
{
    return (md_type == MBEDTLS_MD_SHA256) ? &s_sha256_info : NULL;
}

int mbedtls_md_setup(mbedtls_md_context_t *ctx, const mbedtls_md_info_t *md_info, int hmac)
{
    (void)hmac;
    if (md_info == NULL) {
        return -1;
    }
    ctx->ready = 1;
    return 0;
}

int mbedtls_md_hmac_starts(mbedtls_md_context_t *ctx, const unsigned char *key, size_t keylen)
{
    uint8_t k[HMAC_BLOCK];

    memset(k, 0, sizeof(k));
    if (keylen > HMAC_BLOCK) {
        /* Key longer than block size is hashed first (not used by amiibo, but
         * kept faithful to HMAC). */
        SHA256_CTX kc;
        sha256_init(&kc);
        sha256_update(&kc, key, keylen);
        sha256_final(&kc, k); /* 32 bytes, rest stays zero-padded */
    } else {
        memcpy(k, key, keylen);
    }

    for (int i = 0; i < HMAC_BLOCK; i++) {
        ctx->ipad[i] = (uint8_t)(k[i] ^ 0x36);
        ctx->opad[i] = (uint8_t)(k[i] ^ 0x5c);
    }

    sha256_init(&ctx->sha);
    sha256_update(&ctx->sha, ctx->ipad, HMAC_BLOCK);
    return 0;
}

int mbedtls_md_hmac_update(mbedtls_md_context_t *ctx, const unsigned char *input, size_t ilen)
{
    sha256_update(&ctx->sha, input, ilen);
    return 0;
}

int mbedtls_md_hmac_finish(mbedtls_md_context_t *ctx, unsigned char *output)
{
    uint8_t inner[SHA256_BLOCK_SIZE];
    SHA256_CTX oc;

    sha256_final(&ctx->sha, inner);

    sha256_init(&oc);
    sha256_update(&oc, ctx->opad, HMAC_BLOCK);
    sha256_update(&oc, inner, SHA256_BLOCK_SIZE);
    sha256_final(&oc, output);
    return 0;
}

int mbedtls_md_hmac_reset(mbedtls_md_context_t *ctx)
{
    /* Restart the inner hash from the saved ipad — same key. */
    sha256_init(&ctx->sha);
    sha256_update(&ctx->sha, ctx->ipad, HMAC_BLOCK);
    return 0;
}

void mbedtls_md_free(mbedtls_md_context_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

int mbedtls_md_hmac(const mbedtls_md_info_t *md_info,
                    const unsigned char *key, size_t keylen,
                    const unsigned char *input, size_t ilen,
                    unsigned char *output)
{
    mbedtls_md_context_t ctx;
    if (md_info == NULL) {
        return -1;
    }
    mbedtls_md_init(&ctx);
    mbedtls_md_hmac_starts(&ctx, key, keylen);
    mbedtls_md_hmac_update(&ctx, input, ilen);
    mbedtls_md_hmac_finish(&ctx, output);
    mbedtls_md_free(&ctx);
    return 0;
}
