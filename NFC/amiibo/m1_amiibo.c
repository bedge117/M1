/*
 * m1_amiibo.c — see m1_amiibo.h. Bridges the vendored nfc3d amiibo crypto to
 * the M1's FatFs storage and NTAG215 emulation path.
 */
#include "m1_amiibo.h"
#include "nfc3d/amiibo.h"
#include "mbedtls/md.h"
#include "tiny_aes.h"
#include "sha256.h"
#include "ff.h"
#include <string.h>

/* NTAG215 amiibo dump: 135 pages x 4 = 540 bytes; nfc3d touches the first 520. */
#define AMIIBO_DUMP_SIZE 540

static nfc3d_amiibo_keys s_keys;
static bool              s_loaded = false;

bool m1_amiibo_is_loaded(void)
{
    return s_loaded;
}

bool m1_amiibo_load_keys(const char *path)
{
    FIL     f;
    UINT    br = 0;
    FRESULT r;

    if (s_loaded) {
        return true;
    }
    if (f_open(&f, path, FA_READ) != FR_OK) {
        return false;
    }
    r = f_read(&f, &s_keys, sizeof(s_keys), &br);
    f_close(&f);

    if (r != FR_OK || br != sizeof(s_keys)) {
        return false;
    }
    /* Sanity: same validation nfc3d_amiibo_load_keys applies. */
    if (s_keys.data.magicBytesSize > 16 || s_keys.tag.magicBytesSize > 16) {
        return false;
    }
    s_loaded = true;
    return true;
}

bool m1_amiibo_ensure_keys(void)
{
    static const char *const paths[] = {
        "0:/NFC/key_retail.bin",
        "0:/key_retail.bin",
        "NFC/key_retail.bin",
        "key_retail.bin",
    };

    if (s_loaded) {
        return true;
    }
    for (unsigned i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
        if (m1_amiibo_load_keys(paths[i])) {
            return true;
        }
    }
    return false;
}

bool m1_amiibo_resign(uint8_t *dump540)
{
    static int s_selftest = -1; /* -1 untested, 0 fail, 1 pass */
    uint8_t plain[NFC3D_AMIIBO_SIZE];

    if (!s_loaded || dump540 == NULL) {
        return false;
    }
    /* Verify the crypto primitives once before trusting any re-sign — a wrong
     * AES/HMAC would silently corrupt every dump. */
    if (s_selftest < 0) {
        s_selftest = m1_amiibo_selftest() ? 1 : 0;
    }
    if (s_selftest == 0) {
        return false;
    }
    /* Decrypt to plain (validity result intentionally ignored — a scrubbed or
     * mismatched dump still decrypts; the re-pack below fixes its HMACs). */
    (void)nfc3d_amiibo_unpack(&s_keys, dump540, plain);
    /* Re-encrypt with HMACs regenerated for the UID inside the dump. */
    nfc3d_amiibo_pack(&s_keys, plain, dump540);
    return true;
}

bool m1_amiibo_selftest(void)
{
    /* HMAC-SHA256 — RFC 4231 test case 2. */
    static const uint8_t hkey[]  = "Jefe";
    static const uint8_t hmsg[]  = "what do ya want for nothing?";
    static const uint8_t hexp[32] = {
        0x5b,0xdc,0xc1,0x46,0xbf,0x60,0x75,0x4e,0x6a,0x04,0x24,0x26,0x08,0x95,0x75,0xc7,
        0x5a,0x00,0x3f,0x08,0x9d,0x27,0x39,0x83,0x9d,0xec,0x58,0xb9,0x64,0xec,0x38,0x43,
    };
    uint8_t hout[32];
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                    hkey, 4, hmsg, 28, hout);
    if (memcmp(hout, hexp, 32) != 0) {
        return false;
    }

    /* AES-128 single block — FIPS-197 example. */
    static const uint8_t akey[16] = {
        0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
    uint8_t ablk[16] = {
        0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
    static const uint8_t aexp[16] = {
        0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a };
    struct AES_ctx actx;
    AES_init_ctx(&actx, akey);
    AES_ECB_encrypt(&actx, ablk);
    if (memcmp(ablk, aexp, 16) != 0) {
        return false;
    }

    return true;
}
