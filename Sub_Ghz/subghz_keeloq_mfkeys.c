/* See COPYING.txt for license details. */

/**
 * @file  subghz_keeloq_mfkeys.c
 * @brief KeeLoq manufacturer (master) key store — FatFS backed implementation.
 *
 * Preferred path: loads from the AES-256-CBC encrypted binary file
 *   0:/SUBGHZ/keeloq_mfcodes.enc
 *
 * Plaintext fallback (auto-migrated): loads from
 *   0:/SUBGHZ/keeloq_mfcodes
 * then immediately saves the encrypted version and deletes the plaintext.
 *
 * Encrypted file format:
 *   Bytes 0–3:  Magic { 0x4D, 0x31, 0x4B, 0x4C }  "M1KL"
 *   Byte  4:    Version 0x01
 *   Bytes 5–8:  payload_len (uint32 little-endian)
 *   Bytes 9+:   AES-256-CBC ciphertext = [IV (16 bytes)] + [padded plaintext]
 *               Decrypted payload is text parsed by keeloq_mfkeys_load_text();
 *               both compact "HEX:TYPE:NAME\n" records and RocketGod
 *               multi-line records are accepted.
 *
 * Supported text formats (inside ciphertext or in plaintext fallback):
 *
 * 1. Compact (Flipper-compatible plain-text):
 *      AABBCCDDEEFFAABB:TYPE:Name
 *    where TYPE is 1 (Simple), 2 (Normal), or 3 (Secure) learning.
 *
 * 2. RocketGod SubGHz Toolkit multi-line format:
 *      Manufacturer: Name
 *      Key (Hex):    AABBCCDDEEFFAABB
 *      Key (Dec):    <decimal — ignored>
 *      Type:         TYPE
 *      ------------------------------------
 *    Use the RocketGod multi-line format directly or convert to compact
 *    format first.
 */

#include "subghz_keeloq_mfkeys.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "FreeRTOS.h"
#include "ff.h"
#include "m1_crypto.h"

/*============================================================================*/
/* Internal table                                                              */
/*============================================================================*/

#define KEELOQ_MFKEYS_MAX   512     /**< Maximum entries in the in-memory table */
#define KEELOQ_LINE_MAX     128     /**< Maximum line length in the keystore file */

/* Encrypted-file constants */
#define KEELOQ_ENC_HDR_LEN      9   /**< magic(4) + version(1) + payload_len(4) */
#define KEELOQ_ENC_VERSION      0x01
#define KEELOQ_ENC_MAGIC_0      0x4D  /* 'M' */
#define KEELOQ_ENC_MAGIC_1      0x31  /* '1' */
#define KEELOQ_ENC_MAGIC_2      0x4B  /* 'K' */
#define KEELOQ_ENC_MAGIC_3      0x4C  /* 'L' */

/* Maximum serialised plaintext size (one compact line per entry) */
#define KEELOQ_LINE_SERIAL_MAX  70  /**< 16+1+1+1+47+1+null = 68, rounded up */
#define KEELOQ_SERIAL_BUF_MAX   (KEELOQ_MFKEYS_MAX * KEELOQ_LINE_SERIAL_MAX)

/**
 * Fixed 32-byte AES-256 obfuscation key for the keeloq_mfcodes.enc file.
 *
 * This key is the same on every M1 unit — it provides file-at-rest
 * obfuscation (the plaintext is never on the SD card), NOT device binding.
 *
 * It is intentionally NOT a secret: the threat model is casual inspection
 * of the SD card, not a targeted key-extraction attack.
 */
static const uint8_t s_keeloq_product_key[M1_CRYPTO_AES_KEY_SIZE] = {
    0xA3, 0x7F, 0x2C, 0x51, 0xD8, 0x4E, 0x96, 0xBB,
    0x13, 0x05, 0xE7, 0x2A, 0x6C, 0xF4, 0x39, 0x80,
    0xD1, 0x5B, 0xC2, 0x47, 0x8E, 0x3A, 0x99, 0x0F,
    0x74, 0x1D, 0x62, 0xF5, 0xAB, 0xC8, 0x3E, 0x56,
};

static KeeLoqMfrEntry *s_table   = NULL;
static uint32_t        s_count   = 0;

/*============================================================================*/
/* Helpers                                                                     */
/*============================================================================*/

/** Case-insensitive character comparison. */
static int ci_strcmp(const char *a, const char *b)
{
    while (*a && *b) {
        int ca = tolower((unsigned char)*a);
        int cb = tolower((unsigned char)*b);
        if (ca != cb) return ca - cb;
        a++; b++;
    }
    return tolower((unsigned char)*a) - tolower((unsigned char)*b);
}

/**
 * Parse a hex string into a uint64_t (big-endian, exactly 16 hex digits).
 * Returns false if the string does not contain exactly 16 hex digits
 * followed immediately by a ':' separator.
 */
static bool parse_hex64(const char *p, uint64_t *out)
{
    uint64_t v = 0;
    int digits = 0;
    while (*p && digits < 16) {
        char c = *p++;
        uint8_t nibble;
        if (c >= '0' && c <= '9')       nibble = (uint8_t)(c - '0');
        else if (c >= 'A' && c <= 'F')  nibble = (uint8_t)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f')  nibble = (uint8_t)(c - 'a' + 10);
        else return false;
        v = (v << 4) | nibble;
        digits++;
    }
    /* Must have exactly 16 hex digits followed by ':' */
    if (digits != 16 || *p != ':') return false;
    *out = v;
    return true;
}

/**
 * Parse a standalone hex string into a uint64_t (big-endian, exactly 16
 * hex digits, terminated by end-of-string or whitespace — no ':' required).
 *
 * Returns false if the value is not exactly 16 hex digits, if any non-hex
 * character appears before the 16th digit, or if a hex digit or other
 * non-whitespace character immediately follows the 16th digit (e.g., a 17+
 * digit key is rejected).
 */
static bool parse_hex64_standalone(const char *p, uint64_t *out)
{
    uint64_t v = 0;
    int digits = 0;
    for (; digits < 16; digits++, p++) {
        char c = *p;
        uint8_t nibble;
        if (c >= '0' && c <= '9')       nibble = (uint8_t)(c - '0');
        else if (c >= 'A' && c <= 'F')  nibble = (uint8_t)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f')  nibble = (uint8_t)(c - 'a' + 10);
        else break;  /* non-hex character — stop */
        v = (v << 4) | nibble;
    }
    if (digits != 16) return false;
    /* Reject keys immediately followed by another hex digit (17+ digit strings). */
    char next = *p;
    if ((next >= '0' && next <= '9') ||
        (next >= 'A' && next <= 'F') ||
        (next >= 'a' && next <= 'f'))
        return false;
    *out = v;
    return true;
}

/*============================================================================*/
/* Shared line parser                                                          */
/*============================================================================*/

/**
 * In-progress state for the RocketGod multi-line format.
 * Reset when a "Manufacturer:" line is seen or a "---" separator is consumed.
 */
typedef struct {
    char rg_name[48];
    char rg_hex[20];
    int  rg_type_val;
    bool rg_has_name;
    bool rg_has_hex;
    bool rg_has_type;
} RGState;

/** Try to append the accumulated RocketGod entry to the table. */
static void rg_try_emit(RGState *rg)
{
    if (!rg->rg_has_name || !rg->rg_has_hex || !rg->rg_has_type) return;
    if (s_count >= KEELOQ_MFKEYS_MAX) return;
    if (rg->rg_type_val < KEELOQ_LEARN_SIMPLE ||
        rg->rg_type_val > KEELOQ_LEARN_FAAC_SLH) return;

    uint64_t mfr_key = 0;
    if (!parse_hex64_standalone(rg->rg_hex, &mfr_key)) return;

    KeeLoqMfrEntry *e = &s_table[s_count];
    e->key        = mfr_key;
    e->learn_type = (KeeLoqLearnType)rg->rg_type_val;
    strncpy(e->name, rg->rg_name, sizeof(e->name) - 1);
    e->name[sizeof(e->name) - 1] = '\0';
    s_count++;
}

/**
 * Parse one stripped (no trailing whitespace/CRLF), null-terminated line
 * into the table.  Updates @p rg with any accumulated RocketGod state.
 *
 * @param line  Stripped line (modified in-place for compact format pointer math).
 * @param rg    Persistent RocketGod multi-line state across calls.
 */
static void parse_mfkeys_line(char *line, RGState *rg)
{
    if (line[0] == '\0' || line[0] == '#') return;

    /* Skip Flipper keystore header lines */
    if (strncmp(line, "Filetype:",   9) == 0 ||
        strncmp(line, "Version:",    8) == 0 ||
        strncmp(line, "Encryption:", 11) == 0)
        return;

    /* ── RocketGod multi-line format detection ─────────────────── */

    if (strncmp(line, "Manufacturer:", 13) == 0) {
        /* New RocketGod entry — emit any pending entry first, then reset */
        rg_try_emit(rg);
        memset(rg, 0, sizeof(*rg));
        const char *v = line + 13;
        while (*v == ' ') v++;
        strncpy(rg->rg_name, v, sizeof(rg->rg_name) - 1);
        rg->rg_name[sizeof(rg->rg_name) - 1] = '\0';
        rg->rg_has_name = (rg->rg_name[0] != '\0');
        return;
    }

    if (strncmp(line, "Key (Hex):", 10) == 0) {
        /* The strncmp guard above already verifies ':' is present.
         * Skip past the full "Key (Hex):" prefix (10 chars) then spaces. */
        const char *v = line + 10;
        while (*v == ' ') v++;
        strncpy(rg->rg_hex, v, sizeof(rg->rg_hex) - 1);
        rg->rg_hex[sizeof(rg->rg_hex) - 1] = '\0';
        rg->rg_has_hex = (rg->rg_hex[0] != '\0');
        return;
    }

    if (strncmp(line, "Key (Dec):", 10) == 0) {
        /* Decimal key — present in RocketGod output, ignored by M1 */
        return;
    }

    if (rg->rg_has_name && strncmp(line, "Type:", 5) == 0) {
        const char *v = line + 5;
        while (*v == ' ') v++;
        char *endptr;
        long parsed = strtol(v, &endptr, 10);
        /* Accept only a pure integer field (no trailing garbage) */
        if (endptr != v && (*endptr == '\0' || *endptr == ' ') &&
            parsed >= KEELOQ_LEARN_SIMPLE && parsed <= KEELOQ_LEARN_FAAC_SLH) {
            rg->rg_type_val = (int)parsed;
            rg->rg_has_type = true;
        }
        /* Emit immediately if all fields are now present */
        if (rg->rg_has_name && rg->rg_has_hex && rg->rg_has_type) {
            rg_try_emit(rg);
            memset(rg, 0, sizeof(*rg));
        }
        return;
    }

    if (line[0] == '-' && line[1] == '-') {
        /* RocketGod separator — emit pending entry if complete, then reset */
        rg_try_emit(rg);
        memset(rg, 0, sizeof(*rg));
        return;
    }

    /* ── Compact format: AABBCCDDEEFFAABB:TYPE:Name ─────────────── */

    if (s_count >= KEELOQ_MFKEYS_MAX) return;

    char *p = line;

    /* parse_hex64 consumes exactly 16 hex digits and verifies ':' follows */
    uint64_t mfr_key = 0;
    if (!parse_hex64(p, &mfr_key)) return;

    /* Advance p past the 16 hex digits and the ':' */
    p += 17;

    /* Parse learn type with full integer validation */
    char *endptr;
    long learn_type = strtol(p, &endptr, 10);
    if (endptr == p || *endptr != ':') return;
    if (learn_type < KEELOQ_LEARN_SIMPLE || learn_type > KEELOQ_LEARN_FAAC_SLH) return;
    p = endptr + 1;  /* skip ':' */

    /* Skip leading whitespace before name */
    while (*p == ' ') p++;

    /* Name must be non-empty */
    if (*p == '\0') return;

    /* Populate entry */
    KeeLoqMfrEntry *e = &s_table[s_count];
    e->key        = mfr_key;
    e->learn_type = (KeeLoqLearnType)learn_type;
    strncpy(e->name, p, sizeof(e->name) - 1);
    e->name[sizeof(e->name) - 1] = '\0';
    s_count++;
}

/** Allocate the entry table and reset the count. */
static bool prepare_table(void)
{
    keeloq_mfkeys_free();
    s_table = (KeeLoqMfrEntry *)pvPortMalloc(KEELOQ_MFKEYS_MAX * sizeof(KeeLoqMfrEntry));
    if (!s_table) return false;
    s_count = 0;
    return true;
}

/*============================================================================*/
/* keeloq_mfkeys_load_text()                                                  */
/*============================================================================*/

bool keeloq_mfkeys_load_text(const char *text)
{
    if (!text) return false;
    if (!prepare_table()) return false;

    RGState rg;
    memset(&rg, 0, sizeof(rg));

    char line[KEELOQ_LINE_MAX];
    const char *pos = text;

    while (*pos && s_count < KEELOQ_MFKEYS_MAX) {
        /* Extract one line from the text buffer */
        size_t i = 0;
        while (*pos && *pos != '\n' && i < sizeof(line) - 1)
            line[i++] = *pos++;
        if (*pos == '\n') pos++;
        line[i] = '\0';

        /* Strip trailing whitespace / CRLF */
        while (i > 0 && (line[i-1] == '\r' || line[i-1] == '\n' ||
                          line[i-1] == ' '))
            line[--i] = '\0';

        parse_mfkeys_line(line, &rg);
    }

    /* Emit any trailing RocketGod entry not terminated by a separator */
    rg_try_emit(&rg);

    return true;  /* success even if 0 entries parsed */
}

/*============================================================================*/
/* serialize_table() — internal                                               */
/*============================================================================*/

/**
 * Serialise the in-memory table to compact "HEX:TYPE:NAME\n" text.
 * Writes into @p buf (of @p buf_size bytes).
 * Returns the number of bytes written (not including a null terminator).
 */
static uint32_t serialize_table(char *buf, uint32_t buf_size)
{
    uint32_t total = 0;

    for (uint32_t i = 0; i < s_count; i++) {
        int n = snprintf(buf + total, buf_size - total,
                         "%016llX:%u:%s\n",
                         (unsigned long long)s_table[i].key,
                         (unsigned int)s_table[i].learn_type,
                         s_table[i].name);
        if (n <= 0 || (uint32_t)n >= buf_size - total)
            break;  /* Buffer full — stop (table is still partially saved) */
        total += (uint32_t)n;
    }
    return total;
}

/*============================================================================*/
/* keeloq_mfkeys_save_encrypted()                                             */
/*============================================================================*/

bool keeloq_mfkeys_save_encrypted(void)
{
    if (!s_table || s_count == 0)
        return false;

    /* Compute buffer sizes based on s_count (actual entries, not worst-case
     * maximum).  encrypt_with_key works in-place: the buffer must hold the IV
     * prefix and PKCS7-padded ciphertext, so allocate plaintext + one extra
     * AES block for padding + IV. */
    uint32_t plain_max = (uint32_t)s_count * KEELOQ_LINE_SERIAL_MAX;
    uint32_t enc_max   = M1_CRYPTO_IV_SIZE +
                         ((plain_max / M1_CRYPTO_AES_BLOCK_SIZE) + 1)
                         * M1_CRYPTO_AES_BLOCK_SIZE;

    uint8_t *buf = (uint8_t *)pvPortMalloc(enc_max);
    if (!buf)
        return false;

    /* Declare all locals before the first goto to avoid C99 jump-over issues. */
    bool     ok       = false;
    bool     file_open = false;
    uint32_t text_len = 0;
    uint32_t enc_len  = 0;
    uint8_t  header[KEELOQ_ENC_HDR_LEN];
    FIL      f;
    UINT     bw;
    FRESULT  wfr;

    /* Serialise the table into the start of the buffer */
    text_len = serialize_table((char *)buf, enc_max);
    if (text_len == 0)
        goto cleanup;

    /* Encrypt in-place (buf expands: [text] → [IV][ciphertext]) */
    enc_len = m1_crypto_encrypt_with_key(buf, text_len, enc_max,
                                         s_keeloq_product_key);
    if (enc_len == 0)
        goto cleanup;

    /* Open output file */
    if (f_open(&f, KEELOQ_MFKEYS_ENC_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        goto cleanup;
    file_open = true;

    /* Write header: magic(4) + version(1) + payload_len(4 LE) */
    header[0] = KEELOQ_ENC_MAGIC_0;
    header[1] = KEELOQ_ENC_MAGIC_1;
    header[2] = KEELOQ_ENC_MAGIC_2;
    header[3] = KEELOQ_ENC_MAGIC_3;
    header[4] = KEELOQ_ENC_VERSION;
    header[5] = (uint8_t)(enc_len & 0xFF);
    header[6] = (uint8_t)((enc_len >>  8) & 0xFF);
    header[7] = (uint8_t)((enc_len >> 16) & 0xFF);
    header[8] = (uint8_t)((enc_len >> 24) & 0xFF);

    wfr = f_write(&f, header, sizeof(header), &bw);
    if (wfr != FR_OK || bw != sizeof(header))
        goto cleanup;

    wfr = f_write(&f, buf, enc_len, &bw);
    ok = (wfr == FR_OK && bw == enc_len);

cleanup:
    if (file_open)
        f_close(&f);
    /* Always wipe the heap buffer — it may contain plaintext or encrypted
     * key material regardless of which path we took. */
    memset(buf, 0, enc_max);
    vPortFree(buf);
    return ok;
}

/*============================================================================*/
/* keeloq_mfkeys_load_encrypted()                                             */
/*============================================================================*/

bool keeloq_mfkeys_load_encrypted(void)
{
    FIL f;
    FRESULT fr = f_open(&f, KEELOQ_MFKEYS_ENC_PATH, FA_READ);
    if (fr != FR_OK)
        return false;

    /* Read and validate the 9-byte header */
    uint8_t header[KEELOQ_ENC_HDR_LEN];
    UINT br;
    fr = f_read(&f, header, sizeof(header), &br);
    if (fr != FR_OK || br != sizeof(header)) {
        f_close(&f);
        return false;
    }

    /* Verify magic "M1KL" */
    if (header[0] != KEELOQ_ENC_MAGIC_0 ||
        header[1] != KEELOQ_ENC_MAGIC_1 ||
        header[2] != KEELOQ_ENC_MAGIC_2 ||
        header[3] != KEELOQ_ENC_MAGIC_3) {
        f_close(&f);
        return false;
    }

    /* Verify version */
    if (header[4] != KEELOQ_ENC_VERSION) {
        f_close(&f);
        return false;
    }

    /* Read payload_len (little-endian uint32) */
    uint32_t payload_len = (uint32_t)header[5]         |
                           ((uint32_t)header[6] <<  8) |
                           ((uint32_t)header[7] << 16) |
                           ((uint32_t)header[8] << 24);

    /* Sanity check: must be at least IV + one AES block, and not exceed
     * the maximum possible serialised + encrypted table size. */
    uint32_t max_payload = M1_CRYPTO_IV_SIZE +
                           ((KEELOQ_SERIAL_BUF_MAX / M1_CRYPTO_AES_BLOCK_SIZE) + 1)
                           * M1_CRYPTO_AES_BLOCK_SIZE;
    if (payload_len < (uint32_t)(M1_CRYPTO_IV_SIZE + M1_CRYPTO_AES_BLOCK_SIZE) ||
        payload_len > max_payload) {
        f_close(&f);
        return false;
    }

    /* Allocate buffer: payload + 1 byte for null terminator after decryption */
    uint8_t *buf = (uint8_t *)pvPortMalloc(payload_len + 1);
    if (!buf) {
        f_close(&f);
        return false;
    }

    /* Read encrypted payload */
    fr = f_read(&f, buf, payload_len, &br);
    f_close(&f);
    if (fr != FR_OK || br != payload_len) {
        vPortFree(buf);
        return false;
    }

    /* Decrypt in-place; plaintext starts at buf[0] on success */
    uint32_t plain_len = m1_crypto_decrypt_with_key(buf, payload_len,
                                                     s_keeloq_product_key);
    if (plain_len == 0) {
        memset(buf, 0, payload_len + 1);
        vPortFree(buf);
        return false;
    }

    /* Null-terminate so load_text() sees a valid C string */
    buf[plain_len] = '\0';

    /* Parse the decrypted compact-text payload */
    bool ok = keeloq_mfkeys_load_text((const char *)buf);

    /* Clear sensitive data before freeing */
    memset(buf, 0, payload_len + 1);
    vPortFree(buf);

    return ok;
}

/*============================================================================*/
/* keeloq_mfkeys_load()                                                       */
/*============================================================================*/

bool keeloq_mfkeys_load(void)
{
    /* 1. Build-time embedded keys — highest priority (Flipper parity).
     *    If the firmware was compiled with a private key vault, the keys are
     *    baked into flash and the SD card is never consulted.  This means
     *    the manufacturer keys are not accessible as any file on the SD card. */
    if (keeloq_mfkeys_builtin_len > 0 && keeloq_mfkeys_builtin_text != NULL)
        return keeloq_mfkeys_load_text(keeloq_mfkeys_builtin_text);

    /* 2. Try the encrypted SD card keystore */
    if (keeloq_mfkeys_load_encrypted())
        return true;

    /* 3. Fall back to the legacy plaintext file */
    FIL f;
    FRESULT fr = f_open(&f, KEELOQ_MFKEYS_PATH, FA_READ);
    if (fr != FR_OK)
        return false;

    if (!prepare_table()) {
        f_close(&f);
        return false;
    }

    RGState rg;
    memset(&rg, 0, sizeof(rg));

    char line[KEELOQ_LINE_MAX];

    while (f_gets(line, sizeof(line), &f) && s_count < KEELOQ_MFKEYS_MAX) {
        /* Strip trailing whitespace / CRLF */
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' ||
                            line[len-1] == ' '))
            line[--len] = '\0';

        parse_mfkeys_line(line, &rg);
    }

    /* Emit any trailing RocketGod entry not terminated by a separator */
    rg_try_emit(&rg);

    f_close(&f);

    /* 3. Auto-migrate: save encrypted, delete plaintext (best-effort) */
    if (s_count > 0) {
        if (keeloq_mfkeys_save_encrypted())
            f_unlink(KEELOQ_MFKEYS_PATH);
    }

    return true;
}

void keeloq_mfkeys_free(void)
{
    if (s_table) {
        vPortFree(s_table);
        s_table = NULL;
    }
    s_count = 0;
}

bool keeloq_mfkeys_find(const char *name, KeeLoqMfrEntry *entry)
{
    if (!s_table || !name || !entry) return false;

    for (uint32_t i = 0; i < s_count; i++) {
        if (ci_strcmp(s_table[i].name, name) == 0) {
            *entry = s_table[i];
            return true;
        }
    }
    return false;
}

uint32_t keeloq_mfkeys_count(void)
{
    return s_count;
}

bool keeloq_mfkeys_get_at(uint32_t index, KeeLoqMfrEntry *entry)
{
    if (!s_table || !entry) return false;
    if (index >= s_count)   return false;
    *entry = s_table[index];
    return true;
}
