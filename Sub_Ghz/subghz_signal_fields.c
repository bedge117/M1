/* See COPYING.txt for license details. */

/**
 * @file   subghz_signal_fields.c
 * @brief  KeeLoq-family field extract / assemble — Phase 9a-1.
 *
 * See subghz_signal_fields.h for the bit layouts and contract.
 *
 * M1 Project — Hapax fork
 */

#include "subghz_signal_fields.h"
#include "subghz_keeloq.h"
#include "subghz_nice_flor_s.h"
#include "subghz_came_atomo.h"
#include "subghz_alutech_at_4n.h"

#include <ctype.h>
#include <stddef.h>
#include <string.h>

/*============================================================================*/
/* Bit-layout constants                                                        */
/*============================================================================*/

#define SUBGHZ_SF_SERIAL_MASK_28  0x0FFFFFFFU
#define SUBGHZ_SF_BUTTON_MASK_4   0x0FU

/*============================================================================*/
/* Protocol-name matching                                                      */
/*============================================================================*/

/* Case-insensitive prefix match terminated by NUL or space.  Matches the
 * convention used by Sub_Ghz/subghz_keeloq_create.c::is_supported_keeloq()
 * and Sub_Ghz/subghz_keeloq_encoder.c::keeloq_is_keeloq_protocol(). */
static bool name_matches(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b))
    {
        ++a; ++b;
    }
    return (*b == '\0') && (*a == '\0' || *a == ' ');
}

static bool is_star_line(const char *protocol)
{
    return name_matches(protocol, "Star Line");
}

bool subghz_signal_fields_is_keeloq_family(const char *protocol)
{
    if (!protocol) return false;
    static const char *const names[] = { "KeeLoq", "Star Line", "Jarolift" };
    for (size_t i = 0; i < sizeof(names)/sizeof(names[0]); ++i)
    {
        if (name_matches(protocol, names[i]))
            return true;
    }
    return false;
}

/*============================================================================*/
/* Extract                                                                     */
/*============================================================================*/

bool subghz_signal_fields_keeloq_extract(const char            *protocol,
                                          uint64_t               key,
                                          subghz_keeloq_fields_t *out)
{
    if (!out)
        return false;

    /* Pre-clear so callers see a clean state on failure. */
    out->serial  = 0U;
    out->button  = 0U;
    out->enc_hop = 0U;

    if (!subghz_signal_fields_is_keeloq_family(protocol))
        return false;

    if (is_star_line(protocol))
    {
        /* [63:32] encrypted HOP, [31:4] serial[27:0], [3:0] button[3:0] */
        out->enc_hop = (uint32_t)(key >> 32);
        out->serial  = (uint32_t)((key >> 4) & SUBGHZ_SF_SERIAL_MASK_28);
        out->button  = (uint8_t)(key & SUBGHZ_SF_BUTTON_MASK_4);
    }
    else
    {
        /* KeeLoq / Jarolift:
         *   [63:60] button, [59:32] serial[27:0], [31:0] encrypted HOP */
        out->enc_hop = (uint32_t)(key & 0xFFFFFFFFULL);
        out->serial  = (uint32_t)((key >> 32) & SUBGHZ_SF_SERIAL_MASK_28);
        out->button  = (uint8_t)((key >> 60) & SUBGHZ_SF_BUTTON_MASK_4);
    }
    return true;
}

/*============================================================================*/
/* Assemble                                                                    */
/*============================================================================*/

bool subghz_signal_fields_keeloq_assemble(const char                   *protocol,
                                           const subghz_keeloq_fields_t *fields,
                                           uint64_t                     *key_out)
{
    if (!key_out)
        return false;
    *key_out = 0ULL;

    if (!fields)
        return false;
    if (!subghz_signal_fields_is_keeloq_family(protocol))
        return false;

    const uint32_t s = fields->serial & SUBGHZ_SF_SERIAL_MASK_28;
    const uint8_t  b = fields->button & SUBGHZ_SF_BUTTON_MASK_4;
    const uint32_t h = fields->enc_hop;

    if (is_star_line(protocol))
    {
        *key_out = ((uint64_t)h << 32) |
                   ((uint64_t)s << 4)  |
                   (uint64_t)b;
    }
    else
    {
        *key_out = ((uint64_t)b << 60) |
                   ((uint64_t)s << 32) |
                   (uint64_t)h;
    }
    return true;
}

/*============================================================================*/
/* Counter decode / encode (Phase 9c-1)                                        */
/*============================================================================*/

uint16_t subghz_signal_fields_keeloq_counter_decode(uint32_t enc_hop,
                                                    uint64_t device_key)
{
    const uint32_t plain = keeloq_decrypt(enc_hop, device_key);
    return (uint16_t)(plain >> 16);
}

uint32_t subghz_signal_fields_keeloq_counter_encode(uint32_t enc_hop,
                                                    uint16_t new_counter,
                                                    uint64_t device_key)
{
    /* Decrypt to recover the lower 16 plaintext bits
     * (button / VLOW / discriminant / overflow counter), substitute the
     * 16-bit rolling counter, and re-encrypt with the same device key.
     * This mirrors keeloq_increment_hop()'s structure exactly. */
    const uint32_t plain     = keeloq_decrypt(enc_hop, device_key);
    const uint32_t new_plain = ((uint32_t)new_counter << 16) |
                               (plain & 0x0000FFFFU);
    return keeloq_encrypt(new_plain, device_key);
}

/*============================================================================*/
/* Nice FloR-S field extraction / assembly (P3 — Phase 9e-2)                   */
/*============================================================================*/

bool subghz_signal_fields_is_nice_flor_s(const char *protocol)
{
    return name_matches(protocol, "Nice FloR-S");
}

bool subghz_signal_fields_nice_flor_s_extract(uint64_t                     key,
                                               subghz_nice_flor_s_fields_t *out)
{
    if (!out)
        return false;

    out->button      = (uint8_t)((key >> 48) & 0x0FU);
    out->repeat      = (uint8_t)((key >> 44) & 0x0FU);
    out->enc_payload = key & 0x0FFFFFFFFFFFULL;   /* 44-bit mask */
    return true;
}

bool subghz_signal_fields_nice_flor_s_assemble(
    const subghz_nice_flor_s_fields_t *fields,
    uint64_t                          *key_out)
{
    if (!key_out)
        return false;
    *key_out = 0ULL;

    if (!fields)
        return false;

    *key_out = ((uint64_t)(fields->button & 0x0FU) << 48) |
               ((uint64_t)(fields->repeat & 0x0FU) << 44) |
               (fields->enc_payload & 0x0FFFFFFFFFFFULL);
    return true;
}

/*============================================================================*/
/* Nice FloR-S rolling counter — decode / encode (P3)                          */
/*============================================================================*/

uint16_t subghz_signal_fields_nice_flor_s_counter_decode(
    uint64_t       enc_payload,
    const uint8_t  table[32])
{
    const uint64_t plain = nice_flor_s_decrypt(enc_payload, table);
    return (uint16_t)(plain & 0xFFFFU);
}

uint64_t subghz_signal_fields_nice_flor_s_counter_encode(
    uint64_t       enc_payload,
    uint16_t       new_counter,
    const uint8_t  table[32])
{
    /* Decrypt → substitute counter → re-encrypt.
     * The 28-bit serial (bits [43:16]) is preserved verbatim. */
    const uint64_t plain     = nice_flor_s_decrypt(enc_payload, table);
    const uint64_t new_plain = (plain & ~0xFFFFULL) | (uint64_t)new_counter;
    return nice_flor_s_encrypt(new_plain, table);
}

/*============================================================================*/
/* CAME Atomo field extraction / assembly (P4)                                 */
/*============================================================================*/

bool subghz_signal_fields_is_came_atomo(const char *protocol)
{
    return name_matches(protocol, "CAME Atomo");
}

/**
 * Convert a 62-bit Flipper key to an 8-byte plaintext buffer by
 * reversing the over-the-air transformation and running the LFSR decrypt.
 *
 * Over-the-air: transmitted = (~plain64 >> 4), so plain64 = ~(key << 4).
 * Then decrypt the 8-byte buffer in place.
 */
static void came_atomo_key_to_plain(uint64_t key, uint8_t plain[8])
{
    /* Reverse the OTA encoding: plain64 = ~(key << 4) */
    uint64_t raw = ~(key << 4);

    /* Store as big-endian bytes (MSB-first). */
    plain[0] = (uint8_t)(raw >> 56);
    plain[1] = (uint8_t)(raw >> 48);
    plain[2] = (uint8_t)(raw >> 40);
    plain[3] = (uint8_t)(raw >> 32);
    plain[4] = (uint8_t)(raw >> 24);
    plain[5] = (uint8_t)(raw >> 16);
    plain[6] = (uint8_t)(raw >>  8);
    plain[7] = (uint8_t)(raw);

    came_atomo_decrypt(plain);
}

/**
 * Encrypt an 8-byte plaintext buffer and convert to a 62-bit Flipper key.
 */
static uint64_t came_atomo_plain_to_key(uint8_t plain[8])
{
    came_atomo_encrypt(plain);

    /* Reassemble the 64-bit value from big-endian bytes. */
    uint64_t raw = ((uint64_t)plain[0] << 56) |
                   ((uint64_t)plain[1] << 48) |
                   ((uint64_t)plain[2] << 40) |
                   ((uint64_t)plain[3] << 32) |
                   ((uint64_t)plain[4] << 24) |
                   ((uint64_t)plain[5] << 16) |
                   ((uint64_t)plain[6] <<  8) |
                   ((uint64_t)plain[7]);

    /* Apply OTA encoding: transmitted = (~raw >> 4) */
    return (~raw) >> 4;
}

bool subghz_signal_fields_came_atomo_extract(uint64_t                    key,
                                             subghz_came_atomo_fields_t *out)
{
    if (!out)
        return false;

    out->serial  = 0;
    out->counter = 0;
    out->button  = 0;
    out->cnt_2   = 0;

    uint8_t plain[8];
    came_atomo_key_to_plain(key, plain);

    /* Plaintext layout (big-endian bytes):
     *   plain[0]       = cnt_2 (7 bits, bit 7 always 0)
     *   plain[1:2]     = cnt (16-bit rolling counter, big-endian)
     *   plain[3:6]     = serial (32-bit, big-endian)
     *   plain[7] >> 4  = btn (4-bit button code)                  */
    out->cnt_2   = plain[0] & 0x7FU;
    out->counter = (uint16_t)((uint16_t)plain[1] << 8 | plain[2]);
    out->serial  = ((uint32_t)plain[3] << 24) |
                   ((uint32_t)plain[4] << 16) |
                   ((uint32_t)plain[5] <<  8) |
                   ((uint32_t)plain[6]);
    out->button  = (plain[7] >> 4) & 0x0FU;

    return true;
}

bool subghz_signal_fields_came_atomo_assemble(
    const subghz_came_atomo_fields_t *fields,
    uint64_t                         *key_out)
{
    if (!key_out)
        return false;
    *key_out = 0ULL;
    if (!fields)
        return false;

    uint8_t plain[8];
    plain[0] = fields->cnt_2 & 0x7FU;
    plain[1] = (uint8_t)(fields->counter >> 8);
    plain[2] = (uint8_t)(fields->counter & 0xFF);
    plain[3] = (uint8_t)(fields->serial >> 24);
    plain[4] = (uint8_t)(fields->serial >> 16);
    plain[5] = (uint8_t)(fields->serial >>  8);
    plain[6] = (uint8_t)(fields->serial);
    plain[7] = (uint8_t)((fields->button & 0x0FU) << 4);

    *key_out = came_atomo_plain_to_key(plain);
    return true;
}

/*============================================================================*/
/* CAME Atomo rolling counter — decode / encode (P4)                           */
/*============================================================================*/

uint16_t subghz_signal_fields_came_atomo_counter_decode(uint64_t key)
{
    subghz_came_atomo_fields_t f;
    (void)subghz_signal_fields_came_atomo_extract(key, &f);
    return f.counter;
}

uint64_t subghz_signal_fields_came_atomo_counter_encode(uint64_t key,
                                                        uint16_t new_counter)
{
    subghz_came_atomo_fields_t f;
    (void)subghz_signal_fields_came_atomo_extract(key, &f);
    f.counter = new_counter;

    uint64_t new_key = 0;
    (void)subghz_signal_fields_came_atomo_assemble(&f, &new_key);
    return new_key;
}

/*============================================================================*/
/* Alutech AT-4N field extraction / assembly (P4)                              */
/*============================================================================*/

bool subghz_signal_fields_is_alutech_at_4n(const char *protocol)
{
    return name_matches(protocol, "Alutech AT-4N");
}

bool subghz_signal_fields_alutech_at_4n_extract(
    uint64_t                        key,
    const uint8_t                   table[32],
    subghz_alutech_at_4n_fields_t  *out)
{
    if (!out)
        return false;

    out->serial    = 0;
    out->counter   = 0;
    out->button    = 0;
    out->frame_crc = 0;
    out->enc_data  = 0;

    if (!table)
        return false;

    /* The M1 key field stores the 64-bit encrypted data block directly.
     * (The 8-bit frame CRC from the 72-bit OTA stream is not preserved
     * in the uint64_t key — the file parser reads only 8 bytes.)       */
    out->enc_data = key;

    /* Decrypt the 64-bit data block. */
    uint64_t plain = alutech_at_4n_decrypt(out->enc_data, table);
    uint8_t *p = (uint8_t *)&plain;

    /* Plaintext layout (after decrypt, the byte order matches the
     * platform byte order since alutech_at_4n_decrypt writes back
     * via pointer-to-data):
     *   p[0] = CRC check byte
     *   p[1:4] = serial (MSB-first in the logical layout, but
     *            alutech_at_4n_decrypt writes data1 → p[0..3] and
     *            data2 → p[4..7], so:)
     *   p[0]   = data1 >> 24  (CRC check byte)
     *   p[1]   = data1 >> 16  (serial byte 3)
     *   p[2]   = data1 >> 8   (serial byte 2)
     *   p[3]   = data1        (serial byte 1)
     *   p[4]   = data2 >> 24  (serial byte 0)
     *   p[5]   = data2 >> 16  (counter high)
     *   p[6]   = data2 >> 8   (counter low)
     *   p[7]   = data2        (button)                              */
    out->serial  = ((uint32_t)p[1] << 24) | ((uint32_t)p[2] << 16) |
                   ((uint32_t)p[3] <<  8) | ((uint32_t)p[4]);
    out->counter = (uint16_t)(((uint16_t)p[5] << 8) | p[6]);
    out->button  = p[7];

    return true;
}

bool subghz_signal_fields_alutech_at_4n_assemble(
    const subghz_alutech_at_4n_fields_t *fields,
    const uint8_t                        table[32],
    uint64_t                            *key_out)
{
    if (!key_out)
        return false;
    *key_out = 0ULL;
    if (!fields || !table)
        return false;

    /* Build the plaintext 64-bit block. */
    uint64_t plain = 0;
    uint8_t *p = (uint8_t *)&plain;

    /* CRC check byte over the low byte of the counter. */
    p[0] = alutech_at_4n_decrypt_data_crc((uint8_t)(fields->counter & 0xFFU));
    p[1] = (uint8_t)(fields->serial >> 24);
    p[2] = (uint8_t)(fields->serial >> 16);
    p[3] = (uint8_t)(fields->serial >>  8);
    p[4] = (uint8_t)(fields->serial);
    p[5] = (uint8_t)(fields->counter >> 8);
    p[6] = (uint8_t)(fields->counter & 0xFFU);
    p[7] = fields->button;

    /* Encrypt. */
    uint64_t enc = alutech_at_4n_encrypt(plain, table);

    /* The M1 key stores just the 64-bit encrypted data block;
     * the frame CRC is not included (uint64_t can't hold 72 bits). */
    *key_out = enc;
    return true;
}

/*============================================================================*/
/* Alutech AT-4N rolling counter — decode / encode (P4)                        */
/*============================================================================*/

uint16_t subghz_signal_fields_alutech_at_4n_counter_decode(
    uint64_t       enc_data,
    const uint8_t  table[32])
{
    uint64_t plain = alutech_at_4n_decrypt(enc_data, table);
    uint8_t *p = (uint8_t *)&plain;
    return (uint16_t)(((uint16_t)p[5] << 8) | p[6]);
}

uint64_t subghz_signal_fields_alutech_at_4n_counter_encode(
    uint64_t       enc_data,
    uint16_t       new_counter,
    const uint8_t  table[32])
{
    /* Decrypt → substitute counter + recompute CRC check → re-encrypt. */
    uint64_t plain = alutech_at_4n_decrypt(enc_data, table);
    uint8_t *p = (uint8_t *)&plain;

    /* Update counter bytes. */
    p[5] = (uint8_t)(new_counter >> 8);
    p[6] = (uint8_t)(new_counter & 0xFFU);

    /* Recompute the CRC check byte (byte 0) for the new counter. */
    p[0] = alutech_at_4n_decrypt_data_crc((uint8_t)(new_counter & 0xFFU));

    return alutech_at_4n_encrypt(plain, table);
}

/*============================================================================*/
/* Counter-edit capability probe (Phase 9e-1)                                  */
/*============================================================================*/

/* Static deferred-reason strings — pointed to by callers; never freed. */
static const char SF_REASON_EMPTY[]      = "";
static const char SF_REASON_PHOENIX_V2[] = "Phoenix V2: checksum recompute req.";

/* Phase 9e protocols — counter editing is on the roadmap but the
 * required decode/encode path is not yet implemented.  Each entry cites
 * the specific blocker documented in the Phase 9e checklist:
 *
 *   - Phoenix V2     : Counter bit-field is at a known offset in the
 *                      52-bit code but the trailing discriminant /
 *                      checksum must be recomputed after editing.
 *
 * Nice FloR-S has been promoted to SUPPORTED (P3) — its cipher is now
 * implemented in subghz_nice_flor_s.c with a loadable 32-byte rainbow
 * table, mirroring the KeeLoq key-vault pattern.
 *
 * CAME Atomo has been promoted to SUPPORTED (P4) — its LFSR stream
 * cipher is self-contained (no external key material).
 *
 * Alutech AT-4N has been promoted to SUPPORTED (P4) — its TEA-variant
 * cipher uses a 32-byte rainbow table injected at build time via the
 * ALUTECH_AT_4N_RAINBOW_TABLE secret.
 */
typedef struct {
    const char *name;
    const char *reason;
} sf_deferred_entry_t;

static const sf_deferred_entry_t SF_DEFERRED[] = {
    { "Phoenix_V2",    SF_REASON_PHOENIX_V2 },
};

subghz_counter_edit_status_t
subghz_signal_fields_counter_edit_status(const char  *protocol,
                                         const char **out_reason)
{
    if (subghz_signal_fields_is_keeloq_family(protocol) ||
        subghz_signal_fields_is_nice_flor_s(protocol) ||
        subghz_signal_fields_is_came_atomo(protocol) ||
        subghz_signal_fields_is_alutech_at_4n(protocol))
    {
        if (out_reason)
            *out_reason = SF_REASON_EMPTY;
        return SUBGHZ_COUNTER_EDIT_SUPPORTED;
    }

    if (protocol)
    {
        for (size_t i = 0; i < sizeof(SF_DEFERRED) / sizeof(SF_DEFERRED[0]); ++i)
        {
            if (name_matches(protocol, SF_DEFERRED[i].name))
            {
                if (out_reason)
                    *out_reason = SF_DEFERRED[i].reason;
                return SUBGHZ_COUNTER_EDIT_DEFERRED;
            }
        }
    }

    if (out_reason)
        *out_reason = SF_REASON_EMPTY;
    return SUBGHZ_COUNTER_EDIT_UNSUPPORTED;
}
