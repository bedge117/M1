/* See COPYING.txt for license details. */

/**
 * @file  subghz_keeloq_encoder.c
 * @brief KeeLoq counter-mode rolling-code encoder implementation.
 *
 * Supports KeeLoq, Star Line, and Jarolift (64-bit Flipper format).
 *
 * Key layout reference (Flipper SubGhz Key File format, 64 bits):
 *
 *   KeeLoq / Jarolift (Flipper Bit: 64):
 *     bits [63:32] = FIX = (button[3:0] << 28) | serial[27:0]
 *     bits [31: 0] = HOP = encrypted hop word
 *
 *   Star Line (Bit: 64):
 *     bits [63:32] = HOP = encrypted hop word
 *     bits [31: 4] = serial[27:0]
 *     bits [ 3: 0] = button[3:0]
 */

#include "subghz_keeloq_encoder.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "FreeRTOS.h"

/*============================================================================*/
/* Protocol detection                                                          */
/*============================================================================*/

bool keeloq_is_keeloq_protocol(const char *protocol)
{
    if (!protocol) return false;

    /* Case-insensitive prefix/full-string comparison helper */
    struct { const char *name; } kl_protos[] = {
        { "KeeLoq"    },
        { "Star Line" },
        { "Jarolift"  },
    };
    /* Simple case-insensitive compare */
    for (size_t i = 0; i < sizeof(kl_protos)/sizeof(kl_protos[0]); i++) {
        const char *a = protocol;
        const char *b = kl_protos[i].name;
        while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) {
            a++; b++;
        }
        if (*b == '\0' && (*a == '\0' || *a == ' '))
            return true;
    }
    return false;
}

/*============================================================================*/
/* Field extraction                                                            */
/*============================================================================*/

/**
 * Extract HOP and serial from a 64-bit key value based on the protocol.
 *
 * KeeLoq / Jarolift (Flipper format, Bit: 64):
 *   HOP    = key[31:0]
 *   serial = key[59:32] (28 bits)
 *   button = key[63:60] (4 bits)
 *
 * Star Line (Bit: 64):
 *   HOP    = key[63:32]
 *   serial = key[31:4]  (28 bits)
 *   button = key[3:0]   (4 bits)
 */
static void extract_fields(const char *protocol, uint64_t key,
                            uint32_t *hop, uint32_t *serial, uint8_t *button)
{
    /* Case-insensitive check for Star Line */
    bool is_starline = false;
    const char *a = protocol ? protocol : "";
    const char *b = "Star Line";
    while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) {
        a++; b++;
    }
    is_starline = (*b == '\0');

    if (is_starline) {
        *hop    = (uint32_t)(key >> 32);
        *serial = (uint32_t)((key >> 4) & 0x0FFFFFFFUL);
        *button = (uint8_t)(key & 0x0FU);
    } else {
        /* KeeLoq / Jarolift */
        *hop    = (uint32_t)(key & 0xFFFFFFFFUL);
        *serial = (uint32_t)((key >> 32) & 0x0FFFFFFFUL);
        *button = (uint8_t)((key >> 60) & 0x0FU);
    }
}

/**
 * Reconstruct a 64-bit key from fields for encoding.
 */
static uint64_t reconstruct_key(const char *protocol, uint32_t new_hop,
                                 uint32_t serial, uint8_t button)
{
    bool is_starline = false;
    const char *a = protocol ? protocol : "";
    const char *b = "Star Line";
    while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) {
        a++; b++;
    }
    is_starline = (*b == '\0');

    if (is_starline) {
        return ((uint64_t)new_hop << 32) |
               ((uint64_t)(serial & 0x0FFFFFFFUL) << 4) |
               (button & 0x0FU);
    } else {
        return ((uint64_t)((button & 0x0FU)) << 60) |
               ((uint64_t)(serial & 0x0FFFFFFFUL) << 32) |
               (uint64_t)new_hop;
    }
}

/*============================================================================*/
/* OOK PWM encoding                                                            */
/*============================================================================*/

uint32_t keeloq_pairs_per_rep(uint32_t preamble_pulses, uint32_t data_bits)
{
    /*
     * One repetition consists of:
     *   preamble_pulses pairs  (alternating short-high + short-low)
     *   data_bits pairs        (each bit = one high + one low)
     *   1 guard pair           (short-high + extra-long-low for inter-frame gap)
     */
    return preamble_pulses + data_bits + 1U;
}

/**
 * Encode a single KeeLoq repetition into @p out.
 *
 * Preamble: preamble_count pulses of te_short HIGH + te_short LOW (50% duty).
 * Data: each bit encoded as:
 *   0 → te_short HIGH + te_long LOW
 *   1 → te_long  HIGH + te_short LOW
 * Data bits are sent MSB first (matches Flipper encoder output).
 * Guard: te_short HIGH + 10×te_long LOW (inter-frame gap).
 *
 * @return number of pairs written.
 */
static uint32_t encode_one_rep(SubGhzRawPair *out,
                                uint64_t key,
                                uint32_t bits,
                                uint32_t te_short_us,
                                uint32_t te_long_us,
                                uint32_t preamble_count)
{
    uint32_t pos = 0;

    /* Preamble: alternating short pulses (each pair = one clock pulse) */
    for (uint32_t i = 0; i < preamble_count; i++) {
        out[pos].high_us = te_short_us;
        out[pos].low_us  = te_short_us;
        pos++;
    }

    /* Data bits, MSB first */
    for (int32_t bit_idx = (int32_t)bits - 1; bit_idx >= 0; bit_idx--) {
        uint32_t bit_val = (uint32_t)((key >> bit_idx) & 1U);
        if (bit_val == 0) {
            out[pos].high_us = te_short_us;
            out[pos].low_us  = te_long_us;
        } else {
            out[pos].high_us = te_long_us;
            out[pos].low_us  = te_short_us;
        }
        pos++;
    }

    /* Guard: short high + long low (inter-frame gap) */
    out[pos].high_us = te_short_us;
    out[pos].low_us  = te_long_us * 10U;
    pos++;

    return pos;
}

/*============================================================================*/
/* Main encoder API                                                            */
/*============================================================================*/

KeeLoqEncResult keeloq_encode_replay(
    const KeeLoqEncParams *params,
    SubGhzRawPair        **pairs_out,
    uint32_t              *npairs_out,
    uint8_t                reps)
{
    if (!params || !pairs_out || !npairs_out) return KEELOQ_ENC_BAD_PROTOCOL;

    *pairs_out  = NULL;
    *npairs_out = 0;

    /* Validate protocol */
    if (!keeloq_is_keeloq_protocol(params->protocol))
        return KEELOQ_ENC_BAD_PROTOCOL;

    /* We support 64-bit key format (Flipper-compatible) */
    if (params->bit_count != 64)
        return KEELOQ_ENC_BAD_BITS;

    /* Must have a manufacture name to look up the MK */
    if (!params->manufacture || params->manufacture[0] == '\0')
        return KEELOQ_ENC_NO_KEY;

    /* Find manufacturer key */
    KeeLoqMfrEntry mfr;
    if (!keeloq_mfkeys_find(params->manufacture, &mfr))
        return KEELOQ_ENC_NO_KEY;

    /* Extract packet fields */
    uint32_t hop, serial;
    uint8_t  button;
    extract_fields(params->protocol, params->key_value, &hop, &serial, &button);

    /* Derive device key */
    uint64_t device_key;
    switch (mfr.learn_type) {
    case KEELOQ_LEARN_NORMAL:
        device_key = keeloq_learn_normal(serial, mfr.key);
        break;
    case KEELOQ_LEARN_SIMPLE:
        device_key = keeloq_learn_simple(serial, mfr.key);
        break;
    case KEELOQ_LEARN_SECURE:
        /* Secure learning requires a seed, which is not available from the
         * encoded file data handled here. Refuse to generate an invalid hop
         * word instead of silently falling back to normal learning. */
        return KEELOQ_ENC_BAD_PROTOCOL;
    case KEELOQ_LEARN_MAGIC_XOR_TYPE1:
        device_key = keeloq_learn_magic_xor_type1(serial, mfr.key);
        break;
    case KEELOQ_LEARN_FAAC_SLH:
        /* FAAC SLH requires a seed from the captured data that is not
         * available in this encoder path. */
        return KEELOQ_ENC_BAD_PROTOCOL;
    default:
        return KEELOQ_ENC_BAD_PROTOCOL;
    }

    /* Counter-mode policy (Phase 9d):
     *   Increment mode (default): decrypt → +1 → re-encrypt
     *   Static    mode          : re-emit captured hop verbatim
     * The manufacturer-key derivation path always runs so that field
     * extraction and key reassembly use the same logic in both modes.
     */
    uint32_t new_hop;
    if (params->static_counter) {
        new_hop = hop;
    } else {
        new_hop = keeloq_increment_hop(hop, device_key);
    }

    /* Sync HOP-plaintext button bits [15:12] with the FIX-portion button
     * nibble before re-encrypting.  Without this, a packet whose FIX
     * button was mutated by subghz_button_override_apply() would
     * transmit FIX-button=new but HOP-plaintext-button=captured, which
     * a real receiver will reject (the redundant button transmission is
     * how KeeLoq-family receivers detect tampering).  For an untouched
     * capture this is a no-op because FIX-button == HOP-button by
     * construction on a real remote. */
    {
        uint32_t plain = keeloq_decrypt(new_hop, device_key);
        plain = (plain & 0xFFFF0FFFU) |
                ((uint32_t)(button & 0x0FU) << 12);
        new_hop = keeloq_encrypt(plain, device_key);
    }

    /* Reconstruct the 64-bit key with the new hop word */
    uint64_t new_key = reconstruct_key(params->protocol, new_hop, serial, button);

    /* TE timing */
    uint32_t te_short_us = (params->te > 0) ? params->te : KEELOQ_TE_SHORT;
    uint32_t te_long_us  = te_short_us * 2U;

    /* Allocate output buffer */
    uint32_t per_rep  = keeloq_pairs_per_rep(KEELOQ_PREAMBLE_PULSES, 64U);
    uint32_t total    = per_rep * (uint32_t)reps;
    SubGhzRawPair *buf = (SubGhzRawPair *)pvPortMalloc(total * sizeof(SubGhzRawPair));
    if (!buf) return KEELOQ_ENC_NOMEM;

    /* Encode @p reps repetitions */
    uint32_t written = 0;
    for (uint8_t r = 0; r < reps; r++) {
        written += encode_one_rep(buf + written, new_key, 64U,
                                  te_short_us, te_long_us,
                                  KEELOQ_PREAMBLE_PULSES);
    }

    *pairs_out  = buf;
    *npairs_out = written;
    return KEELOQ_ENC_OK;
}
