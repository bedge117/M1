/* See COPYING.txt for license details. */

/**
 * @file   subghz_signal_fields.h
 * @brief  Per-protocol field extraction / assembly for the 64-bit Flipper
 *         SubGhz Key File representation — pure logic, host-testable
 *         (Phase 9a-1, foundation for the SignalSettings scene).
 *
 * The Saved → Settings flow needs to display and edit individual fields
 * (serial, button, counter) inside the opaque `uint64_t key` carried by
 * `flipper_subghz_signal_t`.  This module owns the bit/byte layouts for
 * each supported protocol family and provides reversible extract /
 * assemble helpers so the scene code never hard-codes shift amounts.
 *
 * Initial scope (Phase 9a-1): **KeeLoq family only** — KeeLoq, Star Line,
 * Jarolift.  These all use a 64-bit Flipper key whose layout depends on
 * the protocol name (KeeLoq/Jarolift vs Star Line — see
 * `subghz_keeloq_create.h` for the exact bit layouts).  The encrypted
 * 32-bit HOP word carries the rolling counter; decrypting it to expose
 * the counter for editing is a *separate* concern handled by the existing
 * `subghz_keeloq.c::keeloq_decrypt()` + per-file manufacturer-key
 * resolution and is wired up in Phase 9c.
 *
 * The module has zero hardware dependencies — no SI4463, no HAL, no
 * FreeRTOS, no FAT FS — and is fully testable on the host.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_SIGNAL_FIELDS_H
#define SUBGHZ_SIGNAL_FIELDS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* KeeLoq-family extracted fields                                              */
/*============================================================================*/

/**
 * Plaintext (pre-cipher) representation of the three fields that the
 * SignalSettings scene displays/edits for a KeeLoq-family `.sub` file.
 *
 * `serial` carries the full 28-bit device serial (the low 6 bits are the
 * KeeLoq "discriminant" that gets baked into the plaintext HOP word).
 * `button` is the 4-bit button code.  `enc_hop` is the 32-bit encrypted
 * HOP word as it appears in the Flipper key — counter editing requires
 * decrypting it with the manufacturer key (Phase 9c).
 */
typedef struct {
    uint32_t serial;    /**< 28-bit device serial                  */
    uint8_t  button;    /**<  4-bit button code                    */
    uint32_t enc_hop;   /**< 32-bit encrypted HOP word (counter inside) */
} subghz_keeloq_fields_t;

/**
 * Return true iff @p protocol is a KeeLoq-family protocol whose 64-bit
 * Flipper-format key layout this module knows how to extract/assemble.
 *
 * Recognised (case-insensitive, prefix-terminated by NUL or space):
 *   - "KeeLoq"
 *   - "Star Line"
 *   - "Jarolift"
 *
 * @param[in] protocol  Protocol name string, may be NULL.
 * @return  true if the protocol is a supported KeeLoq family member.
 */
bool subghz_signal_fields_is_keeloq_family(const char *protocol);

/**
 * Extract the {serial, button, encrypted-hop} triplet from a 64-bit
 * Flipper-format KeeLoq-family key.
 *
 * The bit layout is protocol-dependent and mirrors the encoder's
 * `reconstruct_key()` exactly (see `subghz_keeloq_create.h` for the
 * forward direction):
 *
 *   KeeLoq / Jarolift:
 *     [63:60] button[3:0]
 *     [59:32] serial[27:0]
 *     [31: 0] encrypted HOP
 *
 *   Star Line:
 *     [63:32] encrypted HOP
 *     [31: 4] serial[27:0]
 *     [ 3: 0] button[3:0]
 *
 * @param[in]  protocol  Protocol name — must be a KeeLoq family member.
 * @param[in]  key       64-bit Flipper key as carried by
 *                       `flipper_subghz_signal_t::key`.
 * @param[out] out       Receives the extracted fields.  Must be non-NULL.
 * @retval true   Extraction succeeded.
 * @retval false  @p protocol is NULL/unsupported or @p out is NULL.
 *                On failure @p *out is zero-initialised.
 */
bool subghz_signal_fields_keeloq_extract(const char            *protocol,
                                          uint64_t               key,
                                          subghz_keeloq_fields_t *out);

/**
 * Inverse of @ref subghz_signal_fields_keeloq_extract — assemble a
 * 64-bit Flipper-format KeeLoq-family key from {serial, button,
 * encrypted-hop}, using the protocol-specific layout described above.
 *
 * Field masking: @p serial is masked to 28 bits, @p button to 4 bits.
 * @p enc_hop is taken verbatim as a 32-bit value.
 *
 * @param[in]  protocol  Protocol name — must be a KeeLoq family member.
 * @param[in]  fields    Fields to encode.  Must be non-NULL.
 * @param[out] key_out   Receives the assembled 64-bit Flipper key.
 *                       Must be non-NULL.
 * @retval true   Assembly succeeded.
 * @retval false  @p protocol is NULL/unsupported, or @p fields / @p key_out
 *                is NULL.  On failure @p *key_out is 0.
 */
bool subghz_signal_fields_keeloq_assemble(const char                   *protocol,
                                           const subghz_keeloq_fields_t *fields,
                                           uint64_t                     *key_out);

/*============================================================================*/
/* KeeLoq-family rolling counter — decode / encode (Phase 9c-1)                */
/*============================================================================*/

/**
 * Decrypt the 32-bit encrypted HOP word @p enc_hop with @p device_key and
 * return its 16-bit rolling counter field (plaintext bits [31:16]).
 *
 * The KeeLoq plaintext HOP-word layout (Microchip HCS301, 32-bit):
 *   [31:16] 16-bit rolling counter
 *   [15:12]  4-bit button code (discriminant)
 *   [11:10]  2-bit VLOW / battery flag
 *   [ 9: 4]  6-bit discriminant (low 6 bits of serial)
 *   [ 3: 0]  4-bit overflow / function counter
 *
 * The lower 16 plaintext bits are not returned by this function — they
 * are recovered for re-encryption by
 * @ref subghz_signal_fields_keeloq_counter_encode, which decrypts
 * internally to preserve them.
 *
 * The caller is responsible for resolving @p device_key from the captured
 * file's `Manufacture:` line via the manufacturer-key store
 * (@ref keeloq_mfkeys_find) and the appropriate learning mode
 * (Simple / Normal / Secure — see @ref subghz_keeloq.h).
 *
 * @param  enc_hop     32-bit encrypted HOP word from the Flipper key.
 * @param  device_key  64-bit derived device key (output of
 *                     @ref keeloq_learn_normal / _simple / _secure).
 * @return  16-bit rolling counter.
 */
uint16_t subghz_signal_fields_keeloq_counter_decode(uint32_t enc_hop,
                                                    uint64_t device_key);

/**
 * Substitute @p new_counter into the 16-bit counter field of the
 * plaintext hop word obtained by decrypting @p enc_hop with
 * @p device_key, preserving the lower 16 plaintext bits (button,
 * VLOW, discriminant, overflow counter), and return the re-encrypted
 * 32-bit hop word.
 *
 * Mathematical relation to @ref keeloq_increment_hop:
 *   subghz_signal_fields_keeloq_counter_encode(
 *       h,
 *       subghz_signal_fields_keeloq_counter_decode(h, k) + 1,
 *       k)
 * is equivalent to keeloq_increment_hop(h, k) for any (h, k).
 *
 * @param  enc_hop      Current 32-bit encrypted HOP word (its low 16
 *                      plaintext bits are preserved verbatim).
 * @param  new_counter  Replacement 16-bit counter value.
 * @param  device_key   64-bit derived device key.
 * @return  New 32-bit encrypted HOP word with the substituted counter.
 */
uint32_t subghz_signal_fields_keeloq_counter_encode(uint32_t enc_hop,
                                                    uint16_t new_counter,
                                                    uint64_t device_key);

/*============================================================================*/
/* Nice FloR-S field extraction / assembly (P3 — Phase 9e-2)                   */
/*============================================================================*/

/**
 * Return true iff @p protocol is the Nice FloR-S protocol.
 *
 * Recognised (case-insensitive, prefix-terminated by NUL or space):
 *   - "Nice FloR-S"
 *
 * @param[in] protocol  Protocol name string, may be NULL.
 * @return  true if the protocol is Nice FloR-S.
 */
bool subghz_signal_fields_is_nice_flor_s(const char *protocol);

/**
 * Extracted fields from a 52-bit Nice FloR-S Flipper-format key.
 *
 * The 52-bit over-the-air layout:
 *   [51:48] button   — 4-bit positional button code (plaintext)
 *   [47:44] repeat   — 4-bit repetition counter     (plaintext)
 *   [43: 0] enc_part — 44-bit encrypted payload
 *
 * After decryption of enc_part:
 *   [43:16] 28-bit serial number
 *   [15: 0] 16-bit rolling counter
 */
typedef struct {
    uint8_t  button;      /**< 4-bit button positional code (plaintext) */
    uint8_t  repeat;      /**< 4-bit repetition counter     (plaintext) */
    uint64_t enc_payload; /**< 44-bit encrypted payload     (bits 43:0) */
} subghz_nice_flor_s_fields_t;

/**
 * Extract the {button, repeat, encrypted-payload} triplet from a 52-bit
 * Nice FloR-S key.  No rainbow table is needed — button and repeat are
 * plaintext, and the encrypted payload is returned as-is.
 *
 * @param[in]  key  52-bit Flipper key as carried by
 *                  `flipper_subghz_signal_t::key`.
 * @param[out] out  Receives the extracted fields.  Must be non-NULL.
 * @retval true   Always succeeds when @p out is non-NULL.
 * @retval false  @p out is NULL.
 */
bool subghz_signal_fields_nice_flor_s_extract(uint64_t                     key,
                                               subghz_nice_flor_s_fields_t *out);

/**
 * Inverse of @ref subghz_signal_fields_nice_flor_s_extract — assemble
 * a 52-bit Flipper-format Nice FloR-S key from {button, repeat,
 * encrypted-payload}.
 *
 * @param[in]  fields   Fields to encode.  Must be non-NULL.
 * @param[out] key_out  Receives the assembled 52-bit key.  Must be non-NULL.
 * @retval true   Assembly succeeded.
 * @retval false  @p fields or @p key_out is NULL.  On failure *key_out is 0.
 */
bool subghz_signal_fields_nice_flor_s_assemble(
    const subghz_nice_flor_s_fields_t *fields,
    uint64_t                          *key_out);

/*============================================================================*/
/* Nice FloR-S rolling counter — decode / encode (P3)                          */
/*============================================================================*/

/**
 * Decrypt the 44-bit encrypted payload with the 32-byte rainbow table
 * and return the 16-bit rolling counter.
 *
 * @param  enc_payload  44-bit encrypted payload (bits [43:0] of the key).
 * @param  table        32-byte rainbow table.  Must not be NULL.
 * @return 16-bit rolling counter.
 */
uint16_t subghz_signal_fields_nice_flor_s_counter_decode(
    uint64_t       enc_payload,
    const uint8_t  table[32]);

/**
 * Substitute @p new_counter into the plaintext obtained by decrypting
 * @p enc_payload with @p table, preserving the 28-bit serial, and
 * return the re-encrypted 44-bit payload.
 *
 * @param  enc_payload   Current 44-bit encrypted payload.
 * @param  new_counter   Replacement 16-bit counter value.
 * @param  table         32-byte rainbow table.
 * @return New 44-bit encrypted payload with the substituted counter.
 */
uint64_t subghz_signal_fields_nice_flor_s_counter_encode(
    uint64_t       enc_payload,
    uint16_t       new_counter,
    const uint8_t  table[32]);

/*============================================================================*/
/* CAME Atomo field extraction / assembly (P4)                                 */
/*============================================================================*/

/**
 * Return true iff @p protocol is the CAME Atomo protocol.
 *
 * @param[in] protocol  Protocol name string, may be NULL.
 * @return  true if the protocol is CAME Atomo.
 */
bool subghz_signal_fields_is_came_atomo(const char *protocol);

/**
 * Extracted fields from a 62-bit CAME Atomo Flipper-format key.
 *
 * The over-the-air data is transmitted as (~plaintext64 >> 4), yielding
 * 60 significant bits Manchester-encoded as 62 bits.  The Flipper key
 * stores the 62-bit received value; to recover the plaintext 64-bit
 * block we reverse: plaintext64 = ~(key << 4).
 *
 * Plaintext 64-bit block layout:
 *   [63:57] cnt_2   — 7-bit hold-cycle counter
 *   [56]    always 0
 *   [55:40] cnt     — 16-bit rolling counter
 *   [39: 8] serial  — 32-bit device serial
 *   [ 7: 4] btn     — 4-bit button code
 *   [ 3: 0] always 0
 */
typedef struct {
    uint32_t serial;    /**< 32-bit device serial */
    uint16_t counter;   /**< 16-bit rolling counter */
    uint8_t  button;    /**<  4-bit button code */
    uint8_t  cnt_2;     /**<  7-bit hold-cycle counter */
} subghz_came_atomo_fields_t;

/**
 * Extract the {serial, counter, button, cnt_2} tuple from a CAME Atomo
 * Flipper key by decrypting the LFSR cipher.  No external table is
 * needed — the cipher is self-contained.
 *
 * @param[in]  key  62-bit Flipper key.
 * @param[out] out  Receives the extracted fields.  Must be non-NULL.
 * @retval true   Extraction succeeded.
 * @retval false  @p out is NULL.
 */
bool subghz_signal_fields_came_atomo_extract(uint64_t                      key,
                                             subghz_came_atomo_fields_t   *out);

/**
 * Inverse of @ref subghz_signal_fields_came_atomo_extract — assemble a
 * 62-bit Flipper-format CAME Atomo key from plaintext fields by
 * encrypting and applying the over-the-air transformation.
 *
 * @param[in]  fields   Fields to encode.  Must be non-NULL.
 * @param[out] key_out  Receives the assembled 62-bit key.  Must be non-NULL.
 * @retval true   Assembly succeeded.
 * @retval false  @p fields or @p key_out is NULL.
 */
bool subghz_signal_fields_came_atomo_assemble(
    const subghz_came_atomo_fields_t *fields,
    uint64_t                         *key_out);

/*============================================================================*/
/* CAME Atomo rolling counter — decode / encode (P4)                           */
/*============================================================================*/

/**
 * Decode the 16-bit rolling counter from a CAME Atomo Flipper key.
 * Convenience wrapper — extracts and decrypts in one call.
 *
 * @param  key  62-bit Flipper key.
 * @return 16-bit rolling counter.
 */
uint16_t subghz_signal_fields_came_atomo_counter_decode(uint64_t key);

/**
 * Substitute @p new_counter into the CAME Atomo plaintext, preserving
 * serial, button, and cnt_2, then re-encrypt and return the 62-bit key.
 *
 * @param  key          Current 62-bit Flipper key.
 * @param  new_counter  Replacement 16-bit counter value.
 * @return New 62-bit Flipper key with the substituted counter.
 */
uint64_t subghz_signal_fields_came_atomo_counter_encode(uint64_t key,
                                                        uint16_t new_counter);

/*============================================================================*/
/* Alutech AT-4N field extraction / assembly (P4)                              */
/*============================================================================*/

/**
 * Return true iff @p protocol is the Alutech AT-4N protocol.
 *
 * @param[in] protocol  Protocol name string, may be NULL.
 * @return  true if the protocol is Alutech AT-4N.
 */
bool subghz_signal_fields_is_alutech_at_4n(const char *protocol);

/**
 * Extracted fields from a 72-bit Alutech AT-4N Flipper-format key.
 *
 * The 72-bit key consists of 64 encrypted data bits + 8 CRC bits.
 * After decryption the plaintext layout is:
 *   Byte[0]     CRC check byte
 *   Byte[1:4]   32-bit serial (big-endian)
 *   Byte[5:6]   16-bit counter (big-endian)
 *   Byte[7]     button code
 */
typedef struct {
    uint32_t serial;    /**< 32-bit device serial */
    uint16_t counter;   /**< 16-bit rolling counter */
    uint8_t  button;    /**< button code */
    uint8_t  frame_crc; /**< 8-bit frame CRC (not stored in key; zero) */
    uint64_t enc_data;  /**< 64-bit encrypted data block (= key value) */
} subghz_alutech_at_4n_fields_t;

/**
 * Extract the {serial, counter, button} tuple from an Alutech AT-4N
 * Flipper key by decrypting with the provided rainbow table.
 *
 * @param[in]  key    64-bit Flipper key (encrypted data block).
 * @param[in]  table  32-byte rainbow table.  Must not be NULL.
 * @param[out] out    Receives the extracted fields.  Must be non-NULL.
 * @retval true   Extraction succeeded.
 * @retval false  @p table or @p out is NULL.
 */
bool subghz_signal_fields_alutech_at_4n_extract(
    uint64_t                        key,
    const uint8_t                   table[32],
    subghz_alutech_at_4n_fields_t  *out);

/**
 * Inverse of @ref subghz_signal_fields_alutech_at_4n_extract — assemble
 * a 64-bit Flipper-format Alutech AT-4N key from plaintext fields by
 * computing the CRC check byte and encrypting.
 *
 * @param[in]  fields   Fields to encode.  Must be non-NULL.
 * @param[in]  table    32-byte rainbow table.  Must not be NULL.
 * @param[out] key_out  Receives the assembled 64-bit key.  Must be non-NULL.
 * @retval true   Assembly succeeded.
 * @retval false  @p fields, @p table, or @p key_out is NULL.
 */
bool subghz_signal_fields_alutech_at_4n_assemble(
    const subghz_alutech_at_4n_fields_t *fields,
    const uint8_t                        table[32],
    uint64_t                            *key_out);

/*============================================================================*/
/* Alutech AT-4N rolling counter — decode / encode (P4)                        */
/*============================================================================*/

/**
 * Decrypt the 64-bit encrypted data with the rainbow table and return
 * the 16-bit rolling counter.
 *
 * @param  enc_data  64-bit encrypted data block.
 * @param  table     32-byte rainbow table.  Must not be NULL.
 * @return 16-bit rolling counter.
 */
uint16_t subghz_signal_fields_alutech_at_4n_counter_decode(
    uint64_t       enc_data,
    const uint8_t  table[32]);

/**
 * Substitute @p new_counter into the plaintext obtained by decrypting
 * @p enc_data, recompute the CRC check byte, and return the re-encrypted
 * 64-bit data block.
 *
 * @param  enc_data      Current 64-bit encrypted data block.
 * @param  new_counter   Replacement 16-bit counter value.
 * @param  table         32-byte rainbow table.
 * @return New 64-bit encrypted data block with the substituted counter.
 */
uint64_t subghz_signal_fields_alutech_at_4n_counter_encode(
    uint64_t       enc_data,
    uint16_t       new_counter,
    const uint8_t  table[32]);

/*============================================================================*/
/* Counter-edit capability probe (Phase 9e-1)                                  */
/*============================================================================*/

/**
 * Classification returned by @ref subghz_signal_fields_counter_edit_status.
 *
 * Used by the SignalSettings scene (and SavedMenu gating) to decide whether
 * to expose a Counter editor row for the currently-loaded `.sub` file and,
 * when not yet supported, to display a clear deferred-implementation
 * placeholder rather than a misleading "Protocol not supported" message.
 */
typedef enum {
    /**
     * The protocol has a fully-implemented counter decode/encode path
     * (currently: KeeLoq, Star Line, Jarolift — see Phase 9c-1).
     */
    SUBGHZ_COUNTER_EDIT_SUPPORTED = 0,

    /**
     * The protocol is on the Phase 9e roadmap but the counter
     * extract/substitute path is not yet implemented.  The accompanying
     * reason string returned via the @c out_reason out-parameter explains
     * the specific obstacle (cipher dependency, lookup table, checksum
     * recompute, etc.).  The SignalSettings scene should show this string
     * verbatim so the user can distinguish a deferred protocol from a
     * fully-unsupported one.
     */
    SUBGHZ_COUNTER_EDIT_DEFERRED,

    /**
     * The protocol's counter (if any) is not in scope for editing —
     * static-OOK families, raw recordings, or unknown protocols.
     */
    SUBGHZ_COUNTER_EDIT_UNSUPPORTED,
} subghz_counter_edit_status_t;

/**
 * Classify the counter-editing status of @p protocol.
 *
 * The function performs no I/O and no cipher work — it is a pure
 * name-based lookup over a small static table.  The reason string,
 * when written, points to a static read-only literal owned by this
 * module; callers must not free it.
 *
 * Supported protocols (DEFERRED reason strings cite the specific blocker
 * documented in the Phase 9e checklist entry):
 *
 *   - "KeeLoq", "Star Line", "Jarolift"           → SUPPORTED
 *   - "Nice FloR-S"                               → SUPPORTED
 *   - "CAME Atomo"                                → SUPPORTED
 *   - "Alutech AT-4N"                             → SUPPORTED (with rainbow table)
 *   - "Phoenix_V2"                                → DEFERRED ("Phoenix V2: checksum recompute req.")
 *   - everything else (incl. NULL)                → UNSUPPORTED ("" — empty string)
 *
 * Protocol-name matching follows the same case-insensitive
 * NUL-or-space-terminated prefix convention as
 * @ref subghz_signal_fields_is_keeloq_family.
 *
 * @param[in]  protocol    Protocol name string; may be NULL.
 * @param[out] out_reason  Optional.  When non-NULL, on return *out_reason
 *                         points to a static string describing the status
 *                         (deferred reason for DEFERRED; empty "" for
 *                         SUPPORTED and UNSUPPORTED).  Never NULL on
 *                         return when @p out_reason is non-NULL.
 * @return  Counter-edit status classification.
 */
subghz_counter_edit_status_t
subghz_signal_fields_counter_edit_status(const char  *protocol,
                                         const char **out_reason);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_SIGNAL_FIELDS_H */
