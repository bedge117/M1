/* See COPYING.txt for license details. */

/**
 * @file  subghz_keeloq_encoder.h
 * @brief KeeLoq counter-mode rolling-code encoder.
 *
 * Given a captured KeeLoq rolling-code packet (from a Flipper .sub file or an
 * M1-captured signal), and a loaded manufacturer key, this module:
 *   1. Derives the per-device key using the appropriate learning mode.
 *   2. Decrypts the captured encrypted hop word.
 *   3. Increments the 16-bit counter.
 *   4. Re-encrypts the hop word.
 *   5. Encodes the resulting packet as OOK PWM timing pairs.
 *
 * Supported protocols:
 *   - KeeLoq (Bit: 64 Flipper format — FIX[63:32] = (button<<28)|serial, HOP[31:0])
 *   - Star Line (Bit: 64 Flipper format — HOP[63:32], serial[31:4], button[3:0])
 *   - Jarolift (Bit: 64 Flipper format — same layout as KeeLoq)
 *
 * Note: M1-native Bit:66 (KeeLoq) and Bit:72 (Jarolift) captures are NOT
 * supported — the M1 decoder accumulates these into a uint64 via left-shift,
 * losing the low-order HOP bits and making decryption unreliable.
 * Only Flipper-captured Bit:64 files are accepted.
 *
 * Key format (64-bit, Flipper SubGhz Key File compatible):
 *   bits [63:32] = FIX = (button[3:0] << 28) | serial[27:0]
 *   bits [31: 0] = HOP = encrypted rolling-code word
 *
 * Star Line key format:
 *   bits [63:32] = HOP = encrypted rolling-code word
 *   bits [31: 4] = serial[27:0]
 *   bits [ 3: 0] = button[3:0]
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "subghz_keeloq.h"
#include "subghz_keeloq_mfkeys.h"

/*============================================================================*/
/* Result codes                                                                */
/*============================================================================*/

typedef enum {
    KEELOQ_ENC_OK            = 0,   /**< Success */
    KEELOQ_ENC_NO_KEY        = 1,   /**< Manufacturer key not found */
    KEELOQ_ENC_BAD_BITS      = 2,   /**< Unsupported bit count */
    KEELOQ_ENC_BAD_PROTOCOL  = 3,   /**< Protocol not supported */
    KEELOQ_ENC_NOMEM         = 4,   /**< Allocation failure */
} KeeLoqEncResult;

/*============================================================================*/
/* OOK PWM timing                                                              */
/*============================================================================*/

/** Short pulse duration (µs) for KeeLoq OOK PWM. */
#define KEELOQ_TE_SHORT  400U
/** Long pulse duration (µs) for KeeLoq OOK PWM. */
#define KEELOQ_TE_LONG   800U
/** Number of preamble pulses (alternating HIGH/LOW at te_short). */
#define KEELOQ_PREAMBLE_PULSES  12U

/*============================================================================*/
/* SubGhzRawPair — from subghz_key_encoder.h                                  */
/*============================================================================*/

/* Include the shared pair type from the key encoder — avoids double-typedef. */
#include "subghz_key_encoder.h"

/*============================================================================*/
/* Packet parameter struct                                                     */
/*============================================================================*/

typedef struct {
    const char *protocol;    /**< Protocol name string (e.g. "KeeLoq") */
    const char *manufacture; /**< Manufacturer name from .sub file "Manufacture:" */
    uint64_t    key_value;   /**< Raw 64-bit key from .sub file */
    uint32_t    bit_count;   /**< Bit count from .sub file "Bit:" */
    uint32_t    te;          /**< TE from .sub file (0 = use default 400 µs) */
    /**
     * @brief CounterMode policy (Phase 9d).
     *
     * When @c false (default — Increment mode), the encoder runs the standard
     * counter-mode rolling-code path: decrypt → increment 16-bit counter →
     * re-encrypt → transmit.  This is the original behaviour and is required
     * for binding a new remote whose receiver expects every TX to carry a
     * fresh counter.
     *
     * When @c true (Static mode), the encoder skips the counter increment and
     * re-emits the original encrypted hop word verbatim.  Useful for some
     * receivers / debugging scenarios where the user wants to replay the
     * exact captured packet repeatedly.  The captured hop is still subject
     * to the manufacturer-key derivation path so the field layout is
     * preserved; only `keeloq_increment_hop()` is bypassed.
     */
    bool        static_counter;
} KeeLoqEncParams;

/*============================================================================*/
/* API                                                                         */
/*============================================================================*/

/**
 * @brief  Encode a KeeLoq counter-mode replay packet into OOK PWM pairs.
 *
 * Looks up the manufacturer key by name in the loaded keystore, derives the
 * device key, increments the hop counter, re-encrypts, and writes @p reps
 * repetitions of the OOK PWM waveform into a dynamically allocated pair buffer.
 *
 * The returned buffer must be freed by the caller.
 *
 * @param[in]  params      Packet parameters.
 * @param[out] pairs_out   Set to a malloc'd array of timing pairs (caller must free).
 * @param[out] npairs_out  Number of pairs written into the buffer.
 * @param[in]  reps        Number of complete waveform repetitions to encode.
 * @return KEELOQ_ENC_OK on success, error code otherwise.
 */
KeeLoqEncResult keeloq_encode_replay(
    const KeeLoqEncParams *params,
    SubGhzRawPair        **pairs_out,
    uint32_t              *npairs_out,
    uint8_t                reps);

/**
 * @brief  Compute the number of OOK PWM pairs for one KeeLoq repetition.
 * @param  preamble_pulses  Number of preamble pulse-pair entries.
 * @param  data_bits        Number of data bits.
 * @return Total pairs per repetition (preamble + data + guard).
 */
uint32_t keeloq_pairs_per_rep(uint32_t preamble_pulses, uint32_t data_bits);

/**
 * @brief  Check if a protocol name is KeeLoq-cipher based and may support
 *         counter-mode replay when a manufacturer key is available.
 *
 * Returns true for "KeeLoq", "Star Line", "Jarolift".
 *
 * @param  protocol  Protocol name string (case-insensitive).
 * @return true if counter-mode replay may be possible.
 */
bool keeloq_is_keeloq_protocol(const char *protocol);
