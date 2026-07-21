/* See COPYING.txt for license details. */

/**
 * @file   subghz_keeloq_create.h
 * @brief  KeeLoq-family "create from scratch" 64-bit Flipper key assembler
 *         — pure logic, host-testable (Phase 8c-3).
 *
 * Given a user-entered serial, button, rolling counter, and a manufacturer
 * master key + learn type, this module builds the 64-bit Flipper SubGhz
 * Key File representation that the existing
 * `Sub_Ghz/subghz_keeloq_encoder.c::keeloq_encode_replay()` consumes during
 * playback.
 *
 * The plaintext HOP word follows the Microchip HCS301 layout used by
 * KeeLoq, Star Line, and Jarolift in Flipper-format .sub files
 * (see `subghz_keeloq.h::keeloq_increment_hop` for the layout):
 *
 *   [31:16] rolling counter (16 bits)
 *   [15:12] button code     (4 bits)
 *   [11:10] VLOW            (0)
 *   [ 9: 4] discriminant    (low 6 bits of serial)
 *   [ 3: 0] overflow        (0)
 *
 * The 64-bit Flipper key layout is protocol-dependent and matches the
 * encoder's `reconstruct_key()` exactly:
 *
 *   KeeLoq / Jarolift (Bit:64):
 *     [63:60] button[3:0]
 *     [59:32] serial[27:0]
 *     [31: 0] encrypted HOP
 *
 *   Star Line (Bit:64):
 *     [63:32] encrypted HOP
 *     [31: 4] serial[27:0]
 *     [ 3: 0] button[3:0]
 *
 * The module has zero hardware dependencies — no SI4463, no HAL, no
 * FreeRTOS, no FAT FS — and is fully testable on the host.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_KEELOQ_CREATE_H
#define SUBGHZ_KEELOQ_CREATE_H

#include <stdbool.h>
#include <stdint.h>

#include "subghz_keeloq_mfkeys.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* Result codes                                                                */
/*============================================================================*/

typedef enum {
    KEELOQ_CREATE_OK            = 0,  /**< Success */
    KEELOQ_CREATE_BAD_PROTOCOL  = 1,  /**< Unsupported protocol name */
    KEELOQ_CREATE_BAD_LEARN     = 2,  /**< Unsupported learn type (e.g. Secure) */
    KEELOQ_CREATE_BAD_ARG       = 3,  /**< NULL pointer or other invalid argument */
} KeeLoqCreateResult;

/*============================================================================*/
/* Inputs                                                                      */
/*============================================================================*/

typedef struct {
    const char     *protocol;     /**< Protocol name — e.g. "KeeLoq", "Star Line", "Jarolift" */
    uint32_t        serial;       /**< 28-bit device serial */
    uint8_t         button;       /**<  4-bit button code */
    uint32_t        counter;      /**< 16-bit rolling counter */
    uint64_t        mfr_key;      /**< 64-bit manufacturer master key */
    KeeLoqLearnType learn_type;   /**< Manufacturer-key learn mode (Normal / Simple) */
} KeeLoqCreateParams;

/*============================================================================*/
/* API                                                                         */
/*============================================================================*/

/**
 * Build the 64-bit Flipper SubGhz Key File key value for a freshly-created
 * KeeLoq-family remote.
 *
 * Steps:
 *   1. Build the plaintext HOP word from @p params.counter, @p params.button,
 *      and the low 6 bits of @p params.serial (discriminant).  VLOW and the
 *      4-bit overflow counter are set to 0.
 *   2. Derive the per-device key from @p params.mfr_key and @p params.serial
 *      using @p params.learn_type (Normal or Simple).  Secure Learning is
 *      not supported here because Create-from-scratch has no seed.
 *   3. Encrypt the plaintext HOP with the device key (528-round NLFSR).
 *   4. Assemble the 64-bit Flipper key word from the encrypted HOP, serial,
 *      and button, using the protocol-specific layout described in the
 *      file header.
 *
 * @param[in]  params   Pointer to filled-in parameters (must be non-NULL).
 * @param[out] key_out  Receives the assembled 64-bit Flipper key value.
 *                      Must be non-NULL.
 * @return  @ref KEELOQ_CREATE_OK on success, error code otherwise.  On
 *          failure @p *key_out is set to 0.
 */
KeeLoqCreateResult subghz_keeloq_create_key(const KeeLoqCreateParams *params,
                                             uint64_t                 *key_out);

/**
 * Build only the plaintext HOP word (handy for tests and diagnostics).
 *
 * Layout: see file header.  The low 6 bits of @p serial are used as the
 * discriminant; @p counter is masked to 16 bits and @p button to 4 bits.
 * VLOW and overflow are 0.
 *
 * @param  serial   28-bit device serial
 * @param  button   4-bit button code
 * @param  counter  16-bit rolling counter
 * @return 32-bit plaintext HOP word
 */
uint32_t subghz_keeloq_create_plaintext(uint32_t serial, uint8_t button,
                                         uint32_t counter);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_KEELOQ_CREATE_H */
