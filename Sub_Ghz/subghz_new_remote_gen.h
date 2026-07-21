/* See COPYING.txt for license details. */

/*
 * subghz_new_remote_gen.h
 *
 * Pure-logic random remote key generator for the Bind New Remote wizard.
 *
 * Given a protocol selection and a 64-bit entropy seed, produces the
 * parameters needed to write a valid Flipper .sub key file.  The module
 * has zero hardware dependencies and is fully testable on the host.
 *
 * Usage (on firmware):
 *   uint64_t seed = ((uint64_t)HAL_GetTick() << 32) ^ HAL_GetTick();
 *   NewRemoteParams p;
 *   subghz_new_remote_gen(BW_PROTO_CAME_ATOMO, seed, &p);
 *   // p.proto_name, p.freq_hz, p.key, p.bit_count, p.te → fill flipper_subghz_signal_t
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_NEW_REMOTE_GEN_H
#define SUBGHZ_NEW_REMOTE_GEN_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Protocol IDs                                                               */
/*============================================================================*/

typedef enum {
    BW_PROTO_CAME_ATOMO = 0,   /**< CAME Atomo 433 MHz — 62 bit, OOK PWM, PwmKeyReplay */
    BW_PROTO_NICE_FLOR_S,      /**< Nice FloR-S 433 MHz — 52 bit, OOK PWM, PwmKeyReplay */
    BW_PROTO_ALUTECH_AT4N,     /**< Alutech AT-4N 433 MHz — 64 bit, OOK PWM, PwmKeyReplay */
    BW_PROTO_DITEC_GOL4,       /**< DITEC_GOL4 433 MHz — 54 bit, OOK PWM, PwmKeyReplay */
    BW_PROTO_KINGGATES_STYLO4K,/**< KingGates Stylo4k 433 MHz — 60 bit, OOK PWM, PwmKeyReplay */
    BW_PROTO_COUNT
} BindWizardProto;

/*============================================================================*/
/* Output parameters                                                          */
/*============================================================================*/

/**
 * Generated remote parameters — filled by subghz_new_remote_gen().
 * All fields are ready to copy directly into a flipper_subghz_signal_t.
 */
typedef struct {
    char     proto_name[32]; /**< Protocol name string for .sub Protocol: field */
    uint32_t freq_hz;        /**< Centre frequency in Hz (e.g. 433920000) */
    uint64_t key;            /**< Random key; lower bit_count bits are used */
    uint32_t bit_count;      /**< Number of significant bits in key */
    uint16_t te;             /**< Short timing element in µs; 0 = use registry default */
    char     file_base[40];  /**< Suggested filename base, no path/ext
                                  e.g. "NewRemote_CameAtomo_A1B2C3D4" */
} NewRemoteParams;

/*============================================================================*/
/* API                                                                        */
/*============================================================================*/

/**
 * Return the human-readable display label for a protocol.
 *
 * @param proto  Protocol enum value (must be < BW_PROTO_COUNT).
 * @return       Static string, e.g. "CAME Atomo 433".
 */
const char *subghz_new_remote_proto_label(BindWizardProto proto);

/**
 * Generate random remote parameters for the specified protocol.
 *
 * Uses splitmix64 seeded with @p seed to produce a random key of the
 * correct bit width for @p proto.  The caller is responsible for
 * providing entropy; on firmware this is typically HAL_GetTick() XOR'd
 * with the STM32 UID words.
 *
 * @param proto  Target protocol (must be < BW_PROTO_COUNT).
 * @param seed   64-bit entropy seed.
 * @param out    Output structure (must not be NULL).
 * @return       true on success, false if @p proto is out of range or @p out is NULL.
 */
bool subghz_new_remote_gen(BindWizardProto proto, uint64_t seed, NewRemoteParams *out);

#endif /* SUBGHZ_NEW_REMOTE_GEN_H */
