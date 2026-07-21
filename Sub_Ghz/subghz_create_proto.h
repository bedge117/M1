/* See COPYING.txt for license details. */

/**
 * @file   subghz_create_proto.h
 * @brief  Sub-GHz "Create from scratch" protocol catalog & field-schema
 *         descriptor — pure logic, host-testable.
 *
 * Phase 8a of the Momentum parity migration.  Provides the protocol
 * catalog and per-protocol editable-field bitmask that the upcoming
 * SetType (Phase 8b) and SetKey/SetSerial/SetButton/SetCounter
 * (Phase 8c) scenes will consume.
 *
 * The catalog covers the set of protocols for which the M1 can both
 * **build a key from user-entered fields** and **transmit it
 * indefinitely** via the standard OOK PWM key encoder
 * (`SubGhzProtocolFlag_PwmKeyReplay`).  Phase 8a ships with the five
 * rolling-code protocols already exercised by the Bind New Remote
 * wizard (CAME Atomo / Nice FloR-S / Alutech AT-4N / DITEC GOL4 /
 * KingGates Stylo4k); Phase 8b/8c will extend the catalog to cover
 * KeeLoq counter-mode (Serial+Button+Counter+MfKey) and the static
 * OOK families currently served by the "Add Manually" delegate
 * (Princeton, Nice FLO, CAME 12/24-bit, Linear, GateTX, DoorHan).
 *
 * The module has zero hardware dependencies — no SI4463, no HAL, no
 * FreeRTOS, no FAT FS — and is fully testable on the host.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_CREATE_PROTO_H
#define SUBGHZ_CREATE_PROTO_H

#include <stdbool.h>
#include <stdint.h>

/*============================================================================*/
/* Protocol IDs                                                               */
/*============================================================================*/

typedef enum {
    /* Rolling-code remotes (Phase 8a) — also covered by the Bind New Remote
     * wizard, which generates a random key.  In the Create-from-scratch flow
     * the user enters the hex key directly. */
    SUBGHZ_CREATE_PROTO_CAME_ATOMO = 0,        /**< CAME Atomo 433 — 62 bit OOK PWM */
    SUBGHZ_CREATE_PROTO_NICE_FLOR_S,           /**< Nice FloR-S 433 — 52 bit OOK PWM */
    SUBGHZ_CREATE_PROTO_ALUTECH_AT4N,          /**< Alutech AT-4N 433 — 64 bit OOK PWM */
    SUBGHZ_CREATE_PROTO_DITEC_GOL4,            /**< DITEC GOL4 433 — 54 bit OOK PWM */
    SUBGHZ_CREATE_PROTO_KINGGATES_STYLO4K,     /**< KingGates Stylo4k 433 — 60 bit OOK PWM */

    /* Static-OOK families (Phase 8b-1) — currently served by the legacy
     * `sub_ghz_add_manually()` blocking delegate.  Phase 8b-2/8b-4 will move
     * the picker UI into a scene and retire that delegate. */
    SUBGHZ_CREATE_PROTO_PRINCETON_433,         /**< Princeton 433 — 24 bit OOK PWM 1:3 */
    SUBGHZ_CREATE_PROTO_PRINCETON_315,         /**< Princeton 315 — 24 bit OOK PWM 1:3 */
    SUBGHZ_CREATE_PROTO_NICE_FLO_12_433,       /**< Nice FLO 433 — 12 bit OOK PWM 1:2 */
    SUBGHZ_CREATE_PROTO_NICE_FLO_24_433,       /**< Nice FLO 433 — 24 bit OOK PWM 1:2 */
    SUBGHZ_CREATE_PROTO_CAME_12_433,           /**< CAME 433 — 12 bit OOK PWM 1:2 */
    SUBGHZ_CREATE_PROTO_CAME_24_433,           /**< CAME 433 — 24 bit OOK PWM 1:2 */
    SUBGHZ_CREATE_PROTO_CAME_12_868,           /**< CAME 868 — 12 bit OOK PWM 1:2 */
    SUBGHZ_CREATE_PROTO_LINEAR_300,            /**< Linear 300 — 10 bit OOK PWM 1:3 */
    SUBGHZ_CREATE_PROTO_GATETX_433,            /**< GateTX 433 — 24 bit OOK PWM 1:3 */
    SUBGHZ_CREATE_PROTO_DOORHAN_315,           /**< DoorHan 315 — 24 bit OOK PWM 1:3 */
    SUBGHZ_CREATE_PROTO_DOORHAN_433,           /**< DoorHan 433 — 24 bit OOK PWM 1:3 */
    SUBGHZ_CREATE_PROTO_HOLTEK_HT12X_433,      /**< Holtek HT12X 433 — 12 bit OOK PWM 1:3 */

    /* KeeLoq family (Phase 8c-1) — counter-mode rolling-code remotes that
     * the existing `Sub_Ghz/subghz_keeloq_encoder.c` already supports for
     * replay.  Unlike the Phase 8a rolling-code remotes (which expose a
     * single opaque hex key), these protocols are edited as discrete
     * Serial / Button / Counter / Manufacturer-key fields and assembled
     * into the final 64-bit key by the upcoming Phase 8c-2/3 editor
     * scenes. */
    SUBGHZ_CREATE_PROTO_KEELOQ,                /**< KeeLoq 433 — 64 bit OOK PWM (Flipper Bit:64) */
    SUBGHZ_CREATE_PROTO_STAR_LINE,             /**< Star Line 433 — 64 bit OOK PWM */
    SUBGHZ_CREATE_PROTO_JAROLIFT,              /**< Jarolift 433 — 64 bit OOK PWM (Flipper Bit:64) */

    SUBGHZ_CREATE_PROTO_COUNT
} SubGhzCreateProtoId;

/*============================================================================*/
/* Editable field bitmask                                                     */
/*============================================================================*/

/**
 * Bitmask of user-editable fields on a protocol's create-from-scratch form.
 *
 * @c SUBGHZ_CREATE_FIELD_KEY is the catch-all "the whole hop word is one
 * opaque hex value"; the five rolling-code protocols in the initial Phase 8a
 * catalog all use this representation.  The remaining flags drive the
 * Phase 8c KeeLoq counter-mode workflow, where the device is described as
 * separate Serial / Button / Counter / Manufacturer-key components and the
 * 64-bit key is assembled by the encoder.
 */
typedef enum {
    SUBGHZ_CREATE_FIELD_KEY     = 1U << 0,     /**< Single opaque hex key */
    SUBGHZ_CREATE_FIELD_SERIAL  = 1U << 1,     /**< KeeLoq family serial */
    SUBGHZ_CREATE_FIELD_BUTTON  = 1U << 2,     /**< KeeLoq family button code */
    SUBGHZ_CREATE_FIELD_COUNTER = 1U << 3,     /**< KeeLoq family rolling counter */
    SUBGHZ_CREATE_FIELD_MFKEY   = 1U << 4      /**< KeeLoq manufacturer key name */
} SubGhzCreateFieldFlags;

/*============================================================================*/
/* Protocol spec                                                              */
/*============================================================================*/

typedef struct {
    const char *label;        /**< Display label, e.g. "CAME Atomo 433" */
    const char *proto_name;   /**< Exact registry name for .sub Protocol: field */
    uint32_t    freq_hz;      /**< Centre frequency in Hz */
    uint32_t    bit_count;    /**< Significant bit width of the key (≤ 64) */
    uint16_t    te;           /**< te_short override in µs (0 = registry default) */
    uint16_t    field_flags;  /**< OR of @ref SubGhzCreateFieldFlags */
    const char *file_prefix;  /**< Suggested filename prefix, no spaces */
    /* KeeLoq-family field widths (Phase 8c-1).  Non-zero only when the
     * matching FIELD_SERIAL / FIELD_BUTTON / FIELD_COUNTER flag is set.
     * The editor scenes use these to size their per-field hex/decimal
     * cursors; the encoder uses them to mask the user-entered values
     * before assembling the 64-bit key. */
    uint8_t     serial_bits;  /**< Serial bit width (e.g. 28 for KeeLoq) */
    uint8_t     button_bits;  /**< Button bit width (e.g. 4 for KeeLoq) */
    uint8_t     counter_bits; /**< Counter bit width (e.g. 16 for KeeLoq) */
} SubGhzCreateProtoSpec;

/*============================================================================*/
/* API                                                                        */
/*============================================================================*/

/**
 * @return The number of protocols in the catalog.  Always equal to
 *         @ref SUBGHZ_CREATE_PROTO_COUNT.
 */
uint32_t subghz_create_proto_count(void);

/**
 * @param  id  Protocol identifier (must be < @ref SUBGHZ_CREATE_PROTO_COUNT).
 * @return Pointer to the static spec, or NULL if @p id is out of range.
 *         The returned pointer is owned by the module and remains valid
 *         for the lifetime of the program.
 */
const SubGhzCreateProtoSpec *subghz_create_proto_spec(SubGhzCreateProtoId id);

/**
 * Check whether a protocol exposes a particular editable field.
 *
 * @param id     Protocol identifier.
 * @param field  Single @ref SubGhzCreateFieldFlags value to test.
 * @return       true if @p field is set in the protocol's field_flags,
 *               false otherwise or if @p id is out of range.
 */
bool subghz_create_proto_has_field(SubGhzCreateProtoId id,
                                   SubGhzCreateFieldFlags field);

/**
 * Mask a user-entered key to the protocol's bit width and report whether
 * the original value fit.
 *
 * @param id       Protocol identifier.
 * @param key_in   Raw user-entered key value.
 * @param key_out  If non-NULL, receives @p key_in masked to the
 *                 protocol's bit_count.  Always written when @p id is
 *                 valid, even on overflow.
 * @return         true if @p key_in fits in @p bit_count bits without
 *                 truncation; false if any high bits had to be stripped
 *                 or if @p id is out of range.
 */
bool subghz_create_proto_key_in_range(SubGhzCreateProtoId id,
                                      uint64_t              key_in,
                                      uint64_t             *key_out);

#endif /* SUBGHZ_CREATE_PROTO_H */
