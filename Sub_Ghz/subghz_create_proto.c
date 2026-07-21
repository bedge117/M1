/* See COPYING.txt for license details. */

/*
 * subghz_create_proto.c
 *
 * Pure-logic "Create from scratch" protocol catalog — see header for the
 * design rationale.  Zero hardware dependencies.
 *
 * Phase 8a of the Momentum parity migration.
 *
 * M1 Project — Hapax fork
 */

#include "subghz_create_proto.h"

/*============================================================================*/
/* Static protocol catalog                                                    */
/*============================================================================*/

/*
 * The five protocols below are the OOK-PWM rolling-code remotes that the
 * M1 can both build from a user-entered key and replay indefinitely via
 * the standard PWM key encoder.  Metadata mirrors the Bind New Remote
 * generator (see Sub_Ghz/subghz_new_remote_gen.c proto_specs[]) so the
 * two stay byte-for-byte consistent — Phase 8b will dedupe once the
 * SetType scene lands.
 *
 * Bit-count notes:
 *   - Alutech AT-4N: registry min_count_bit is 72, but the decoder also
 *     stores at most 64 bits.  64-bit keys transmit all data the
 *     decoder can recover within that limit.
 */
static const SubGhzCreateProtoSpec s_proto_catalog[SUBGHZ_CREATE_PROTO_COUNT] = {
    [SUBGHZ_CREATE_PROTO_CAME_ATOMO] = {
        .label       = "CAME Atomo 433",
        .proto_name  = "CAME Atomo",
        .freq_hz     = 433920000U,
        .bit_count   = 62U,
        .te          = 200U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "CameAtomo",
    },
    [SUBGHZ_CREATE_PROTO_NICE_FLOR_S] = {
        .label       = "Nice FloR-S 433",
        .proto_name  = "Nice FloR-S",
        .freq_hz     = 433920000U,
        .bit_count   = 52U,
        .te          = 500U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "NiceFlors",
    },
    [SUBGHZ_CREATE_PROTO_ALUTECH_AT4N] = {
        .label       = "Alutech AT-4N 433",
        .proto_name  = "Alutech AT-4N",
        .freq_hz     = 433920000U,
        .bit_count   = 64U,
        .te          = 400U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "Alutech",
    },
    [SUBGHZ_CREATE_PROTO_DITEC_GOL4] = {
        .label       = "DITEC GOL4 433",
        .proto_name  = "DITEC_GOL4",
        .freq_hz     = 433920000U,
        .bit_count   = 54U,
        .te          = 400U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "DitecGol4",
    },
    [SUBGHZ_CREATE_PROTO_KINGGATES_STYLO4K] = {
        .label       = "KingGates Stylo4k 433",
        .proto_name  = "KingGates Stylo4k",
        .freq_hz     = 433920000U,
        .bit_count   = 60U,
        .te          = 400U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "KingGates",
    },

    /*----------------------------------------------------------------------*
     * Phase 8b-1 — Static-OOK families
     *
     * Metadata mirrors the legacy `subghz_add_manually_list[]` in
     * m1_csrc/m1_sub_ghz.c plus Holtek HT12X (which the registry already
     * supports but Add Manually does not yet expose).  The proto_name is
     * the canonical name from Sub_Ghz/subghz_protocol_registry.c so the
     * .sub Protocol: field that the upcoming Phase 8b-4 writer emits will
     * match the receiver decoder exactly — fixing two long-standing bugs
     * in the legacy code path (`Nice FLO 12b` → "Nice" and `Gate TX 433`
     * → "Gate"; both gated only through the strstr() fallback today).
     *
     * The legacy table also stored a `ratio` field (1:2 vs 1:3 OOK PWM)
     * used to derive te_long.  That ratio is **not** carried in this
     * catalog — `subghz_key_encoder.c` derives te_long from the registry
     * `te_long / te_short` ratio when reg_idx is known, so the catalog
     * only needs to carry te_short.  The strstr() fallback in the
     * encoder applies only when the protocol is not in the registry,
     * which is not the case for any entry in this catalog.
     *
     * `Linear 300` is the only entry below 315 MHz; the others span
     * 315 / 433 / 868 bands.
     *----------------------------------------------------------------------*/
    [SUBGHZ_CREATE_PROTO_PRINCETON_433] = {
        .label       = "Princeton 433",
        .proto_name  = "Princeton",
        .freq_hz     = 433920000U,
        .bit_count   = 24U,
        .te          = 350U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "Princeton",
    },
    [SUBGHZ_CREATE_PROTO_PRINCETON_315] = {
        .label       = "Princeton 315",
        .proto_name  = "Princeton",
        .freq_hz     = 315000000U,
        .bit_count   = 24U,
        .te          = 350U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "Princeton",
    },
    [SUBGHZ_CREATE_PROTO_NICE_FLO_12_433] = {
        .label       = "Nice FLO 12b",
        .proto_name  = "Nice FLO",
        .freq_hz     = 433920000U,
        .bit_count   = 12U,
        .te          = 700U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "NiceFLO",
    },
    [SUBGHZ_CREATE_PROTO_NICE_FLO_24_433] = {
        .label       = "Nice FLO 24b",
        .proto_name  = "Nice FLO",
        .freq_hz     = 433920000U,
        .bit_count   = 24U,
        .te          = 700U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "NiceFLO",
    },
    [SUBGHZ_CREATE_PROTO_CAME_12_433] = {
        .label       = "CAME 12bit",
        .proto_name  = "CAME",
        .freq_hz     = 433920000U,
        .bit_count   = 12U,
        .te          = 320U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "CAME",
    },
    [SUBGHZ_CREATE_PROTO_CAME_24_433] = {
        .label       = "CAME 24bit",
        .proto_name  = "CAME",
        .freq_hz     = 433920000U,
        .bit_count   = 24U,
        .te          = 320U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "CAME",
    },
    [SUBGHZ_CREATE_PROTO_CAME_12_868] = {
        .label       = "CAME 12b 868",
        .proto_name  = "CAME",
        .freq_hz     = 868350000U,
        .bit_count   = 12U,
        .te          = 320U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "CAME",
    },
    [SUBGHZ_CREATE_PROTO_LINEAR_300] = {
        .label       = "Linear 300",
        .proto_name  = "Linear",
        .freq_hz     = 300000000U,
        .bit_count   = 10U,
        .te          = 500U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "Linear",
    },
    [SUBGHZ_CREATE_PROTO_GATETX_433] = {
        .label       = "Gate TX 433",
        .proto_name  = "GateTX",
        .freq_hz     = 433920000U,
        .bit_count   = 24U,
        .te          = 350U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "GateTX",
    },
    [SUBGHZ_CREATE_PROTO_DOORHAN_315] = {
        .label       = "DoorHan 315",
        .proto_name  = "DoorHan",
        .freq_hz     = 315000000U,
        .bit_count   = 24U,
        .te          = 350U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "DoorHan",
    },
    [SUBGHZ_CREATE_PROTO_DOORHAN_433] = {
        .label       = "DoorHan 433",
        .proto_name  = "DoorHan",
        .freq_hz     = 433920000U,
        .bit_count   = 24U,
        .te          = 350U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "DoorHan",
    },
    [SUBGHZ_CREATE_PROTO_HOLTEK_HT12X_433] = {
        .label       = "Holtek HT12X 433",
        .proto_name  = "Holtek_HT12X",
        .freq_hz     = 433920000U,
        .bit_count   = 12U,
        .te          = 340U,
        .field_flags = SUBGHZ_CREATE_FIELD_KEY,
        .file_prefix = "Holtek",
    },

    /*----------------------------------------------------------------------*
     * Phase 8c-1 — KeeLoq family (counter-mode rolling-code)
     *
     * The three protocols below are the KeeLoq-cipher based remotes that
     * `Sub_Ghz/subghz_keeloq_encoder.c::keeloq_encode_replay()` already
     * supports for counter-mode replay when a manufacturer key is loaded.
     *
     * Unlike Phase 8a's rolling-code remotes (CAME Atomo / Nice FloR-S /
     * etc.) which expose a single opaque hex key, KeeLoq-family devices
     * are described as four discrete fields:
     *   - Serial (28 bits) — device-specific identifier
     *   - Button (4 bits) — pressed-button code
     *   - Counter (16 bits) — rolling counter (post-decrypt)
     *   - Manufacture (name) — selects the manufacturer key for encryption
     *
     * The Phase 8c-2/3 editor scenes will collect these four values,
     * assemble the 64-bit Flipper-format key (KeeLoq/Jarolift:
     * (button<<60)|(serial<<32)|HOP; Star Line: HOP<<32|(serial<<4)|button),
     * write a `.sub` file with the `Manufacture:` field set, and replay
     * via the existing counter-mode encoder.
     *
     * Bit widths are constant across the family.  bit_count is 64 — the
     * size of the assembled Flipper Bit:64 key value, matching the format
     * the encoder consumes.
     *----------------------------------------------------------------------*/
    [SUBGHZ_CREATE_PROTO_KEELOQ] = {
        .label        = "KeeLoq 433",
        .proto_name   = "KeeLoq",
        .freq_hz      = 433920000U,
        .bit_count    = 64U,
        .te           = 400U,
        .field_flags  = (uint16_t)(SUBGHZ_CREATE_FIELD_SERIAL  |
                                   SUBGHZ_CREATE_FIELD_BUTTON  |
                                   SUBGHZ_CREATE_FIELD_COUNTER |
                                   SUBGHZ_CREATE_FIELD_MFKEY),
        .file_prefix  = "KeeLoq",
        .serial_bits  = 28U,
        .button_bits  = 4U,
        .counter_bits = 16U,
    },
    [SUBGHZ_CREATE_PROTO_STAR_LINE] = {
        .label        = "Star Line 433",
        .proto_name   = "Star Line",
        .freq_hz      = 433920000U,
        .bit_count    = 64U,
        .te           = 400U,
        .field_flags  = (uint16_t)(SUBGHZ_CREATE_FIELD_SERIAL  |
                                   SUBGHZ_CREATE_FIELD_BUTTON  |
                                   SUBGHZ_CREATE_FIELD_COUNTER |
                                   SUBGHZ_CREATE_FIELD_MFKEY),
        .file_prefix  = "StarLine",
        .serial_bits  = 28U,
        .button_bits  = 4U,
        .counter_bits = 16U,
    },
    [SUBGHZ_CREATE_PROTO_JAROLIFT] = {
        .label        = "Jarolift 433",
        .proto_name   = "Jarolift",
        .freq_hz      = 433920000U,
        .bit_count    = 64U,
        .te           = 400U,
        .field_flags  = (uint16_t)(SUBGHZ_CREATE_FIELD_SERIAL  |
                                   SUBGHZ_CREATE_FIELD_BUTTON  |
                                   SUBGHZ_CREATE_FIELD_COUNTER |
                                   SUBGHZ_CREATE_FIELD_MFKEY),
        .file_prefix  = "Jarolift",
        .serial_bits  = 28U,
        .button_bits  = 4U,
        .counter_bits = 16U,
    },
};

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

static inline bool id_in_range(SubGhzCreateProtoId id)
{
    return (uint32_t)id < (uint32_t)SUBGHZ_CREATE_PROTO_COUNT;
}

/* Build a mask of @p bits low-order set bits.  bits must be in [0, 64]. */
static inline uint64_t low_mask_u64(uint32_t bits)
{
    if (bits == 0U) {
        return 0U;
    }
    if (bits >= 64U) {
        return ~(uint64_t)0U;
    }
    return ((uint64_t)1U << bits) - 1U;
}

/*============================================================================*/
/* Public API                                                                 */
/*============================================================================*/

uint32_t subghz_create_proto_count(void)
{
    return (uint32_t)SUBGHZ_CREATE_PROTO_COUNT;
}

const SubGhzCreateProtoSpec *subghz_create_proto_spec(SubGhzCreateProtoId id)
{
    if (!id_in_range(id)) {
        return (const SubGhzCreateProtoSpec *)0;
    }
    return &s_proto_catalog[id];
}

bool subghz_create_proto_has_field(SubGhzCreateProtoId id,
                                   SubGhzCreateFieldFlags field)
{
    if (!id_in_range(id)) {
        return false;
    }
    return (s_proto_catalog[id].field_flags & (uint16_t)field) != 0U;
}

bool subghz_create_proto_key_in_range(SubGhzCreateProtoId id,
                                      uint64_t              key_in,
                                      uint64_t             *key_out)
{
    if (!id_in_range(id)) {
        return false;
    }
    const uint64_t mask = low_mask_u64(s_proto_catalog[id].bit_count);
    const uint64_t masked = key_in & mask;
    if (key_out) {
        *key_out = masked;
    }
    return masked == key_in;
}
