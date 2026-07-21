/* See COPYING.txt for license details. */

/*
 * subghz_protocol_registry.h
 *
 * Flipper/Momentum-compatible Sub-GHz protocol registry.
 *
 * This header defines the protocol descriptor and registry types that mirror
 * the Flipper Zero firmware architecture (lib/subghz/protocols/).  The goal
 * is to minimise the manual work needed when porting a new protocol from
 * Flipper/Momentum by providing:
 *
 *   1.  A protocol descriptor struct with the same fields Flipper uses
 *       (name, type, timing constants, decode function pointer).
 *
 *   2.  A flat registry array that auto-enumerates protocol indices —
 *       adding a new protocol is a single line in the array.
 *
 *   3.  Flipper-compatible helper macros (DURATION_DIFF, subghz_protocol_blocks_add_bit)
 *       so ported decode logic compiles with minimal changes.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_PROTOCOL_REGISTRY_H
#define SUBGHZ_PROTOCOL_REGISTRY_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/*============================================================================*/
/* Protocol Type / Flag / Filter — mirrors Flipper's lib/subghz/types.h       */
/*============================================================================*/

/** Protocol behaviour type (Flipper: SubGhzProtocolType) */
typedef enum {
    SubGhzProtocolTypeStatic  = 0,   /**< Fixed (non-rolling) code */
    SubGhzProtocolTypeDynamic = 1,   /**< Rolling / encrypted code */
    SubGhzProtocolTypeRAW     = 2,   /**< Raw capture */
    SubGhzProtocolTypeWeather = 3,   /**< Weather station sensor (M1-specific extension) */
    SubGhzProtocolTypeTPMS    = 4,   /**< Tire-pressure sensor   (M1-specific extension) */
} SubGhzProtocolType;

/** Protocol capability / frequency flags (Flipper: SubGhzProtocolFlag) */
typedef enum {
    SubGhzProtocolFlag_315          = (1u << 0),
    SubGhzProtocolFlag_433          = (1u << 1),
    SubGhzProtocolFlag_868          = (1u << 2),
    SubGhzProtocolFlag_AM           = (1u << 3),
    SubGhzProtocolFlag_FM           = (1u << 4),
    SubGhzProtocolFlag_Decodable    = (1u << 5),
    SubGhzProtocolFlag_Load         = (1u << 6),
    SubGhzProtocolFlag_Save         = (1u << 7),
    SubGhzProtocolFlag_Send         = (1u << 8),
    SubGhzProtocolFlag_300          = (1u << 9),  /**< Operates at 300 MHz band */
    /**
     * PwmKeyReplay — set on Dynamic (rolling-code) protocols whose saved Key:
     * value can be faithfully re-encoded as standard OOK PWM using only the
     * registry timing (te_short / te_long).
     *
     * A protocol MUST have this flag set before the key encoder will attempt
     * to produce a RAW pulse stream from it.  Protocols that lack this flag
     * (Manchester-encoded, KeeLoq-cipher, AES-encrypted, FSK, ternary, etc.)
     * are rejected with SUBGHZ_KEY_ERR_DYNAMIC.
     *
     * Research summary (2026-04-17):
     *   CAN replay  (flag set)   : CAME Atomo, CAME TWEE, Nice FloR-S,
     *                              Alutech AT-4N, KingGates Stylo4k, Scher-Khan
     *                              (Magicar + Logicar), Toyota, DITEC_GOL4
     *   CANNOT replay (Manchester): FAAC SLH, Somfy Telis, Somfy Keytis, Revers_RB2
     *   CANNOT replay (cipher)   : KeeLoq, Star Line, Jarolift, Security+ 1.0/2.0
     *   CANNOT replay (AES-128)  : Hormann BiSecur, Beninca ARC
     */
    SubGhzProtocolFlag_PwmKeyReplay = (1u << 10),
} SubGhzProtocolFlag;

/** Category filter (matches Flipper SubGhzProtocolFilter values) */
typedef enum {
    SubGhzProtocolFilter_Auto      = 0,   /**< Accept in auto-detect scan */
    SubGhzProtocolFilter_Weather   = 1,   /**< Weather sensor only */
    SubGhzProtocolFilter_TPMS      = 2,   /**< TPMS only */
    SubGhzProtocolFilter_Industrial= 3,   /**< Industrial / paging (POCSAG, X10) */
} SubGhzProtocolFilter;

/*============================================================================*/
/* Timing constants block — mirrors Flipper's SubGhzBlockConst                */
/*============================================================================*/

typedef struct {
    uint16_t te_short;              /**< Short symbol duration (μs) */
    uint16_t te_long;               /**< Long  symbol duration (μs) */
    uint16_t te_delta;              /**< Timing tolerance (μs, absolute — Flipper style) */
    uint8_t  te_tolerance_pct;      /**< Timing tolerance (%, M1 legacy) — used when te_delta==0 */
    uint8_t  preamble_bits;         /**< Pulse pairs to skip as preamble */
    uint16_t min_count_bit_for_found; /**< Minimum valid bits */
} SubGhzBlockConst;

/*============================================================================*/
/* M1 decode function signature (unchanged from existing decoders)            */
/*============================================================================*/

/**
 * Decoder function pointer.
 *
 * @param protocol_index  Index of this protocol in the registry.
 * @param pulse_count     Number of pulses in the global pulse_times[] buffer.
 * @return 0 on successful decode, 1 on failure.
 *
 * The function reads pulses from subghz_decenc_ctl.pulse_times[] and on
 * success writes results into subghz_decenc_ctl.n64_decodedvalue, etc.
 * This is the existing M1 convention — no changes needed in decoder bodies.
 */
typedef uint8_t (*SubGhzDecodeFn)(uint16_t protocol_index, uint16_t pulse_count);

/*============================================================================*/
/* Polymorphic Info-screen renderer (Phase 11-1)                              */
/*============================================================================*/

/**
 * Lightweight, registry-friendly view of a loaded `.sub` signal.
 *
 * Populated by the caller from a `flipper_subghz_signal_t` (in the firmware
 * Saved scene) or from explicit values (in host tests).  The view contains
 * only primitive types so the registry header does not need to know about
 * `flipper_subghz.h` — `Sub_Ghz/` stays independent of `m1_csrc/`.
 *
 * @field protocol   Protocol name string (may be NULL / empty for RAW).
 * @field key        64-bit Flipper key as carried in the `.sub` file.
 * @field bit_count  Bit width of the key (parsed from `Bit:` line).
 * @field te         Symbol time in microseconds (parsed from `TE:` line).
 */
typedef struct {
    const char *protocol;
    uint64_t    key;
    uint32_t    bit_count;
    uint32_t    te;
} SubGhzSignalView;

/**
 * Polymorphic per-protocol Info-screen renderer.
 *
 * Each protocol may install a renderer that formats the loaded signal as
 * a multi-line human-readable string (rows separated by `\n`).  The caller
 * pre-allocates @p buf of size @p buflen; the renderer must always write a
 * NUL-terminated string (even on truncation), and must never write past
 * @p buflen.
 *
 * Default behaviour (renderer is NULL): the Info screen falls back to the
 * existing generic layout — "Proto: …\nKey: 0x…\nBits: …  TE: …".
 *
 * @param[in]  view   Loaded-signal view (never NULL when invoked by the
 *                    Saved scene; tests may pass NULL — see contract).
 * @param[out] buf    Caller-owned output buffer.
 * @param[in]  buflen Size of @p buf in bytes (including NUL terminator).
 */
typedef void (*SubGhzGetStringFn)(const SubGhzSignalView *view,
                                  char                   *buf,
                                  size_t                  buflen);

/*============================================================================*/
/* Protocol Descriptor — one per protocol                                     */
/*============================================================================*/

/**
 * Complete protocol descriptor.
 *
 * Combines the Flipper SubGhzProtocol metadata with M1's SubGHz_protocol_t
 * timing parameters and decode function pointer.  Adding a new protocol
 * requires filling ONE of these structs and adding it to the registry array.
 */
typedef struct {
    /* --- Identity (Flipper-compatible) --- */
    const char          *name;      /**< Flipper SUBGHZ_PROTOCOL_*_NAME constant */
    SubGhzProtocolType   type;      /**< Static / Dynamic / RAW / Weather / TPMS */
    uint32_t             flags;     /**< SubGhzProtocolFlag bitmask */
    SubGhzProtocolFilter filter;    /**< Category filter */

    /* --- Timing (M1 + Flipper hybrid) --- */
    SubGhzBlockConst     timing;    /**< Pulse timing parameters */

    /* --- Decode (M1 convention) --- */
    SubGhzDecodeFn       decode;    /**< Decoder function (NULL = not implemented) */

    /* --- Info rendering (Phase 11-1) --- */
    SubGhzGetStringFn    get_string; /**< Polymorphic Info renderer (NULL = generic) */
} SubGhzProtocolDef;

/*============================================================================*/
/* Protocol Registry                                                          */
/*============================================================================*/

/** Global registry array (defined in subghz_protocol_registry.c) */
extern const SubGhzProtocolDef subghz_protocol_registry[];

/** Number of protocols in the registry (defined in subghz_protocol_registry.c) */
extern const uint16_t subghz_protocol_registry_count;

/*============================================================================*/
/* Registry Lookup Helpers                                                     */
/*============================================================================*/

/**
 * Portable ASCII case-insensitive string compare (replacement for POSIX strcasecmp).
 * Returns 0 if equal, negative/positive if a < b / a > b.
 */
int subghz_ascii_strcasecmp(const char *a, const char *b);

/** Find a protocol index by its Flipper-compatible name.  Returns -1 if not found. */
int16_t subghz_protocol_find_by_name(const char *name);

/** Get the protocol descriptor by index.  Returns NULL if out of range. */
const SubGhzProtocolDef* subghz_protocol_get(uint16_t index);

/** Get the protocol name by index (equivalent to old protocol_text[]).  Returns NULL if out of range. */
const char* subghz_protocol_get_name(uint16_t index);

/*============================================================================*/
/* Flipper-Compatible Building Blocks                                         */
/*                                                                            */
/* The canonical implementations now live in dedicated headers that mirror     */
/* Flipper's lib/subghz/blocks/ and lib/toolbox/ directories:                 */
/*                                                                            */
/*   subghz_blocks_math.h          — DURATION_DIFF, bit macros, CRC, parity  */
/*   subghz_block_decoder.h        — SubGhzBlockDecoder struct, add_bit       */
/*   subghz_block_generic.h        — SubGhzBlockGeneric struct (decode out)   */
/*   subghz_level_duration.h       — LevelDuration pair (encoder waveforms)   */
/*   subghz_block_encoder.h        — SubGhzProtocolBlockEncoder, bit array    */
/*   subghz_manchester_decoder.h   — manchester_advance() state machine       */
/*   subghz_manchester_encoder.h   — manchester_encoder_advance/reset/finish  */
/*                                                                            */
/* Include them here so any file that includes the registry header             */
/* automatically gets access to all Flipper-compatible helpers.               */
/*============================================================================*/

#include "subghz_blocks_math.h"
#include "subghz_block_decoder.h"
#include "subghz_block_generic.h"
#include "subghz_level_duration.h"
#include "subghz_block_encoder.h"
#include "subghz_manchester_decoder.h"
#include "subghz_manchester_encoder.h"

/*============================================================================*/
/* Convenience Macro for Declaring a Protocol                                 */
/*============================================================================*/

/**
 * SUBGHZ_PROTOCOL_DEFINE(varname, ...)
 *
 * Expands to a static const SubGhzProtocolDef.  Use in the registry array:
 *
 *     const SubGhzProtocolDef subghz_protocol_registry[] = {
 *         SUBGHZ_PROTOCOL_DEFINE(
 *             .name   = "Princeton",
 *             .type   = SubGhzProtocolTypeStatic,
 *             .flags  = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM |
 *                       SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save |
 *                       SubGhzProtocolFlag_Send,
 *             .filter = SubGhzProtocolFilter_Auto,
 *             .timing = { .te_short=370, .te_long=1140, .te_tolerance_pct=20,
 *                         .min_count_bit_for_found=24 },
 *             .decode = subghz_decode_princeton,
 *         ),
 *         // ... more protocols ...
 *     };
 *
 * The macro is trivial (just curly braces) but documents intent and makes
 * grep/search easier across the codebase.
 */
#define SUBGHZ_PROTOCOL_DEFINE(...) { __VA_ARGS__ }

#endif /* SUBGHZ_PROTOCOL_REGISTRY_H */
