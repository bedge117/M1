/* See COPYING.txt for license details. */

/*
 * subghz_key_encoder.c
 *
 * KEY→RAW OOK PWM encoder — extracted from sub_ghz_replay_flipper_file()
 * in m1_sub_ghz.c (lines 1735-1850).
 *
 * This module is hardware-independent.  It uses only the protocol registry
 * for timing lookup and performs pure arithmetic for signal generation.
 *
 * M1 Project — Hapax fork
 */

#include <string.h>
#include "subghz_key_encoder.h"
#include "subghz_protocol_registry.h"

/*============================================================================*/
/* subghz_key_resolve_timing                                                   */
/*============================================================================*/

uint8_t subghz_key_resolve_timing(const SubGhzKeyParams *params, SubGhzKeyTiming *timing)
{
    if (!params || !timing)
        return SUBGHZ_KEY_ERR_UNSUPPORTED;

    int16_t reg_idx = subghz_protocol_find_by_name(params->protocol);

    /* Weather sensors and TPMS — cannot encode. */
    if (reg_idx >= 0)
    {
        const SubGhzProtocolDef *proto = &subghz_protocol_registry[reg_idx];
        if (proto->type == SubGhzProtocolTypeWeather ||
            proto->type == SubGhzProtocolTypeTPMS)
        {
            return SUBGHZ_KEY_ERR_DYNAMIC;
        }

        /* Rolling-code protocols: only allow replay when the registry explicitly
         * marks the protocol as OOK-PWM-replayable via SubGhzProtocolFlag_PwmKeyReplay.
         *
         * Protocols WITHOUT this flag fall through to SUBGHZ_KEY_ERR_DYNAMIC because
         * they are one of:
         *   - Manchester-encoded (FAAC SLH, Somfy Telis/Keytis, Revers_RB2):
         *       the decoded Key: value cannot be re-encoded using PWM timings
         *   - KeeLoq-cipher-based (KeeLoq, Star Line, Jarolift):
         *       requires manufacturer key decryption; raw counter replay is invalid
         *   - AES-128-encrypted (Hormann BiSecur, Beninca ARC):
         *       not feasible to replay without the AES key
         *   - Ternary / 2-sub-packet (Security+ 1.0):
         *       has a dedicated brute-force counter mode; plain PWM re-encode is wrong
         *   - FSK / bidirectional challenge-response (Security+ 2.0):
         *       not OOK; receiver sends a challenge that must be answered
         *   - Special preamble/start encoding (KIA Seed):
         *       preamble + start-bit format is not standard OOK PWM
         *
         * Protocols WITH SubGhzProtocolFlag_PwmKeyReplay (CAME Atomo, Nice FloR-S,
         * Alutech AT-4N, CAME TWEE, KingGates Stylo4k, Scher-Khan Magicar/Logicar,
         * Toyota, DITEC_GOL4) use plain OOK PWM with a fixed te_short/te_long ratio
         * and carry no per-code encryption, so a captured Key: value can be
         * faithfully retransmitted. */
        if (proto->type == SubGhzProtocolTypeDynamic)
        {
            if (!(proto->flags & SubGhzProtocolFlag_PwmKeyReplay))
                return SUBGHZ_KEY_ERR_DYNAMIC;
            /* PwmKeyReplay set: fall through to registry-based timing */
        }
    }

    /* Registry-based timing: use actual te_short / te_long from protocol.
     * This now covers both Static and Dynamic (OOK-PWM-encodable rolling) protocols. */
    if (reg_idx >= 0 &&
        (subghz_protocol_registry[reg_idx].type == SubGhzProtocolTypeStatic ||
         subghz_protocol_registry[reg_idx].type == SubGhzProtocolTypeDynamic) &&
        subghz_protocol_registry[reg_idx].timing.te_short > 0 &&
        subghz_protocol_registry[reg_idx].timing.te_long > 0)
    {
        const SubGhzBlockConst *t = &subghz_protocol_registry[reg_idx].timing;
        timing->te_short = (params->te > 0) ? params->te : t->te_short;
        timing->te_long  = (uint32_t)timing->te_short * t->te_long / t->te_short;
        timing->gap_low  = timing->te_short * 30;
    }
    /* Legacy strstr() fallback for unknown protocols */
    else if (strstr(params->protocol, "Princeton") || strstr(params->protocol, "Gate") ||
             strstr(params->protocol, "Holtek") || strstr(params->protocol, "Linear") ||
             strstr(params->protocol, "SMC5326") || strstr(params->protocol, "Power") ||
             strstr(params->protocol, "iDo"))
    {
        /* 1:3 ratio protocols */
        timing->te_short = (params->te > 0) ? params->te : 350;
        timing->te_long  = timing->te_short * 3;
        timing->gap_low  = timing->te_short * 30;
    }
    else if (strstr(params->protocol, "CAME") || strstr(params->protocol, "Nice") ||
             strstr(params->protocol, "Ansonic"))
    {
        /* 1:2 ratio protocols */
        timing->te_short = (params->te > 0) ? params->te : 320;
        timing->te_long  = timing->te_short * 2;
        timing->gap_low  = timing->te_short * 36;
    }
    else if (params->te > 0)
    {
        /* Generic fallback: unknown protocol, but .sub file provides TE.
         * Use the TE value with a 1:3 ratio — this is the default encoding
         * ratio used by Princeton and most OOK PWM garage/gate protocols.
         * The 1:3 ratio is a heuristic; protocols with 1:2 encoding
         * (CAME, Nice FLO) should be added to the registry for accuracy.
         * This fallback enables best-effort emulation of third-party
         * firmware protocols and regional gate systems not in our registry. */
        timing->te_short = params->te;
        timing->te_long  = timing->te_short * 3;
        timing->gap_low  = timing->te_short * 30;
    }
    else
    {
        return SUBGHZ_KEY_ERR_UNSUPPORTED;
    }

    return SUBGHZ_KEY_OK;
}

/*============================================================================*/
/* subghz_key_encode                                                           */
/*============================================================================*/

uint32_t subghz_key_encode(const SubGhzKeyParams *params,
                            const SubGhzKeyTiming *timing,
                            SubGhzRawPair *out,
                            uint32_t max_pairs,
                            uint8_t repetitions)
{
    if (!params || !timing || !out || max_pairs == 0 || repetitions == 0)
        return 0;

    /* Clamp bit count to 64 (uint64_t max) */
    uint32_t bit_count = params->bit_count;
    if (bit_count > 64) bit_count = 64;
    if (bit_count == 0) return 0;

    /* Each repetition = bit_count data pairs + 1 sync gap pair */
    uint32_t pairs_per_rep = bit_count + 1;
    uint32_t total_pairs   = (uint32_t)repetitions * pairs_per_rep;

    if (total_pairs > max_pairs)
        return 0;   /* overflow */

    uint32_t idx = 0;

    for (uint8_t rep = 0; rep < repetitions; rep++)
    {
        uint64_t mask = 1ULL << (bit_count - 1);

        for (uint32_t b = 0; b < bit_count; b++)
        {
            if (params->key_value & mask)
            {
                /* Bit 1: long HIGH, short LOW */
                out[idx].high_us = timing->te_long;
                out[idx].low_us  = timing->te_short;
            }
            else
            {
                /* Bit 0: short HIGH, long LOW */
                out[idx].high_us = timing->te_short;
                out[idx].low_us  = timing->te_long;
            }
            mask >>= 1;
            idx++;
        }

        /* Sync gap: short HIGH pulse + long LOW gap */
        out[idx].high_us = timing->te_short;
        out[idx].low_us  = timing->gap_low;
        idx++;
    }

    return idx;
}

/*============================================================================*/
/* subghz_low_te_calc_reps                                                     */
/*============================================================================*/

uint8_t subghz_low_te_calc_reps(uint32_t key_te)
{
    if (key_te == 0 || key_te >= 250)
        return 3;

    /* Scale so each burst ≈ 140ms (TE=370, 3 reps baseline).
     * Use ceiling division: ceil(3 * 370 / key_te). */
    uint8_t reps = (uint8_t)((3U * 370U + key_te - 1U) / key_te);
    if (reps > 12) reps = 12;
    if (reps < 3)  reps = 3;
    return reps;
}

/*============================================================================*/
/* Protocol-specific encoders                                                  */
/*============================================================================*/

bool subghz_key_has_custom_encoder(const char *protocol)
{
    if (!protocol) return false;
    return (subghz_ascii_strcasecmp(protocol, "Magellan") == 0);
}

/*
 * Magellan encoder — implements the Magellan alarm/sensor on-air protocol.
 *
 * Magellan protocol uses a unique framing (all timings in µs):
 *   Header:    800µs HIGH + 200µs LOW, then 12× (200µs HIGH + 200µs LOW),
 *              then 200µs HIGH + 400µs LOW
 *   Start bit: 1200µs HIGH + 400µs LOW
 *   Data bits: 32 bits, MSB first:
 *     Bit 1: 200µs HIGH + 400µs LOW  (te_short + te_long) — INVERTED vs standard OOK
 *     Bit 0: 400µs HIGH + 200µs LOW  (te_long  + te_short) — INVERTED vs standard OOK
 *   Stop bit:  200µs HIGH + 40000µs LOW  (te_short + te_long×100)
 *
 * te_short=200, te_long=400
 * Protocol is fixed 32-bit; bit_count must equal 32.
 * Total: 1 + 12 + 1 + 1 + 32 + 1 = 48 pairs per repetition
 */
static uint32_t subghz_key_encode_magellan(const SubGhzKeyParams *params,
                                            SubGhzRawPair *out,
                                            uint32_t max_pairs,
                                            uint8_t repetitions)
{
    /* Magellan protocol is fixed 32-bit; reject any other bit count */
    if (params->bit_count != 32)
        return 0;

    const uint32_t TE_SHORT = 200;
    const uint32_t TE_LONG  = 400;

    uint32_t total_pairs = (uint32_t)SUBGHZ_MAGELLAN_PAIRS_PER_REP * repetitions;
    if (total_pairs > max_pairs)
        return 0;

    uint32_t idx = 0;

    for (uint8_t rep = 0; rep < repetitions; rep++)
    {
        /* Header burst: 800µs HIGH (4×te_short) + 200µs LOW */
        out[idx].high_us = TE_SHORT * 4;
        out[idx].low_us  = TE_SHORT;
        idx++;

        /* Header toggle: 12× (200µs HIGH + 200µs LOW) */
        for (uint8_t i = 0; i < 12; i++)
        {
            out[idx].high_us = TE_SHORT;
            out[idx].low_us  = TE_SHORT;
            idx++;
        }

        /* Header end: 200µs HIGH + 400µs LOW */
        out[idx].high_us = TE_SHORT;
        out[idx].low_us  = TE_LONG;
        idx++;

        /* Start bit: 1200µs HIGH (3×te_long) + 400µs LOW */
        out[idx].high_us = TE_LONG * 3;
        out[idx].low_us  = TE_LONG;
        idx++;

        /* Data bits: MSB first, INVERTED polarity (bit_count validated == 32 above) */
        uint64_t mask = 1ULL << (params->bit_count - 1);
        for (uint32_t b = 0; b < params->bit_count; b++)
        {
            if (params->key_value & mask)
            {
                /* Bit 1: SHORT HIGH + LONG LOW (inverted vs standard OOK PWM) */
                out[idx].high_us = TE_SHORT;
                out[idx].low_us  = TE_LONG;
            }
            else
            {
                /* Bit 0: LONG HIGH + SHORT LOW (inverted vs standard OOK PWM) */
                out[idx].high_us = TE_LONG;
                out[idx].low_us  = TE_SHORT;
            }
            mask >>= 1;
            idx++;
        }

        /* Stop bit: 200µs HIGH + 40000µs LOW (te_long × 100) */
        out[idx].high_us = TE_SHORT;
        out[idx].low_us  = TE_LONG * 100;
        idx++;
    }

    return idx;
}

uint32_t subghz_key_encode_custom(const SubGhzKeyParams *params,
                                   SubGhzRawPair *out,
                                   uint32_t max_pairs,
                                   uint8_t repetitions)
{
    if (!params || !out || max_pairs == 0 || repetitions == 0 || params->protocol[0] == '\0')
        return 0;

    if (subghz_ascii_strcasecmp(params->protocol, "Magellan") == 0)
        return subghz_key_encode_magellan(params, out, max_pairs, repetitions);

    return 0;  /* No custom encoder for this protocol */
}

uint32_t subghz_key_custom_required_pairs(const SubGhzKeyParams *params,
                                           uint8_t repetitions)
{
    if (!params || params->protocol[0] == '\0' || repetitions == 0)
        return 0;

    if (subghz_ascii_strcasecmp(params->protocol, "Magellan") == 0)
    {
        if (params->bit_count != 32)
            return 0;  /* Invalid Magellan bit count */
        return (uint32_t)SUBGHZ_MAGELLAN_PAIRS_PER_REP * repetitions;
    }

    return 0;  /* No custom encoder for this protocol */
}
