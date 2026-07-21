/* See COPYING.txt for license details. */

/*
 * subghz_new_remote_gen.c
 *
 * Pure-logic random remote key generator — zero hardware dependencies.
 * See subghz_new_remote_gen.h for the public API.
 *
 * M1 Project — Hapax fork
 */

#include <stdio.h>
#include <string.h>
#include "subghz_new_remote_gen.h"

/*============================================================================*/
/* Static protocol table                                                      */
/*============================================================================*/

typedef struct {
    const char *label;       /* Display label shown in the wizard proto picker */
    const char *proto_name;  /* Exact protocol name string for .sub file */
    uint32_t    freq_hz;     /* Centre frequency */
    uint32_t    bit_count;   /* Key bit width (must be ≤ 64) */
    uint16_t    te;          /* te_short override in µs (0 = use registry default) */
    const char *file_prefix; /* Filename prefix segment (no spaces) */
} ProtoSpec;

static const ProtoSpec proto_specs[BW_PROTO_COUNT] = {
    [BW_PROTO_CAME_ATOMO] = {
        .label       = "CAME Atomo 433",
        .proto_name  = "CAME Atomo",
        .freq_hz     = 433920000,
        .bit_count   = 62,
        .te          = 200,
        .file_prefix = "CameAtomo",
    },
    [BW_PROTO_NICE_FLOR_S] = {
        .label       = "Nice FloR-S 433",
        .proto_name  = "Nice FloR-S",
        .freq_hz     = 433920000,
        .bit_count   = 52,
        .te          = 500,
        .file_prefix = "NiceFlors",
    },
    [BW_PROTO_ALUTECH_AT4N] = {
        .label       = "Alutech AT-4N 433",
        .proto_name  = "Alutech AT-4N",
        .freq_hz     = 433920000,
        /* Registry min_count_bit = 72, but uint64_t caps at 64.
         * The M1 decoder also stores at most 64 bits; 64-bit keys
         * transmit all recognisable data within that limit. */
        .bit_count   = 64,
        .te          = 400,
        .file_prefix = "Alutech",
    },
    [BW_PROTO_DITEC_GOL4] = {
        .label       = "DITEC GOL4 433",
        .proto_name  = "DITEC_GOL4",
        .freq_hz     = 433920000,
        .bit_count   = 54,
        .te          = 400,
        .file_prefix = "DitecGol4",
    },
    [BW_PROTO_KINGGATES_STYLO4K] = {
        .label       = "KingGates Stylo4k 433",
        .proto_name  = "KingGates Stylo4k",
        .freq_hz     = 433920000,
        .bit_count   = 60,
        .te          = 400,
        .file_prefix = "KingGates",
    },
};

/*============================================================================*/
/* splitmix64 PRNG (public-domain, Sebastiano Vigna)                         */
/*============================================================================*/

static uint64_t splitmix64(uint64_t *state)
{
    uint64_t z = (*state += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

/*============================================================================*/
/* Public API                                                                 */
/*============================================================================*/

const char *subghz_new_remote_proto_label(BindWizardProto proto)
{
    if ((unsigned)proto >= BW_PROTO_COUNT)
        return "Unknown";
    return proto_specs[proto].label;
}

bool subghz_new_remote_gen(BindWizardProto proto, uint64_t seed, NewRemoteParams *out)
{
    if ((unsigned)proto >= BW_PROTO_COUNT || out == NULL)
        return false;

    const ProtoSpec *s = &proto_specs[proto];

    /* Generate a random 64-bit value and mask it to bit_count bits. */
    uint64_t state = seed;
    uint64_t raw   = splitmix64(&state);

    /* key occupies the lower bit_count bits (LSB-aligned, MSB of those
     * bits is bit number bit_count-1).  The key encoder reads from
     * mask = 1ULL << (bit_count - 1) downwards. */
    uint64_t mask = (s->bit_count >= 64) ? UINT64_MAX
                                         : ((1ULL << s->bit_count) - 1ULL);
    uint64_t key = raw & mask;

    /* Fill output */
    strncpy(out->proto_name, s->proto_name, sizeof(out->proto_name) - 1);
    out->proto_name[sizeof(out->proto_name) - 1] = '\0';
    out->freq_hz   = s->freq_hz;
    out->key       = key;
    out->bit_count = s->bit_count;
    out->te        = s->te;

    /* Build filename base: "NewRemote_<prefix>_<lower12hex>"
     * Using 12 hex digits (48 bits) significantly reduces collision
     * probability compared to 8 digits (32 bits / birthday ~77k keys). */
    snprintf(out->file_base, sizeof(out->file_base),
             "NewRemote_%s_%012llx", s->file_prefix,
             (unsigned long long)(key & 0xFFFFFFFFFFFFULL));

    return true;
}
