/* See COPYING.txt for license details. */

/*
 * subghz_autosave.c
 *
 * Pure-logic Sub-GHz autosave helpers — filename generation and duplicate
 * detection for the autosave feature.
 *
 * Extracted for host-side testability.
 * Hardware-independent: no FatFS, no RTOS, no display.
 *
 * M1 Project
 */

#include "subghz_autosave.h"
#include <stdio.h>
#include <string.h>

/*──────────── Filename generation ────────────*/

int subghz_autosave_make_path(char *out, size_t out_size,
                              const char *protocol, uint64_t key,
                              uint32_t tick, bool use_sgh)
{
    if (out == NULL || out_size == 0 || protocol == NULL)
        return 0;

    const char *ext = use_sgh ? ".sgh" : ".sub";
    int n = snprintf(out, out_size,
                     SUBGHZ_AUTOSAVE_DIR_PATH "/%s_%lX_%lu%s",
                     protocol,
                     (unsigned long)(uint32_t)key,
                     (unsigned long)tick,
                     ext);
    if (n < 0 || (size_t)n >= out_size)
        return 0;
    return n;
}

/*──────────── Duplicate detection ────────────*/

#define AUTOSAVE_DUP_RING_SIZE  32
#define AUTOSAVE_DUP_PROTO_LEN  24   /* stored protocol name for collision guard */

typedef struct {
    uint32_t protocol_hash;
    uint64_t key;
    char     protocol_name[AUTOSAVE_DUP_PROTO_LEN];
} autosave_dup_entry_t;

static autosave_dup_entry_t s_dup_ring[AUTOSAVE_DUP_RING_SIZE];
static uint8_t s_dup_count;
static uint8_t s_dup_head;

/* Simple djb2 hash for protocol name */
static uint32_t hash_str(const char *s)
{
    uint32_t h = 5381;
    while (*s)
        h = h * 33 + (uint8_t)*s++;
    return h;
}

bool subghz_autosave_is_duplicate(const char *protocol, uint64_t key)
{
    if (protocol == NULL)
        return true;  /* invalid — treat as duplicate to prevent save */

    uint32_t ph = hash_str(protocol);

    /* Check existing entries — secondary strcmp guards against hash collisions */
    for (uint8_t i = 0; i < s_dup_count; i++)
    {
        if (s_dup_ring[i].protocol_hash == ph && s_dup_ring[i].key == key
            && strncmp(s_dup_ring[i].protocol_name, protocol, AUTOSAVE_DUP_PROTO_LEN - 1) == 0)
            return true;
    }

    return false;
}

void subghz_autosave_record(const char *protocol, uint64_t key)
{
    if (protocol == NULL)
        return;

    uint32_t ph = hash_str(protocol);

    s_dup_ring[s_dup_head].protocol_hash = ph;
    s_dup_ring[s_dup_head].key = key;
    strncpy(s_dup_ring[s_dup_head].protocol_name, protocol, AUTOSAVE_DUP_PROTO_LEN - 1);
    s_dup_ring[s_dup_head].protocol_name[AUTOSAVE_DUP_PROTO_LEN - 1] = '\0';
    s_dup_head = (s_dup_head + 1) % AUTOSAVE_DUP_RING_SIZE;
    if (s_dup_count < AUTOSAVE_DUP_RING_SIZE)
        s_dup_count++;
}

void subghz_autosave_dup_reset(void)
{
    s_dup_count = 0;
    s_dup_head = 0;
    memset(s_dup_ring, 0, sizeof(s_dup_ring));
}
