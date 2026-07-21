/* See COPYING.txt for license details. */

/**
 * @file   subghz_scene_state.c
 * @brief  Per-scene 32-bit state slot — pure logic, host-testable.
 */

#include "subghz_scene_state.h"
#include <stddef.h>
#include <string.h>

void subghz_scene_state_init(subghz_scene_state_array_t *arr)
{
    if (arr == NULL) return;
    memset(arr->slot, 0, sizeof(arr->slot));
}

uint32_t subghz_scene_state_get(const subghz_scene_state_array_t *arr,
                                 uint8_t scene_id)
{
    if (arr == NULL) return 0;
    if (scene_id >= SUBGHZ_SCENE_STATE_SLOTS) return 0;
    return arr->slot[scene_id];
}

bool subghz_scene_state_set(subghz_scene_state_array_t *arr,
                             uint8_t scene_id,
                             uint32_t value)
{
    if (arr == NULL) return false;
    if (scene_id >= SUBGHZ_SCENE_STATE_SLOTS) return false;
    arr->slot[scene_id] = value;
    return true;
}
