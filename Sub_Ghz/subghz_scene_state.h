/* See COPYING.txt for license details. */

/**
 * @file   subghz_scene_state.h
 * @brief  Per-scene 32-bit state slot — pure logic, host-testable.
 *
 * Modelled on Flipper / Momentum's `scene_manager_set_scene_state()` /
 * `scene_manager_get_scene_state()`.  Each scene ID owns a single uint32_t
 * slot that persists across push/pop of that scene, giving scenes a clean
 * place to stash UI state (cursor position, sub-mode, last-selected target)
 * without resorting to file-scope statics.
 *
 * The semantic contract intentionally mirrors Flipper's:
 *   - All slots are zero-initialised by `init()`.
 *   - `set()` on an out-of-range id is a silent no-op (matches Flipper).
 *   - `get()` on an out-of-range id returns 0 (matches Flipper).
 *
 * This module is hardware-independent — it does NOT touch SubGhzApp,
 * RTOS, display, or any HAL.  The Sub-GHz scene layer embeds an instance
 * inside SubGhzApp and exposes thin wrappers.
 */

#ifndef SUBGHZ_SCENE_STATE_H_
#define SUBGHZ_SCENE_STATE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Maximum number of scene-state slots supported by this module.
 *
 * Sized comfortably above SubGhzSceneCount (currently 19) to leave room
 * for the upcoming Transmitter / SavedMenu / Delete / MoreRAW / DecodeRAW /
 * SignalSettings / SetType / SetKey / SetSerial / SetButton / SetCounter
 * scenes added in later phases.  Bumping this is cheap — 4 bytes per slot.
 */
#ifndef SUBGHZ_SCENE_STATE_SLOTS
#define SUBGHZ_SCENE_STATE_SLOTS 48
#endif

/**
 * @brief  Per-scene state storage.  Embed in your application context.
 *
 * Zero-initialised storage is also a valid initial state — `init()` is
 * provided for explicit clarity and for clearing on reset.
 */
typedef struct {
    uint32_t slot[SUBGHZ_SCENE_STATE_SLOTS];
} subghz_scene_state_array_t;

/**
 * @brief  Clear every slot to zero.  Safe to call with NULL (no-op).
 */
void subghz_scene_state_init(subghz_scene_state_array_t *arr);

/**
 * @brief  Read the state slot for a scene id.
 * @return The stored value, or 0 if @p arr is NULL or @p scene_id is
 *         out of range.
 */
uint32_t subghz_scene_state_get(const subghz_scene_state_array_t *arr,
                                 uint8_t scene_id);

/**
 * @brief  Write the state slot for a scene id.
 *
 * Out-of-range @p scene_id is a silent no-op (matches Flipper semantics —
 * callers should not have to guard every set with a range check).
 *
 * @retval true   The value was stored
 * @retval false  Rejected (NULL arr or out-of-range scene_id)
 */
bool subghz_scene_state_set(subghz_scene_state_array_t *arr,
                             uint8_t scene_id,
                             uint32_t value);

/**
 * @brief  Compile-time bound: capacity of the array.
 */
static inline uint8_t subghz_scene_state_capacity(void)
{
    return (uint8_t)SUBGHZ_SCENE_STATE_SLOTS;
}

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_SCENE_STATE_H_ */
