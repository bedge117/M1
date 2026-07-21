/* See COPYING.txt for license details. */

/**
 * @file   subghz_scene_polish.h
 * @brief  Sub-GHz scene-manager polish primitives — pure logic, host-testable.
 *
 * Phase 10 of the Momentum parity migration.  Provides three small,
 * hardware-independent helpers that back the scene-manager API additions
 * required by the upcoming Transmitter scene (Phase 3b):
 *
 *   1. `subghz_scene_stack_find()` — backs `search_and_pop_to(scene)`.
 *      Given the current scene-stack array, returns the depth at which
 *      `target` was last pushed, so the caller can pop scenes one by
 *      one (calling their `on_exit` handlers) until that scene is on
 *      top.  Used by deep flows like Save → SaveName → SaveSuccess
 *      that need to return to a known parent in one step.
 *
 *   2. `subghz_scene_tick_due()` — backs the periodic-tick event hook.
 *      Given `now_ms`, `last_tick_ms`, and the configured `period_ms`,
 *      returns true if enough wall-clock time has elapsed to fire the
 *      next `SubGhzEventTick`.  Uses uint32_t wraparound-safe arithmetic
 *      so it behaves correctly across `HAL_GetTick()`'s 49.7-day rollover.
 *      Disabled when `period_ms == 0`.
 *
 *   3. `subghz_scene_custom_payload_t` — typedef for the 32-bit custom-
 *      event payload word.  This is the type behind
 *      `subghz_scene_send_custom_event(app, payload)` and
 *      `subghz_scene_custom_payload(app)` in the scene-manager API.
 *      Defined here for visibility from both the scene manager and the
 *      pure tests.
 *
 * This module is hardware-independent — it does NOT touch SubGhzApp,
 * RTOS, display, or any HAL.  All scene IDs are passed as raw `uint8_t`
 * so the module can be compiled and tested on the host without
 * pulling in `m1_subghz_scene.h` (which transitively includes the HAL).
 */

#ifndef SUBGHZ_SCENE_POLISH_H_
#define SUBGHZ_SCENE_POLISH_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  32-bit payload word carried alongside a `SubGhzEventCustom`.
 *
 * The semantic interpretation of the payload is scene-specific.
 * Conventional split:
 *   - bits 31..16 — opcode / sub-event id
 *   - bits 15..0  — opcode-specific value (counter, button index, etc.)
 *
 * Scenes that do not care about the split simply treat it as one
 * opaque 32-bit value.
 */
typedef uint32_t subghz_scene_custom_payload_t;

/**
 * @brief  Locate the topmost occurrence of @p target in a scene stack.
 *
 * Searches @p stack[0..depth-1] from the top down and returns the index
 * of the first match found.  Returning the topmost match (rather than
 * the bottom one) lets `search_and_pop_to` correctly handle stacks where
 * the same scene appears more than once — popping to the *deepest* copy
 * would skip past intermediate state the user expects to return to.
 *
 * @param  stack  Pointer to scene-stack array, treated as `uint8_t[]`.
 *                NULL-safe.
 * @param  depth  Current stack depth.  Zero is permitted (returns -1).
 * @param  target Scene id to search for, cast to `uint8_t`.
 *
 * @return  Index (0..depth-1) of the topmost matching slot, or -1 if
 *          @p target is not present.
 */
int subghz_scene_stack_find(const uint8_t *stack,
                             uint8_t depth,
                             uint8_t target);

/**
 * @brief  Compute the new stack depth that would result from popping
 *         scenes one at a time until @p target is on top.
 *
 * Pure-logic counterpart to `search_and_pop_to`.  Does NOT call any
 * `on_exit` handlers — the dispatcher does that as it pops each
 * level.  Use this to validate whether a search target is reachable
 * before initiating the pop loop.
 *
 * @return  New depth (1..depth) if @p target was found, leaving it on
 *          top of the stack; @p depth unchanged if not found.
 */
uint8_t subghz_scene_stack_pop_to_depth(const uint8_t *stack,
                                         uint8_t depth,
                                         uint8_t target);

/**
 * @brief  Decide whether a tick event is due.
 *
 * Returns true when at least @p period_ms milliseconds have elapsed
 * between @p last_tick_ms and @p now_ms.  Implementation uses
 * `(uint32_t)(now - last) >= period` so the comparison is safe across
 * HAL tick wraparound at 2^32 ms (~49.7 days).
 *
 * Edge cases:
 *   - @p period_ms == 0 — feature disabled, always returns false.
 *   - @p now_ms == @p last_tick_ms — zero elapsed; returns false unless
 *     @p period_ms == 0 (which is already false).
 *
 * @note Callers are expected to update their `last_tick_ms` snapshot to
 *       @p now_ms *only* when this function returns true, to keep the
 *       cadence stable across long iteration intervals.
 */
bool subghz_scene_tick_due(uint32_t now_ms,
                            uint32_t last_tick_ms,
                            uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_SCENE_POLISH_H_ */
