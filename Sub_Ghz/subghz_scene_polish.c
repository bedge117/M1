/* See COPYING.txt for license details. */

/**
 * @file   subghz_scene_polish.c
 * @brief  Sub-GHz scene-manager polish primitives — pure logic.
 *
 * See subghz_scene_polish.h for the design rationale.
 */

#include "subghz_scene_polish.h"

int subghz_scene_stack_find(const uint8_t *stack,
                             uint8_t depth,
                             uint8_t target)
{
    if (stack == NULL || depth == 0) return -1;

    /* Top-down scan: index (depth-1) is the current scene; index 0 is
     * the root.  We want the topmost match so a freshly-pushed scene
     * always wins over an older duplicate further down the stack. */
    for (int i = (int)depth - 1; i >= 0; --i)
    {
        if (stack[i] == target) return i;
    }
    return -1;
}

uint8_t subghz_scene_stack_pop_to_depth(const uint8_t *stack,
                                         uint8_t depth,
                                         uint8_t target)
{
    int idx = subghz_scene_stack_find(stack, depth, target);
    if (idx < 0) return depth;            /* not found — leave stack alone */

    /* idx is 0-based; new depth places target at top → depth = idx+1. */
    return (uint8_t)(idx + 1);
}

bool subghz_scene_tick_due(uint32_t now_ms,
                            uint32_t last_tick_ms,
                            uint32_t period_ms)
{
    if (period_ms == 0) return false;     /* feature disabled */

    /* Wrap-safe elapsed computation: the subtraction in uint32_t wraps
     * cleanly so a 49.7-day rollover does not freeze the scheduler. */
    uint32_t elapsed = now_ms - last_tick_ms;
    return elapsed >= period_ms;
}
