/* See COPYING.txt for license details. */

/**
 * @file   subghz_submenu_model.c
 * @brief  Reusable scrollable-list model — pure logic.
 *
 * See subghz_submenu_model.h for the contract and invariants.
 */

#include "subghz_submenu_model.h"
#include <stddef.h>

/*----------------------------------------------------------------------------*/
/* Internal helpers                                                            */
/*----------------------------------------------------------------------------*/

/**
 * Re-anchor `scroll_offset` so `selected` is visible and the window does
 * not run past the end of the list.  Assumes `visible_count >= 1`.
 */
static void anchor_window(subghz_submenu_model_t *m)
{
    if (m->item_count == 0) {
        m->selected      = 0;
        m->scroll_offset = 0;
        return;
    }

    if (m->selected >= m->item_count) {
        m->selected = (uint8_t)(m->item_count - 1u);
    }

    /* Selection above the window: scroll up to it. */
    if (m->selected < m->scroll_offset) {
        m->scroll_offset = m->selected;
    }

    /* Selection below the window: scroll down so it lands on the bottom row. */
    if (m->selected >= (uint8_t)(m->scroll_offset + m->visible_count)) {
        m->scroll_offset =
            (uint8_t)(m->selected - m->visible_count + 1u);
    }

    /* Do not allow the window to extend past the end of the list when the
     * list is long enough to fill the window — keeps the bottom of the list
     * aligned with the bottom of the visible area. */
    if (m->item_count >= m->visible_count) {
        uint8_t max_offset = (uint8_t)(m->item_count - m->visible_count);
        if (m->scroll_offset > max_offset) {
            m->scroll_offset = max_offset;
        }
    } else {
        /* Shorter than the window — always start at top. */
        m->scroll_offset = 0;
    }
}

static uint8_t clamp_visible(uint8_t v)
{
    return (v == 0u) ? 1u : v;
}

/*----------------------------------------------------------------------------*/
/* Public API                                                                  */
/*----------------------------------------------------------------------------*/

void subghz_submenu_model_init(subghz_submenu_model_t *m,
                               uint8_t item_count,
                               uint8_t visible_count)
{
    if (m == NULL) return;
    m->item_count    = item_count;
    m->visible_count = clamp_visible(visible_count);
    m->selected      = 0;
    m->scroll_offset = 0;
    anchor_window(m);
}

void subghz_submenu_model_set_item_count(subghz_submenu_model_t *m,
                                         uint8_t item_count)
{
    if (m == NULL) return;
    m->item_count = item_count;
    if (item_count == 0) {
        m->selected      = 0;
        m->scroll_offset = 0;
        return;
    }
    if (m->selected >= item_count) {
        m->selected = (uint8_t)(item_count - 1u);
    }
    anchor_window(m);
}

void subghz_submenu_model_set_visible_count(subghz_submenu_model_t *m,
                                            uint8_t visible_count)
{
    if (m == NULL) return;
    m->visible_count = clamp_visible(visible_count);
    anchor_window(m);
}

void subghz_submenu_model_set_selected(subghz_submenu_model_t *m,
                                       uint8_t index)
{
    if (m == NULL) return;
    if (m->item_count == 0) {
        m->selected      = 0;
        m->scroll_offset = 0;
        return;
    }
    m->selected = (index >= m->item_count)
                  ? (uint8_t)(m->item_count - 1u)
                  : index;
    anchor_window(m);
}

void subghz_submenu_model_up(subghz_submenu_model_t *m)
{
    if (m == NULL || m->item_count == 0) return;
    if (m->selected == 0) {
        /* Wrap to last. */
        m->selected = (uint8_t)(m->item_count - 1u);
    } else {
        m->selected = (uint8_t)(m->selected - 1u);
    }
    anchor_window(m);
}

void subghz_submenu_model_down(subghz_submenu_model_t *m)
{
    if (m == NULL || m->item_count == 0) return;
    if ((uint8_t)(m->selected + 1u) >= m->item_count) {
        /* Wrap to first. */
        m->selected      = 0;
        m->scroll_offset = 0;
    } else {
        m->selected = (uint8_t)(m->selected + 1u);
    }
    anchor_window(m);
}

bool subghz_submenu_model_is_visible(const subghz_submenu_model_t *m,
                                     uint8_t idx)
{
    if (m == NULL || m->item_count == 0) return false;
    if (idx >= m->item_count) return false;
    if (idx < m->scroll_offset) return false;
    if (idx >= (uint8_t)(m->scroll_offset + m->visible_count)) return false;
    return true;
}

uint8_t subghz_submenu_model_visible_row(const subghz_submenu_model_t *m)
{
    if (m == NULL || m->item_count == 0) return 0;
    if (m->selected < m->scroll_offset) return 0;
    return (uint8_t)(m->selected - m->scroll_offset);
}
