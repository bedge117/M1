/* See COPYING.txt for license details. */

/**
 * @file   subghz_hex_editor.c
 * @brief  Pure-logic hex-digit editor — see header for full description.
 *
 * M1 Project — Hapax fork
 */

#include "subghz_hex_editor.h"

#include <string.h>

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

static uint8_t clamp_bit_count(uint8_t bits)
{
    if (bits == 0U)
        return 1U;
    if (bits > 64U)
        return 64U;
    return bits;
}

static uint8_t digits_for_bits(uint8_t bits)
{
    /* Round up to nearest hex digit: (bits + 3) / 4 */
    uint8_t d = (uint8_t)((bits + 3U) / 4U);
    if (d == 0U)
        d = 1U;
    if (d > SUBGHZ_HEX_EDITOR_MAX_DIGITS)
        d = SUBGHZ_HEX_EDITOR_MAX_DIGITS;
    return d;
}

static uint64_t mask_for_bits(uint8_t bits)
{
    if (bits >= 64U)
        return (uint64_t)~0ULL;
    return ((uint64_t)1ULL << bits) - 1ULL;
}

/*============================================================================*/
/* Public API                                                                 */
/*============================================================================*/

void subghz_hex_editor_init(subghz_hex_editor_t *ed, uint8_t bit_count)
{
    if (!ed)
        return;

    memset(ed, 0, sizeof(*ed));
    ed->bit_count   = clamp_bit_count(bit_count);
    ed->digit_count = digits_for_bits(ed->bit_count);
    ed->cursor      = 0U;
}

void subghz_hex_editor_set_value(subghz_hex_editor_t *ed, uint64_t value)
{
    if (!ed || ed->digit_count == 0U)
        return;

    value &= mask_for_bits(ed->bit_count);

    /* Big-endian: digits[0] = most-significant nibble.  The
     * most-significant nibble may carry only the high bit_count % 4
     * bits when bit_count is not a multiple of 4 — the masking above
     * already enforces that constraint on `value` itself, so the
     * resulting top nibble naturally fits in the protocol's bit width. */
    for (int i = (int)ed->digit_count - 1; i >= 0; --i)
    {
        ed->digits[i] = (uint8_t)(value & 0x0FU);
        value >>= 4;
    }
}

uint64_t subghz_hex_editor_value(const subghz_hex_editor_t *ed)
{
    if (!ed || ed->digit_count == 0U)
        return 0ULL;

    uint64_t v = 0ULL;
    for (uint8_t i = 0U; i < ed->digit_count; ++i)
    {
        v = (v << 4) | (uint64_t)(ed->digits[i] & 0x0FU);
    }
    return v & mask_for_bits(ed->bit_count);
}

void subghz_hex_editor_up(subghz_hex_editor_t *ed)
{
    if (!ed || ed->cursor >= ed->digit_count)
        return;
    ed->digits[ed->cursor] = (uint8_t)((ed->digits[ed->cursor] + 1U) & 0x0FU);
}

void subghz_hex_editor_down(subghz_hex_editor_t *ed)
{
    if (!ed || ed->cursor >= ed->digit_count)
        return;
    ed->digits[ed->cursor] = (uint8_t)((ed->digits[ed->cursor] - 1U) & 0x0FU);
}

void subghz_hex_editor_left(subghz_hex_editor_t *ed)
{
    if (!ed)
        return;
    if (ed->cursor > 0U)
        ed->cursor--;
}

void subghz_hex_editor_right(subghz_hex_editor_t *ed)
{
    if (!ed || ed->digit_count == 0U)
        return;
    if (ed->cursor + 1U < ed->digit_count)
        ed->cursor++;
}
