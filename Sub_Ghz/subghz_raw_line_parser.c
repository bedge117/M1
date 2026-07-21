/* See COPYING.txt for license details. */

/*
 * subghz_raw_line_parser.c
 *
 * RAW_Data line parser — extracted from sub_ghz_replay_flipper_file()
 * in m1_sub_ghz.c (lines 1675-1722 and 1534-1597).
 *
 * Parses Flipper's signed raw timing values into absolute-value samples,
 * handling cross-buffer number splits when f_gets truncates a long line.
 *
 * This module is hardware-independent.
 *
 * M1 Project — Hapax fork
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "subghz_raw_line_parser.h"

/*============================================================================*/
/* subghz_raw_line_state_init                                                  */
/*============================================================================*/

void subghz_raw_line_state_init(SubGhzRawLineState *state)
{
    if (state)
        state->leftover[0] = '\0';
}

/*============================================================================*/
/* subghz_parse_raw_data_line                                                  */
/*============================================================================*/

uint16_t subghz_parse_raw_data_line(const char *text,
                                     bool line_complete,
                                     SubGhzRawLineState *state,
                                     uint32_t *samples_out,
                                     uint16_t max_samples)
{
    if (!text || !state || !samples_out || max_samples == 0)
        return 0;

    const char *p = text;
    uint16_t count = 0;

    /* If we have leftover digits from the previous buffer boundary,
     * prepend them to the first token of this buffer. */
    if (state->leftover[0] != '\0')
    {
        /* Find end of first numeric token (digits, minus sign) */
        const char *tok_end = p;
        while (*tok_end && *tok_end != ' ' && *tok_end != '\r' && *tok_end != '\n')
            tok_end++;

        /* Build combined number: leftover + start of this buffer */
        char combined[32];
        snprintf(combined, sizeof(combined), "%s%.*s",
                 state->leftover, (int)(tok_end - p), p);
        state->leftover[0] = '\0';

        long val = strtol(combined, NULL, 10);
        if (val < 0) val = -val;
        if (val != 0 && count < max_samples)
            samples_out[count++] = (uint32_t)val;

        p = tok_end;
    }

    /* Parse remaining numbers */
    while (*p)
    {
        /* Skip whitespace */
        while (*p == ' ') p++;
        if (*p == '\0') break;

        char *endp;
        long val = strtol(p, &endp, 10);
        if (endp == p) break;   /* no more numbers */
        p = endp;

        if (val < 0) val = -val;
        if (val == 0) continue; /* skip zero */

        /* If we're at end of buffer and line is truncated,
         * this number might be incomplete — save as leftover */
        if (!line_complete && *p == '\0')
        {
            snprintf(state->leftover, sizeof(state->leftover),
                     "%lu", (unsigned long)val);
            break;
        }

        if (count < max_samples)
            samples_out[count++] = (uint32_t)val;
    }

    return count;
}
