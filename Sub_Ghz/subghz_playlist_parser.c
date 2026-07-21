/* See COPYING.txt for license details. */

/*
 * subghz_playlist_parser.c
 *
 * Path remapping and delay-parsing utility for Sub-GHz playlist files.
 * Extracted from m1_subghz_scene_playlist.c.
 *
 * M1 Project — Hapax fork
 */

#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "subghz_playlist_parser.h"

void subghz_remap_flipper_path(const char *src, char *dst, size_t dlen)
{
    if (!src || !dst || dlen == 0)
        return;

    /* Strip Flipper's /ext/ prefix: /ext/subghz/... → /subghz/... */
    if (strncmp(src, "/ext/", 5) == 0)
        src += 4;  /* skip "/ext" — keep the leading "/" */

    /* Uppercase the directory: /subghz/ → /SUBGHZ/ */
    if (strncmp(src, "/subghz/", 8) == 0)
    {
        snprintf(dst, dlen, "/SUBGHZ/%s", src + 8);
    }
    else
    {
        strncpy(dst, src, dlen - 1);
        dst[dlen - 1] = '\0';
    }
}

bool subghz_playlist_parse_delay(const char *line, uint16_t *delay_ms)
{
    if (!line || !delay_ms)
        return false;

    /* Must start with '#' */
    if (line[0] != '#')
        return false;

    /* Skip '#' and optional whitespace */
    const char *p = line + 1;
    while (*p == ' ' || *p == '\t')
        p++;

    /* Case-insensitive match for "delay:" */
    if (strncasecmp(p, "delay:", 6) != 0)
        return false;

    p += 6;

    /* Skip whitespace between "delay:" and the number */
    while (*p == ' ' || *p == '\t')
        p++;

    if (!isdigit((unsigned char)*p))
        return false;

    char *end;
    long val = strtol(p, &end, 10);
    if (end == p)
        return false;

    /* Clamp to 0–60000 ms */
    if (val < 0)   val = 0;
    if (val > 60000) val = 60000;

    *delay_ms = (uint16_t)val;
    return true;
}
