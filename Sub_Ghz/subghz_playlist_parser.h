/* See COPYING.txt for license details. */

/*
 * subghz_playlist_parser.h
 *
 * Path remapping and delay-parsing utility for Sub-GHz playlist files.
 * Extracted from m1_subghz_scene_playlist.c.
 *
 * Converts Flipper-style paths (e.g. "/ext/subghz/foo.sub") to M1
 * convention (e.g. "/SUBGHZ/foo.sub") so playlist files from
 * UberGuidoZ/Flipper collections work without manual editing.
 *
 * Also exposes subghz_playlist_parse_delay() to extract a delay value
 * from lines of the form "# delay: <ms>" (used by UberGuidoZ playlists to
 * set the inter-signal gap).
 *
 * Both modules are hardware-independent and compile on both ARM and host.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_PLAYLIST_PARSER_H
#define SUBGHZ_PLAYLIST_PARSER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Remap a Flipper-style path to M1 convention.
 *
 * Converts "/ext/subghz/..." → "/SUBGHZ/..." so that playlist files
 * from UberGuidoZ/Flipper work without manual editing.
 *
 * If the path does not match the Flipper convention, it is copied
 * unchanged.
 *
 * @param src   Source path (trimmed, null-terminated)
 * @param dst   Destination buffer
 * @param dlen  Size of destination buffer (must be > 0)
 */
void subghz_remap_flipper_path(const char *src, char *dst, size_t dlen);

/**
 * Try to parse a "# delay: <ms>" comment line.
 *
 * Matches lines of the form (leading/trailing whitespace ignored after '#'):
 *   # delay: 500
 *   # delay:1000
 *   # Delay: 250
 *
 * @param line      Null-terminated line (CR/LF already stripped).
 * @param delay_ms  Output: delay in milliseconds (clamped to 0–60000).
 * @return true if the line is a delay directive and @p delay_ms was set,
 *         false otherwise.
 */
bool subghz_playlist_parse_delay(const char *line, uint16_t *delay_ms);

#endif /* SUBGHZ_PLAYLIST_PARSER_H */
