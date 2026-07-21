/* See COPYING.txt for license details. */

/*
 * subghz_signal_format.h
 *
 * Polymorphic Info-screen renderers (Phase 11-1).
 *
 * Each function defined here matches the `SubGhzGetStringFn` signature
 * declared in `subghz_protocol_registry.h` and is installed onto a
 * specific protocol's registry entry via the `.get_string` slot.
 *
 * These are pure-logic helpers — they take a `SubGhzSignalView` and a
 * caller-owned buffer, and write a NUL-terminated multi-line string
 * (rows separated by `\n`).  No hardware access, no heap allocation,
 * no file I/O.  Host-testable.
 *
 * The Saved scene's "Signal Info" draw helper consults
 * `proto->get_string` and, when non-NULL, invokes the registered
 * renderer for the file's protocol.  When NULL the scene falls back
 * to the existing generic "Proto / Key / Bits / TE" layout.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_SIGNAL_FORMAT_H
#define SUBGHZ_SIGNAL_FORMAT_H

#include <stddef.h>
#include "subghz_protocol_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * KeeLoq-family Info renderer.
 *
 * Formats the loaded signal as a multi-line block of the form:
 *
 *   Proto: KeeLoq
 *   Serial: 0xAABBCCDD
 *   Button: 0xX
 *   EncHop: 0xWWWWWWWW
 *
 * The Serial / Button / EncHop fields are extracted via the Phase 9a-1
 * `subghz_signal_fields_keeloq_extract()` helper, so the layout matches
 * the values shown in the SignalSettings scene exactly.
 *
 * Contract:
 *   - If @p view is NULL or @p buflen is zero, the function is a no-op
 *     except that, when @p buf is non-NULL and @p buflen >= 1, the
 *     first byte of @p buf is set to NUL.
 *   - If @p view->protocol is NULL or is not a recognised KeeLoq-family
 *     name, the renderer falls back to a generic "Proto: <unknown>"
 *     header followed by the raw key — this guards against a registry
 *     mis-wiring (e.g. the get_string slot being installed on a
 *     non-KeeLoq entry).
 *   - On normal output the buffer always ends with NUL (no trailing
 *     newline); truncation is silent.
 *
 * @param[in]  view    Loaded-signal view (may be NULL).
 * @param[out] buf     Caller-owned output buffer.
 * @param[in]  buflen  Size of @p buf in bytes (including NUL terminator).
 */
void subghz_signal_format_keeloq_info(const SubGhzSignalView *view,
                                       char                   *buf,
                                       size_t                  buflen);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_SIGNAL_FORMAT_H */
