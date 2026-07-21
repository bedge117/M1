/* See COPYING.txt for license details. */

/*
 * subghz_signal_format.c
 *
 * Polymorphic Info-screen renderers — see subghz_signal_format.h for the
 * full contract.  Pure-logic; host-testable.
 *
 * M1 Project — Hapax fork
 */

#include "subghz_signal_format.h"
#include "subghz_signal_fields.h"

#include <stdio.h>
#include <string.h>

/*============================================================================*/
/* Small append helper — writes a NUL-terminated chunk into a fixed buffer    */
/* with explicit length tracking so we never overrun and always NUL-terminate */
/*============================================================================*/

static void buf_append(char *buf, size_t buflen, size_t *used,
                       const char *src)
{
    if (!buf || buflen == 0 || !src || !used) return;
    if (*used >= buflen - 1) return;   /* buffer already full (NUL slot reserved) */

    size_t avail = buflen - 1 - *used;
    size_t len   = strlen(src);
    if (len > avail) len = avail;

    memcpy(buf + *used, src, len);
    *used += len;
    buf[*used] = '\0';
}

/*============================================================================*/
/* KeeLoq-family renderer                                                     */
/*============================================================================*/

void subghz_signal_format_keeloq_info(const SubGhzSignalView *view,
                                       char                   *buf,
                                       size_t                  buflen)
{
    /* Always NUL-terminate when we can — protects against callers that
     * forget to zero the buffer. */
    if (buf && buflen > 0)
        buf[0] = '\0';

    if (!view || !buf || buflen < 2)
        return;

    size_t used = 0;
    char   tmp[40];

    /* Header line: "Proto: <name>" — use a safe fallback when NULL. */
    const char *proto = view->protocol ? view->protocol : "?";
    snprintf(tmp, sizeof(tmp), "Proto: %s\n", proto);
    buf_append(buf, buflen, &used, tmp);

    /* Field decomposition — only succeeds for recognised KeeLoq-family names.
     * On failure the extractor zero-initialises fields, so we fall back to
     * a raw key dump rather than producing a misleading "Serial: 0x0". */
    subghz_keeloq_fields_t fields = {0};
    bool ok = subghz_signal_fields_keeloq_extract(view->protocol,
                                                   view->key,
                                                   &fields);
    if (!ok)
    {
        snprintf(tmp, sizeof(tmp), "Key: 0x%08lX%08lX\n",
                 (unsigned long)((view->key >> 32) & 0xFFFFFFFFUL),
                 (unsigned long)( view->key        & 0xFFFFFFFFUL));
        buf_append(buf, buflen, &used, tmp);
        return;
    }

    snprintf(tmp, sizeof(tmp), "Serial: 0x%07lX\n",
             (unsigned long)(fields.serial & 0x0FFFFFFFUL));
    buf_append(buf, buflen, &used, tmp);

    snprintf(tmp, sizeof(tmp), "Button: 0x%X\n",
             (unsigned)(fields.button & 0x0F));
    buf_append(buf, buflen, &used, tmp);

    snprintf(tmp, sizeof(tmp), "EncHop: 0x%08lX",
             (unsigned long)fields.enc_hop);
    buf_append(buf, buflen, &used, tmp);
}
