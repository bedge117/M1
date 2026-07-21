/* See COPYING.txt for license details. */

/**
 * @file   subghz_button_override.c
 * @brief  Sub-GHz button-override key mutation — pure logic.
 *
 * See subghz_button_override.h for the public API and design rationale.
 *
 * This module is intentionally narrow.  Adding a new protocol here
 * requires verifying that:
 *
 *   1. The protocol's button index is encoded as a contiguous
 *      plain-bit field in the Flipper SubGhz Key value (NOT inside an
 *      encrypted payload).
 *   2. The M1 encoder path that emits the OOK PWM waveform (whether
 *      that is keeloq_encode_replay() or the generic
 *      subghz_key_encode()) will pick up the new button bits and emit
 *      a valid signal that a real receiver will decode as the new
 *      button.
 *
 * The KeeLoq family satisfies (1) and (2) because:
 *   - keeloq_encode_replay() runs extract_fields() on the supplied
 *     key_value, decrypts the hop with the manufacturer key, then
 *     re-encrypts the hop and reconstructs the key with the button
 *     bits taken from extract_fields() — i.e. it honours whatever
 *     button bits are in the supplied key value.
 *   - Star Line's button bits are in the FIX portion which is
 *     transmitted verbatim (no cipher operates on those bits).
 */

#include "subghz_button_override.h"
#include <stddef.h>
#include <ctype.h>

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

/**
 * @brief  Case-insensitive comparison of @p input against @p canonical,
 *         tolerating leading/trailing whitespace in @p input.
 *
 * @retval 0  Match.
 * @retval non-zero  No match.
 */
static int icase_eq_trimmed(const char *input, const char *canonical)
{
    if (input == NULL || canonical == NULL) return 1;

    while (*input != '\0' && isspace((unsigned char)*input)) ++input;

    while (*input != '\0' && *canonical != '\0')
    {
        int a = tolower((unsigned char)*input);
        int b = tolower((unsigned char)*canonical);
        if (a != b) return 1;
        ++input;
        ++canonical;
    }
    if (*canonical != '\0') return 1;
    while (*input != '\0')
    {
        if (!isspace((unsigned char)*input)) return 1;
        ++input;
    }
    return 0;
}

/*============================================================================*/
/* Protocol classification                                                    */
/*============================================================================*/

typedef enum {
    BTN_OVR_PROTO_UNSUPPORTED = 0,
    BTN_OVR_PROTO_KEELOQ,    /* KeeLoq / Jarolift — button at bits [63:60] */
    BTN_OVR_PROTO_STARLINE,  /* Star Line         — button at bits [ 3: 0] */
} btn_ovr_proto_t;

static btn_ovr_proto_t classify_protocol(const char *protocol)
{
    if (protocol == NULL) return BTN_OVR_PROTO_UNSUPPORTED;

    /* Star Line first — it has its own button layout. */
    if (icase_eq_trimmed(protocol, "Star Line") == 0)
        return BTN_OVR_PROTO_STARLINE;

    /* KeeLoq and Jarolift share the FIX-upper-nibble layout. */
    if (icase_eq_trimmed(protocol, "KeeLoq") == 0)   return BTN_OVR_PROTO_KEELOQ;
    if (icase_eq_trimmed(protocol, "Jarolift") == 0) return BTN_OVR_PROTO_KEELOQ;

    return BTN_OVR_PROTO_UNSUPPORTED;
}

/*============================================================================*/
/* Public API                                                                 */
/*============================================================================*/

bool subghz_button_override_supports(const char *protocol)
{
    return classify_protocol(protocol) != BTN_OVR_PROTO_UNSUPPORTED;
}

bool subghz_button_override_apply(const char *protocol,
                                  uint64_t    key_in,
                                  uint8_t     button_index,
                                  uint64_t   *key_out)
{
    if (key_out == NULL) return false;

    /* Default: caller-friendly fallback so a single assignment path
     * works in callers regardless of return value. */
    *key_out = key_in;

    const uint64_t btn = (uint64_t)(button_index & 0x0FU);

    switch (classify_protocol(protocol))
    {
        case BTN_OVR_PROTO_KEELOQ:
        {
            /* Clear bits [63:60] then OR in the new button nibble. */
            uint64_t cleared = key_in & ~((uint64_t)0x0FU << 60);
            *key_out = cleared | (btn << 60);
            return true;
        }

        case BTN_OVR_PROTO_STARLINE:
        {
            /* Clear bits [3:0] then OR in the new button nibble. */
            uint64_t cleared = key_in & ~((uint64_t)0x0FU);
            *key_out = cleared | btn;
            return true;
        }

        case BTN_OVR_PROTO_UNSUPPORTED:
        default:
            return false;
    }
}
