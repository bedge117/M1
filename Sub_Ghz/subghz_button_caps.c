/* See COPYING.txt for license details. */

/**
 * @file   subghz_button_caps.c
 * @brief  Sub-GHz protocol button-cycling capability lookup — pure logic.
 *
 * See subghz_button_caps.h for the full design rationale.
 *
 * Coverage rules for adding a new protocol to the table below:
 *   1. Protocol MUST have `SubGhzProtocolFlag_PwmKeyReplay` set (or be
 *      a KeeLoq-family counter-mode protocol).  Anything that cannot
 *      be replayed OTA must not appear here.
 *   2. The encoder MUST be able to synthesise per-button output for
 *      any button index 0..N-1 without requiring an additional captured
 *      sample per button.  KeeLoq satisfies this because the cipher and
 *      counter are public; static OOK protocols do not, because they
 *      transmit the captured payload verbatim.
 *   3. The button index range MUST be a contiguous 0..N-1 with no
 *      "reserved" codes.  Protocols with sparse / multi-byte button
 *      fields (e.g. Faac SLH's 4-byte command word) are intentionally
 *      excluded from cycling until per-protocol handlers exist.
 */

#include "subghz_button_caps.h"
#include <stddef.h>
#include <ctype.h>

/**
 * @brief  Per-protocol button capability entry.
 *
 * `name` is compared case-insensitively against the trimmed input.
 */
typedef struct {
    const char *name;
    uint8_t     button_count;
} button_caps_entry_t;

/* Hand-curated capability table — see header for inclusion criteria. */
static const button_caps_entry_t s_table[] = {
    /* KeeLoq family — counter-mode encoder, 2-bit button field.
     * The classic NLFSR cipher exposes a 4-button cycle in the
     * encrypted hop word; M1's keeloq_encoder supports overriding
     * `btn` on the manufacturer-key path. */
    { "KeeLoq",         4 },
    { "Jarolift",       4 },
    { "Star Line",      4 },

    /* Nice FloR-S — 4-bit button code (16 values).  PwmKeyReplay
     * encoder can mutate the `btn` nibble in the OOK PWM key bitstream
     * without recapture. */
    { "Nice FloR-S",   16 },

    /* CAME family — 2-bit button code in the OOK PWM payload.
     * Same key/serial for all 4 buttons. */
    { "CAME Atomo",     4 },
    { "CAME TWEE",      4 },

    /* Alutech AT-4N — 2-bit button code, OOK PWM. */
    { "Alutech AT-4N",  4 },

    /* KingGates Stylo4k — 2-bit button code, OOK PWM. */
    { "KingGates Stylo4k", 4 },

    /* DITEC_GOL4 — 2-bit button code, OOK PWM. */
    { "DITEC_GOL4",     4 },

    /* Scher-Khan (Magicar / Logicar variants share the same name
     * in the registry).  4 buttons. */
    { "Scher-Khan",     4 },

    /* Toyota — 4 buttons (lock, unlock, trunk, panic). */
    { "Toyota",         4 },
};

#define BUTTON_CAPS_TABLE_SIZE \
    (sizeof(s_table) / sizeof(s_table[0]))

/*----------------------------------------------------------------------------*/
/* Helpers                                                                    */
/*----------------------------------------------------------------------------*/

/**
 * @brief  Case-insensitive comparison that tolerates leading/trailing
 *         whitespace in `input` (but not in `canonical`).  Returns 0
 *         on match, non-zero otherwise.
 *
 * Pure ASCII fold via tolower(); we never see non-ASCII protocol names
 * in the registry, so the locale-dependence of tolower() is moot.
 */
static int icase_eq_trimmed(const char *input, const char *canonical)
{
    if (input == NULL || canonical == NULL) return 1;

    /* Skip leading whitespace in input. */
    while (*input != '\0' && isspace((unsigned char)*input)) ++input;

    /* Walk both strings in lockstep. */
    while (*input != '\0' && *canonical != '\0')
    {
        int a = tolower((unsigned char)*input);
        int b = tolower((unsigned char)*canonical);
        if (a != b) return 1;
        ++input;
        ++canonical;
    }

    /* canonical must be fully consumed; input may have trailing ws. */
    if (*canonical != '\0') return 1;
    while (*input != '\0')
    {
        if (!isspace((unsigned char)*input)) return 1;
        ++input;
    }
    return 0;
}

/*----------------------------------------------------------------------------*/
/* Public API                                                                 */
/*----------------------------------------------------------------------------*/

subghz_button_caps_t subghz_button_caps_for_protocol(
    const char *protocol_name)
{
    subghz_button_caps_t caps = { /*.supports_cycling=*/false,
                                   /*.button_count=*/0u };

    if (protocol_name == NULL) return caps;

    for (size_t i = 0; i < BUTTON_CAPS_TABLE_SIZE; ++i)
    {
        if (icase_eq_trimmed(protocol_name, s_table[i].name) == 0)
        {
            caps.supports_cycling = (s_table[i].button_count >= 2u);
            caps.button_count     = s_table[i].button_count;
            return caps;
        }
    }
    return caps;
}
