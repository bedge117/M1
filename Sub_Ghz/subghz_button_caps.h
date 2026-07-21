/* See COPYING.txt for license details. */

/**
 * @file   subghz_button_caps.h
 * @brief  Sub-GHz protocol button-cycling capability lookup — pure logic.
 *
 * Phase 4a of the Momentum parity migration.  Given a protocol name
 * (the same string used in `subghz_protocol_registry.c::name`), returns
 * whether the protocol supports on-device button cycling and, if so,
 * how many distinct button codes it exposes.
 *
 *     scene on_enter  → subghz_button_caps_for_protocol(name)
 *                        ├─ supports_cycling : true iff M1 can render a
 *                        │                     different OOK PWM key for
 *                        │                     each value of `btn` 0..N-1
 *                        └─ button_count     : N (1..16); 0 = unsupported
 *
 * The result is consumed by the Transmitter scene (Phase 4b) to populate
 * @ref subghz_transmitter_ctl_t.allow_button_cycle and `.button_count`.
 *
 * This module is hardware-independent: no SI4463, no DMA, no RTOS, no
 * FAT FS.  All lookups are pure (const table) so host tests can pin
 * protocol coverage and the capability decision in CI.
 *
 * Design rationale
 * ----------------
 *  - The mapping is intentionally a hand-curated table — protocol-level
 *    button semantics are not encoded in `subghz_protocol_registry.c`,
 *    and adding the field there would touch every protocol entry.
 *  - Only protocols whose key encoder/replay can synthesise per-button
 *    output **on the M1** are listed.  Protocols that cannot be replayed
 *    OTA (FAAC SLH, Somfy Telis, Security+ 2.0, KIA Seed, etc.) return
 *    `{false, 0}` regardless of how many physical buttons their real
 *    remotes have.
 *  - Static OOK protocols (Princeton, CAME, Nice FLO, Holtek, etc.) have
 *    `data` directly transmitted with no per-button derivation, so they
 *    are intentionally excluded from cycling — a remote with multiple
 *    buttons is represented by multiple saved files, not by cycling.
 *
 * Coverage (rolling-code OOK PWM, Phase 4a):
 *
 *   | Protocol            | Button count | Encoder family               |
 *   |---------------------|--------------|------------------------------|
 *   | KeeLoq              | 4            | counter-mode (NLFSR cipher)  |
 *   | Jarolift            | 4            | KeeLoq-based                 |
 *   | Star Line           | 4            | KeeLoq-based                 |
 *   | Nice FloR-S         | 16           | OOK PWM (4-bit btn field)    |
 *   | CAME Atomo          | 4            | OOK PWM                      |
 *   | CAME TWEE           | 4            | OOK PWM                      |
 *   | Alutech AT-4N       | 4            | OOK PWM                      |
 *   | KingGates Stylo4k   | 4            | OOK PWM                      |
 *   | DITEC_GOL4          | 4            | OOK PWM                      |
 *   | Scher-Khan          | 4            | OOK PWM (Magicar/Logicar)    |
 *   | Toyota              | 4            | OOK PWM                      |
 *
 * All other protocol names return `{false, 0}`.  Name lookup is
 * case-insensitive and tolerant of leading/trailing whitespace, but
 * must otherwise match the canonical registry spelling.
 */

#ifndef SUBGHZ_BUTTON_CAPS_H_
#define SUBGHZ_BUTTON_CAPS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Button-cycling capability for a single protocol.
 */
typedef struct {
    /** True iff the protocol supports M1-replayable on-device button
     *  cycling.  False for static OOK, AES-encrypted rolling, or any
     *  protocol whose per-button key cannot be synthesised without
     *  a captured sample per button. */
    bool    supports_cycling;

    /** Number of distinct button codes when cycling is supported.
     *  Always >= 2 when `supports_cycling == true`; 0 otherwise. */
    uint8_t button_count;
} subghz_button_caps_t;

/**
 * @brief  Look up button-cycling capability for a protocol name.
 *
 * Pure, no allocation, NULL-safe.  Matching is case-insensitive and
 * tolerates leading/trailing whitespace.  Unknown / NULL / empty
 * names return `{false, 0}`.
 *
 * The returned struct is a value type — safe to copy and store.
 *
 * @param protocol_name  Protocol name as it appears in
 *                       @ref subghz_protocol_registry_t.name.  May be
 *                       NULL or empty (returns `{false, 0}`).
 * @return  Capability descriptor; see @ref subghz_button_caps_t.
 */
subghz_button_caps_t subghz_button_caps_for_protocol(
    const char *protocol_name);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_BUTTON_CAPS_H_ */
