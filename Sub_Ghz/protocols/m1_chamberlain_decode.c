/* See COPYING.txt for license details. */

/*
 * m1_chamberlain_decode.c
 *
 * Chamberlain / LiftMaster Security+ 1.0 decoder.
 *
 * Security+ 1.0 uses a ternary (3-symbol) OOK encoding at 315 MHz or 390 MHz.
 * te_short = 500 us.  Each symbol is a (LOW, HIGH) pulse pair:
 *   Symbol 0  →  LOW 1500 us  +  HIGH  500 us   (3:1)
 *   Symbol 1  →  LOW 1000 us  +  HIGH 1000 us   (2:2)
 *   Symbol 2  →  LOW  500 us  +  HIGH 1500 us   (1:3)
 *
 * One complete message = two sub-packets, each carrying 1 header + 20 data symbols.
 *   Sub-packet 1 header = Symbol 0, followed by 20 data symbols.
 *   Sub-packet 2 header = Symbol 2, followed by 20 data symbols.
 *
 * The full rolling code + fixed code are reconstructed from both sub-packets
 * using the argilo/secplus algorithm (MIT licence), as implemented in Flipper
 * Zero firmware (GPLv3):
 *   https://github.com/flipperdevices/flipperzero-firmware/blob/dev/lib/subghz/protocols/secplus_v1.c
 *
 * M1 pulse capture model
 * ─────────────────────
 * pulse_times[] stores alternating raw durations (high and low) captured by the
 * radio ISR.  A duration ≥ INTERPACKET_GAP_MIN (1500 us) triggers packet decode.
 *
 * Sub-packet 1: the 1500 us header-LOW is consumed as the triggering gap from the
 * PREVIOUS packet; capture starts with the 500 us header-HIGH.
 *   pulse_times[0]           = 500 us (header-HIGH, confirms sub-pkt 1)
 *   pulse_times[1..40]       = 20 data symbol pairs (LOW, HIGH each)
 *   pulse_times[41]          = 59500 us gap  →  npulsecount == 42
 *
 * Sub-packet 2: the 1500 us header-HIGH is itself consumed as a triggering gap
 * (1500 us == INTERPACKET_GAP_MIN).  Capture starts with the first data-symbol LOW.
 *   pulse_times[0..39]       = 20 data symbol pairs (LOW, HIGH each)
 *   pulse_times[40]          = 58000 us gap  →  npulsecount == 41
 */

#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"
#include "../../m1_csrc/bit_util.h"

#define M1_LOGDB_TAG    "SUBGHZ_SEC+1"

/* ── Symbol classification ──────────────────────────────────────────────── */
#define TE_SHORT    500u   /* us */
#define TE_MID      1000u  /* us */
#define TE_LONG     1500u  /* us */
#define TE_TOL      200u   /* us tolerance */

#define SYM_ERR     (-1)
#define SYM_0         0    /* LOW 1500, HIGH  500 */
#define SYM_1         1    /* LOW 1000, HIGH 1000 */
#define SYM_2         2    /* LOW  500, HIGH 1500 */

#define PKT1_HEADER   SYM_0
#define PKT2_HEADER   SYM_2
#define SYMS_PER_PKT  20

#define PKT1_ACCEPTED (1u << 0)
#define PKT2_ACCEPTED (1u << 1)

/* ── Static accumulator (persists across two sub-packet decode calls) ───── */
static int8_t  s_sym[2][SYMS_PER_PKT];
static uint8_t s_pkt_mask;

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static inline int8_t classify_sym(uint16_t t_low, uint16_t t_high)
{
    if (get_diff(t_low, TE_LONG)  < TE_TOL && get_diff(t_high, TE_SHORT) < TE_TOL)
        return SYM_0;
    if (get_diff(t_low, TE_MID)   < TE_TOL && get_diff(t_high, TE_MID)   < TE_TOL)
        return SYM_1;
    if (get_diff(t_low, TE_SHORT) < TE_TOL && get_diff(t_high, TE_LONG)  < TE_TOL)
        return SYM_2;
    return SYM_ERR;
}

/*
 * Decode 20 symbols starting at pulse_times[offset].
 * Each symbol occupies two consecutive entries: [offset+2k], [offset+2k+1]
 * = (LOW, HIGH) for that symbol.
 */
static bool decode_symbols(uint8_t pkt_idx, uint8_t offset, uint16_t pulsecount)
{
    for (uint8_t k = 0; k < SYMS_PER_PKT; k++) {
        uint8_t lo_i = offset + (uint8_t)(k * 2);
        uint8_t hi_i = lo_i + 1;
        if (hi_i >= pulsecount) {
            s_pkt_mask = 0;
            return false;
        }
        int8_t sym = classify_sym(subghz_decenc_ctl.pulse_times[lo_i],
                                  subghz_decenc_ctl.pulse_times[hi_i]);
        if (sym == SYM_ERR) {
            s_pkt_mask = 0;
            return false;
        }
        s_sym[pkt_idx][k] = sym;
    }
    return true;
}

/*
 * Reconstruct rolling code and fixed code from the two accumulated symbol
 * halves.  Algorithm from argilo/secplus (MIT) / Flipper secplus_v1.c (GPLv3).
 *
 * data_array layout (Flipper convention):
 *   [0]      = PKT1_HEADER (0)
 *   [1..20]  = 20 data symbols from sub-packet 1 (s_sym[0][0..19])
 *   [21]     = PKT2_HEADER (2)
 *   [22..41] = 20 data symbols from sub-packet 2 (s_sym[1][0..19])
 *
 * Decode loop (from Flipper secplus_v1.c, GPLv3):
 *   Packet 1: iterate odd indices 1,3,5,…,19
 *     rolling digit = data_array[i]
 *     fixed  digit  = (60 + data_array[i+1] - acc) % 3
 *   Packet 2: iterate even indices 22,24,26,…,40
 *     rolling digit = data_array[i]
 *     fixed  digit  = (60 + data_array[i+1] - acc) % 3
 *
 * Result: generic.data = (uint64_t)fixed << 32 | rolling
 *         (after bit-reversal of rolling, matching Flipper output)
 */
static void reconstruct_payload(uint16_t p)
{
    /* Build data_array in Flipper's index convention */
    int8_t da[42];
    da[0]  = PKT1_HEADER;
    da[21] = PKT2_HEADER;
    for (uint8_t i = 0; i < SYMS_PER_PKT; i++) {
        da[1  + i] = s_sym[0][i];
        da[22 + i] = s_sym[1][i];
    }

    uint32_t rolling = 0, fixed = 0, acc = 0;
    uint8_t  digit;

    /* Packet 1: pairs at odd+even indices 1..20 */
    for (uint8_t i = 1; i < 21; i += 2) {
        digit   = (uint8_t)da[i];
        rolling = rolling * 3u + digit;
        acc    += digit;

        digit  = (uint8_t)((60u + (uint8_t)da[i + 1] - acc) % 3u);
        fixed  = fixed * 3u + digit;
        acc   += digit;
    }

    acc = 0;
    /* Packet 2: pairs at even+odd indices 22..41 */
    for (uint8_t i = 22; i < 42; i += 2) {
        digit   = (uint8_t)da[i];
        rolling = rolling * 3u + digit;
        acc    += digit;

        digit  = (uint8_t)((60u + (uint8_t)da[i + 1] - acc) % 3u);
        fixed  = fixed * 3u + digit;
        acc   += digit;
    }

    rolling = reverse32(rolling);

    M1_LOG_I(M1_LOGDB_TAG, "Sec+1.0 rolling=0x%08lX fixed=0x%08lX\r\n",
             (unsigned long)rolling, (unsigned long)fixed);

    subghz_decenc_ctl.n64_decodedvalue  = ((uint64_t)fixed << 32) | rolling;
    subghz_decenc_ctl.n32_rollingcode   = rolling;
    subghz_decenc_ctl.n32_serialnumber  = fixed;
    subghz_decenc_ctl.ndecodedbitlength = 40;
    subghz_decenc_ctl.ndecodeddelay     = 0;
    subghz_decenc_ctl.ndecodedprotocol  = p;
}

/* ── Public entry point ─────────────────────────────────────────────────── */

uint8_t subghz_decode_chamberlain(uint16_t p, uint16_t pulsecount)
{
    /*
     * Distinguish sub-packet 1 (npulsecount == 42) from sub-packet 2
     * (npulsecount == 41) based on pulse count.
     *
     * Edge case: sub-packet 2 header-HIGH is exactly 1500 us.  When captured
     * (rather than consumed as a gap) pulsecount will be 43; the header pair
     * occupies [0,1] and data starts at [2].
     */
    uint8_t pkt_idx;
    uint8_t data_offset;

    if (pulsecount == 42) {
        /* Sub-packet 1: verify header HIGH ≈ te_short (500 us) */
        if (get_diff(subghz_decenc_ctl.pulse_times[0], TE_SHORT) >= TE_TOL) {
            s_pkt_mask = 0;
            return 1;
        }
        pkt_idx     = 0;
        data_offset = 1; /* LOW of sym[0] is at index 1 */
    } else if (pulsecount == 41) {
        /* Sub-packet 2: header was consumed as gap, data starts at [0] */
        pkt_idx     = 1;
        data_offset = 0;
    } else if (pulsecount == 43) {
        /* Sub-packet 2 edge case: header pair at [0,1], data at [2] */
        /* Verify header: LOW ≈ te_short, HIGH ≈ te_long */
        if (get_diff(subghz_decenc_ctl.pulse_times[0], TE_SHORT) >= TE_TOL ||
            get_diff(subghz_decenc_ctl.pulse_times[1], TE_LONG)  >= TE_TOL) {
            s_pkt_mask = 0;
            return 1;
        }
        pkt_idx     = 1;
        data_offset = 2;
    } else {
        return 1; /* wrong pulse count for this protocol */
    }

    if (!decode_symbols(pkt_idx, data_offset, pulsecount)) {
        M1_LOG_D(M1_LOGDB_TAG, "sym err pkt%u\r\n", pkt_idx);
        return 1;
    }

    s_pkt_mask |= (pkt_idx == 0) ? PKT1_ACCEPTED : PKT2_ACCEPTED;

    if (s_pkt_mask != (PKT1_ACCEPTED | PKT2_ACCEPTED))
        return 1; /* waiting for the other sub-packet */

    s_pkt_mask = 0; /* reset for next reception */
    reconstruct_payload(p);
    return 0; /* success */
}
