/* See COPYING.txt for license details. */

/**
 * @file   subghz_rssi_history.h
 * @brief  Pure-logic RSSI spectrogram history for the Sub-GHz Read Raw scene.
 *
 *  Extracted from m1_sub_ghz.c so host unit tests can exercise the push/reset
 *  logic without pulling in the full firmware context (u8g2, FreeRTOS, etc.).
 *
 *  Convention: this header has no dependencies beyond <stdbool.h>, <stdint.h>
 *  and <string.h> so it can be included in both firmware and test builds.
 *  Do NOT add anything here that references hardware, FreeRTOS, or u8g2.
 *
 *  Bug fixed here (v0.9.1.48): the original static subghz_raw_rssi_push() in
 *  m1_sub_ghz.c wrote u_rssi=0 to the last committed history slot whenever
 *  RSSI dropped below SUBGHZ_RSSI_THRESHOLD_MIN while trace=false.  This
 *  erased a committed bar every time the signal briefly exceeded the capture
 *  threshold and then fell back to the noise floor — producing an empty
 *  waveform despite an active recording (non-zero SPL counter).  The fix:
 *  only update the last committed slot when u_rssi > 0.
 */

#ifndef SUBGHZ_RSSI_HISTORY_H_
#define SUBGHZ_RSSI_HISTORY_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/** Number of RSSI history entries (= waveform pixel columns). */
#define SUBGHZ_RSSI_HISTORY_SIZE   100

/** Minimum RSSI for display mapping. Values below this yield a bar height of
 *  zero; when trace=false, u==0 must NOT overwrite the last committed slot
 *  (otherwise previously committed bars would be erased). */
#define SUBGHZ_RSSI_THRESHOLD_MIN  (-90.0f)

/** Scale factor: dBm above THRESHOLD_MIN → pixel height.
 *  Lower value → taller bars (denser fill).  Changed from 2.7 → 1.8 in
 *  v0.9.1.42 for Momentum parity. */
#define SUBGHZ_RSSI_DIVIDER        1.8f

/**
 * @brief  RSSI spectrogram history state.
 *
 *  Mutation should go through the inline helpers below. Firmware may read buf/head/end
 *  directly when rendering; test code allocates one instance on the stack.
 */
typedef struct {
    /** Circular history buffer (+2 guard bytes). */
    uint8_t  buf[SUBGHZ_RSSI_HISTORY_SIZE + 2];
    /** Live RSSI → pixel height (for cursor indicator, updated on every push). */
    uint8_t  current;
    /** Next-write position in buf[].  Bars are committed at head, then head++. */
    uint8_t  head;
    /** True once buf has wrapped around (head has rolled over from 99 → 0). */
    bool     end;
} SubghzRssiHistory;

/* -------------------------------------------------------------------------
 * Pure-logic helpers — no hardware dependencies
 * ---------------------------------------------------------------------- */

/**
 * @brief  Convert an RSSI dBm value to a pixel bar height.
 *
 * @param  rssi_dbm  RSSI in dBm (typically −120 … 0).
 * @retval  Pixel height (not clamped). Returns 0 when rssi_dbm is below
 *          SUBGHZ_RSSI_THRESHOLD_MIN (−90 dBm); at 0 dBm this yields ~50.
 */
static inline uint8_t subghz_rssi_to_u8(float rssi_dbm)
{
    if (rssi_dbm < SUBGHZ_RSSI_THRESHOLD_MIN)
        return 0;
    return (uint8_t)((rssi_dbm - SUBGHZ_RSSI_THRESHOLD_MIN) / SUBGHZ_RSSI_DIVIDER);
}

/**
 * @brief  Reset the RSSI history to the fresh-start state.
 *
 *  Called by start_raw_rx() at the beginning of each recording session and by
 *  scene_on_enter() on first entry.
 *
 * @param  h  Pointer to the history instance to reset.
 */
static inline void subghz_rssi_history_reset(SubghzRssiHistory *h)
{
    memset(h->buf, 0, sizeof(h->buf));
    h->current = 0;
    h->head    = 0;
    h->end     = false;
}

/**
 * @brief  Push one RSSI sample into the history.
 *
 *  This is the core spectrogram logic — extracted and bug-fixed.
 *
 *  @param  h        Pointer to the history instance.
 *  @param  rssi_dbm RSSI reading in dBm.
 *  @param  trace    When true, commit a new bar and advance the cursor.
 *                   When false, the cursor stays in place; only update the
 *                   last committed bar if u_rssi > 0 (never erase it).
 *
 *  Bug fixed: original code wrote u_rssi=0 to the last committed slot on
 *  trace=false whenever RSSI < THRESHOLD_MIN.  This erased previously
 *  captured bars when the signal briefly dropped to the noise floor, leaving
 *  an empty waveform despite an active recording (issue: "Read Raw graph").
 */
/**
 * @brief  Rate-limit guard for RSSI cursor advances.
 *
 *  Returns true when at least @p interval_ms milliseconds have elapsed since
 *  @p last_ms.  All parameters are ms timestamps / intervals from
 *  HAL_GetTick() or any monotonic uint32_t ms source.  The subtraction is
 *  unsigned so 32-bit roll-over is handled correctly (correct as long as
 *  interval_ms < 2^31 ms ≈ 24 days).
 *
 *  This is a pure function — no hardware dependencies — so it can be called
 *  from draw() in the scene and exercised in host unit tests.
 *
 * @param  now_ms       Current timestamp in ms (from HAL_GetTick()).
 * @param  last_ms      Timestamp of the most-recent allowed cursor advance.
 * @param  interval_ms  Minimum inter-advance interval in ms (use 100U to match
 *                      Momentum's 100 ms tick rate).
 * @retval true   when (now_ms − last_ms) >= interval_ms.
 * @retval false  otherwise (within the rate-limit window).
 */
static inline bool subghz_rssi_rate_allow(uint32_t now_ms, uint32_t last_ms,
                                           uint32_t interval_ms)
{
    return (now_ms - last_ms) >= interval_ms;
}
/**
 * @brief  Push one RSSI sample into the history.
 */
static inline void subghz_rssi_history_push(SubghzRssiHistory *h,
                                             float rssi_dbm, bool trace)
{
    uint8_t u = subghz_rssi_to_u8(rssi_dbm);
    h->current = u;

    if (trace)
    {
        h->buf[h->head] = u;
        h->head++;
        if (h->head >= SUBGHZ_RSSI_HISTORY_SIZE)
        {
            h->end  = true;
            h->head = 0;
        }
    }
    else
    {
        /* Only update the last committed slot when u > 0.
         *
         * If RSSI dropped below THRESHOLD_MIN (u=0), leave the slot as-is.
         * Writing 0 would erase the bar that was committed on the previous
         * trace=true push — the root cause of the "empty waveform" bug. */
        if (u > 0)
        {
            uint8_t slot = (h->head > 0)
                         ? (h->head - 1)
                         : (h->end ? (SUBGHZ_RSSI_HISTORY_SIZE - 1) : 0);
            h->buf[slot] = u;
        }
    }
}

#endif /* SUBGHZ_RSSI_HISTORY_H_ */
