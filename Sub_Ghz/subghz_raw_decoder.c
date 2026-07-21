/* See COPYING.txt for license details. */

/*
 * subghz_raw_decoder.c
 *
 * Offline RAW→protocol decode engine.
 * Extracted from m1_subghz_scene_saved.c::do_decode_raw().
 *
 * This module is hardware-independent: all hardware interaction is
 * abstracted behind the SubGhzRawDecodeTryFn callback.
 *
 * M1 Project — Hapax fork
 */

#include <stdlib.h>
#include <string.h>
#include "subghz_raw_decoder.h"

/*============================================================================*/
/* Internal helpers                                                           */
/*============================================================================*/

/**
 * Check if a result is a duplicate of an already-recorded result.
 * Deduplication key: protocol index + key value.
 * Returns the index of the duplicate, or -1 if not found.
 */
static int find_duplicate(const SubGhzRawDecodeResult *results,
                           uint8_t count,
                           const SubGhzRawDecodeResult *candidate)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (results[i].protocol == candidate->protocol &&
            results[i].key == candidate->key)
            return (int)i;
    }
    return -1;
}

/**
 * Try to decode a packet and store the result if unique.
 *
 * @return true if a new (non-duplicate) result was added.
 */
static bool try_packet(const uint16_t *pulse_buf,
                        uint16_t pulse_count,
                        uint32_t frequency,
                        SubGhzRawDecodeResult *results,
                        uint8_t *decode_count,
                        uint8_t max_results,
                        SubGhzRawDecodeTryFn try_decode,
                        void *user_ctx)
{
    if (*decode_count >= max_results)
        return false;

    SubGhzRawDecodeResult result;
    memset(&result, 0, sizeof(result));

    if (!try_decode(pulse_buf, pulse_count, &result, user_ctx))
        return false;

    /* Require non-zero key (filters out false-positive decodes) */
    if (result.key == 0)
        return false;

    result.frequency = frequency;
    result.rssi = 0;   /* RSSI is meaningless for offline decode */

    int dup_idx = find_duplicate(results, *decode_count, &result);
    if (dup_idx >= 0)
    {
        /* Same protocol+key seen again — bump confidence counter.
         * Real remotes repeat their transmission several times, so a
         * high repeat_count distinguishes genuine matches from timing
         * coincidences that only fire once. */
        if (results[dup_idx].repeat_count < 255)
            results[dup_idx].repeat_count++;
        return false;
    }

    result.repeat_count = 1;
    results[(*decode_count)++] = result;
    return true;
}

/*============================================================================*/
/* Public API                                                                 */
/*============================================================================*/

uint8_t subghz_decode_raw_offline(const int16_t *raw_data,
                                   uint16_t raw_count,
                                   uint32_t frequency,
                                   SubGhzRawDecodeResult *results,
                                   uint8_t max_results,
                                   SubGhzRawDecodeTryFn try_decode,
                                   void *user_ctx)
{
    if (!raw_data || raw_count == 0 || !results || max_results == 0 || !try_decode)
        return 0;

    uint16_t pulse_buf[PACKET_PULSE_COUNT_MAX];
    uint16_t pulse_count = 0;
    uint8_t decode_count = 0;

    for (uint16_t i = 0; i < raw_count; i++)
    {
        uint16_t dur = (uint16_t)abs((int)raw_data[i]);

        if (dur < PACKET_PULSE_TIME_MIN)
        {
            /* Below minimum pulse — noise, reset accumulator */
            pulse_count = 0;
            continue;
        }

        if (dur >= INTERPACKET_GAP_MIN)
        {
            /* Inter-packet gap — end the current packet */
            if (pulse_count < PACKET_PULSE_COUNT_MAX)
                pulse_buf[pulse_count++] = dur;

            if (pulse_count >= PACKET_PULSE_COUNT_MIN)
            {
                try_packet(pulse_buf, pulse_count, frequency,
                           results, &decode_count, max_results,
                           try_decode, user_ctx);
            }
            pulse_count = 0;
        }
        else
        {
            /* Normal pulse — accumulate */
            if (pulse_count < PACKET_PULSE_COUNT_MAX)
            {
                pulse_buf[pulse_count++] = dur;
            }
            else
            {
                pulse_count = 0;   /* overflow, reset */
            }
        }
    }

    /* Try to decode any remaining pulses (file may end without a final gap) */
    if (pulse_count >= PACKET_PULSE_COUNT_MIN && decode_count < max_results)
    {
        try_packet(pulse_buf, pulse_count, frequency,
                   results, &decode_count, max_results,
                   try_decode, user_ctx);
    }

    return decode_count;
}
