/* See COPYING.txt for license details. */

/*
 * subghz_raw_decoder.h
 *
 * Offline RAW→protocol decode engine extracted from
 * m1_subghz_scene_saved.c::do_decode_raw().
 *
 * Feeds an array of raw timing samples (alternating positive/negative
 * int16_t values representing mark/space durations in µs) through the
 * protocol decoder registry.  Uses inter-packet gap detection to
 * reconstruct pulse packets, then tries every registered decoder.
 *
 * This module is hardware-independent and compiles on both ARM and host.
 *
 * M1 Project — Hapax fork
 */

#ifndef SUBGHZ_RAW_DECODER_H
#define SUBGHZ_RAW_DECODER_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Constants — mirrored from m1_sub_ghz_decenc.h for testability              */
/*============================================================================*/

/* These must match the firmware values; they are re-declared here so
 * tests can include only this header without pulling in the full
 * hardware-dependent decode/encode header. */
#ifndef INTERPACKET_GAP_MIN
#define INTERPACKET_GAP_MIN         1500   /* µs */
#endif
#ifndef PACKET_PULSE_TIME_MIN
#define PACKET_PULSE_TIME_MIN       80     /* µs — keep in sync with m1_sub_ghz_decenc.h */
#endif
#ifndef PACKET_PULSE_COUNT_MIN
#define PACKET_PULSE_COUNT_MIN      40
#endif
#ifndef PACKET_PULSE_COUNT_MAX
#define PACKET_PULSE_COUNT_MAX      256
#endif

/*============================================================================*/
/* Decoded result entry                                                       */
/*============================================================================*/

/** Minimal decode result — contains only the fields needed for display. */
typedef struct {
    uint16_t protocol;      /**< Protocol index in the registry */
    uint64_t key;           /**< Decoded key value */
    uint16_t bit_len;       /**< Decoded bit length */
    uint16_t te;            /**< Timing element (µs) */
    uint32_t frequency;     /**< Source frequency (passed through from caller) */
    int16_t  rssi;          /**< RSSI (always 0 for offline decode) */
    /* Extended fields */
    uint32_t serial_number;
    uint32_t rolling_code;
    uint8_t  button_id;
    uint8_t  repeat_count;  /**< Number of times this protocol+key was decoded
                                 from distinct packet segments.  A genuine
                                 remote transmission repeats several times, so
                                 repeat_count >= 2 is a strong confidence
                                 indicator.  False-positive matches from timing
                                 coincidences typically appear only once. */
} SubGhzRawDecodeResult;

/*============================================================================*/
/* Callback interface — decouples from hardware globals                       */
/*============================================================================*/

/**
 * Decoder callback — tries to decode a packet of `pulse_count` pulses
 * stored in `pulse_buf`.
 *
 * On success, fills `out_result` and returns true.
 * On failure (no protocol matched), returns false.
 *
 * The implementation on ARM will forward to the global protocol registry +
 * subghz_decenc_ctl.  The test harness provides a mock.
 */
typedef bool (*SubGhzRawDecodeTryFn)(const uint16_t *pulse_buf,
                                      uint16_t pulse_count,
                                      SubGhzRawDecodeResult *out_result,
                                      void *user_ctx);

/*============================================================================*/
/* API                                                                        */
/*============================================================================*/

/**
 * Decode protocols from an array of raw timing samples.
 *
 * The raw_data array contains alternating positive/negative int16_t values
 * (mark/space durations in µs as stored in Flipper .sub RAW files).
 * The sign is stripped; only absolute durations are used.
 *
 * @param raw_data      Array of signed timing samples
 * @param raw_count     Number of samples in raw_data
 * @param frequency     Frequency in Hz (copied to results for display)
 * @param results       Output array for decoded results
 * @param max_results   Size of the results array
 * @param try_decode    Callback that attempts protocol decode on a pulse packet
 * @param user_ctx      Opaque context passed to try_decode
 * @return Number of unique protocols decoded (0 = none found)
 */
uint8_t subghz_decode_raw_offline(const int16_t *raw_data,
                                   uint16_t raw_count,
                                   uint32_t frequency,
                                   SubGhzRawDecodeResult *results,
                                   uint8_t max_results,
                                   SubGhzRawDecodeTryFn try_decode,
                                   void *user_ctx);

/*============================================================================*/
/* Canonical ARM-side decode callback                                         */
/*============================================================================*/

/**
 * Standard firmware bridge implementing SubGhzRawDecodeTryFn.
 *
 * Copies one packet of pulse timings into the global subghz_decenc_ctl
 * buffer and runs every registered protocol decoder against it.  On the
 * first decoder that succeeds and whose result can be read via
 * subghz_decenc_read(), fills *out_result and returns true.
 *
 * Implemented in subghz_protocol_registry.c.  This is the canonical
 * callback used by firmware code, while host-side tests may also build
 * that file with stubs to satisfy platform-specific dependencies.
 * Pass this directly to subghz_decode_raw_offline() instead of writing
 * a local static callback in each scene file.
 */
bool subghz_registry_decode_try_fn(const uint16_t *pulse_buf,
                                    uint16_t        pulse_count,
                                    SubGhzRawDecodeResult *out_result,
                                    void           *user_ctx);

#endif /* SUBGHZ_RAW_DECODER_H */
