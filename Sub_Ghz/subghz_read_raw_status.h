/* See COPYING.txt for license details. */

#ifndef SUBGHZ_READ_RAW_STATUS_H
#define SUBGHZ_READ_RAW_STATUS_H

/*
 * Hardware-independent helper for the Sub-GHz Read Raw "Record failed" screen.
 *
 * start_raw_rx() in m1_subghz_scene_read_raw.c can fail for three distinct
 * reasons.  Collapsing them into a single "SD card error" message hides whether
 * the fault is a heap shortage (write-buffer alloc) or a genuine SD/FatFs
 * failure, which makes field reports ambiguous (see issue #610).  This module
 * maps the failure code to a short, distinct status line so the cause is
 * obvious from the device screen.
 */
typedef enum {
    SUBGHZ_READ_RAW_START_OK       = 0, /* recording started                  */
    SUBGHZ_READ_RAW_START_ERR_OOM  = 1, /* ring-buffer (capture) malloc failed */
    SUBGHZ_READ_RAW_START_ERR_SD   = 2, /* SD status / dir / file open failed  */
    SUBGHZ_READ_RAW_START_ERR_MEM  = 3, /* SD write-buffer alloc failed        */
} subghz_read_raw_start_err_t;

/* Returns the first status line shown on the "Record failed" message box for a
 * given start error.  Always returns a non-NULL, stable string literal. */
const char *subghz_read_raw_start_err_line1(subghz_read_raw_start_err_t err);

#endif /* SUBGHZ_READ_RAW_STATUS_H */
