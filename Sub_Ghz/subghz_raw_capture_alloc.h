/* See COPYING.txt for license details. */

#ifndef SUBGHZ_RAW_CAPTURE_ALLOC_H
#define SUBGHZ_RAW_CAPTURE_ALLOC_H

#include <stdbool.h>
#include <stddef.h>

typedef void *(*subghz_raw_capture_alloc_fn)(size_t size);
typedef void (*subghz_raw_capture_free_fn)(void *ptr);

/* Hardware-independent helper used by Read Raw capture setup to verify that
 * the SD writer's heap reserve is still available after capture buffers are
 * allocated. */
bool subghz_raw_capture_reserve_heap(size_t bytes,
                                     subghz_raw_capture_alloc_fn alloc_fn,
                                     subghz_raw_capture_free_fn free_fn);

#endif /* SUBGHZ_RAW_CAPTURE_ALLOC_H */
