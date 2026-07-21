/* See COPYING.txt for license details. */

#include "subghz_raw_capture_alloc.h"

bool subghz_raw_capture_reserve_heap(size_t bytes,
                                     subghz_raw_capture_alloc_fn alloc_fn,
                                     subghz_raw_capture_free_fn free_fn)
{
    void *reserve;

    if (bytes == 0U)
        return true;

    if (alloc_fn == NULL || free_fn == NULL)
        return false;

    reserve = alloc_fn(bytes);
    if (reserve == NULL)
        return false;

    free_fn(reserve);
    return true;
}
