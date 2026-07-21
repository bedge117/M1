/* See COPYING.txt for license details. */

#include "subghz_read_raw_status.h"

const char *subghz_read_raw_start_err_line1(subghz_read_raw_start_err_t err)
{
    switch (err)
    {
        case SUBGHZ_READ_RAW_START_ERR_OOM:
            return "Low memory";
        case SUBGHZ_READ_RAW_START_ERR_MEM:
            return "SD mem error";
        case SUBGHZ_READ_RAW_START_ERR_SD:
            return "SD card error";
        case SUBGHZ_READ_RAW_START_OK:
        default:
            return "";
    }
}
