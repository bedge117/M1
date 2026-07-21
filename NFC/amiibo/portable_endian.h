/*
 * portable_endian.h — minimal big-endian helpers for the amiibo module.
 * The STM32H5 (Cortex-M33) is little-endian, so host<->big-endian is a byteswap.
 * Only the 16-bit helpers used by nfc3d/amiibo.c (copy_app_data) are provided.
 */
#ifndef M1_PORTABLE_ENDIAN_H
#define M1_PORTABLE_ENDIAN_H

#include <stdint.h>

static inline uint16_t m1_bswap16(uint16_t x)
{
    return (uint16_t)((x << 8) | (x >> 8));
}

#ifndef htobe16
#define htobe16(x) m1_bswap16(x)
#endif
#ifndef be16toh
#define be16toh(x) m1_bswap16(x)
#endif

#endif /* M1_PORTABLE_ENDIAN_H */
