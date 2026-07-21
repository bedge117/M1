/*
 * M1 Protocol — frame build/parse and CRC (M1 side copy)
 *
 * Identical to ESP32 copy. Keep in sync.
 */

#include "m1_protocol.h"
#include <string.h>

uint16_t m1p_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

size_t m1p_build_frame(uint8_t *buf, size_t buf_size,
                       uint8_t cmd, uint8_t flags, uint16_t seq,
                       const uint8_t *payload, uint16_t payload_len)
{
    size_t total = M1P_HEADER_SIZE + payload_len + M1P_CRC_SIZE;
    if (total > buf_size || payload_len > M1P_MAX_PAYLOAD_SIZE)
        return 0;

    m1p_header_t *hdr = (m1p_header_t *)buf;
    hdr->sync  = M1P_SYNC_WORD;
    hdr->cmd   = cmd;
    hdr->flags = flags;
    hdr->seq   = seq;
    hdr->len   = payload_len;

    if (payload_len > 0 && payload != NULL)
        memcpy(buf + M1P_HEADER_SIZE, payload, payload_len);

    uint16_t crc = m1p_crc16(buf, M1P_HEADER_SIZE + payload_len);
    buf[M1P_HEADER_SIZE + payload_len]     = (uint8_t)(crc >> 8);
    buf[M1P_HEADER_SIZE + payload_len + 1] = (uint8_t)(crc & 0xFF);

    return total;
}

int m1p_parse_frame(const uint8_t *buf, size_t buf_len,
                    m1p_header_t *hdr_out,
                    const uint8_t **payload_out,
                    uint16_t *payload_len_out)
{
    if (buf_len < M1P_HEADER_SIZE + M1P_CRC_SIZE)
        return -1;

    const m1p_header_t *hdr = (const m1p_header_t *)buf;

    if (hdr->sync != M1P_SYNC_WORD)
        return -1;

    size_t total = M1P_HEADER_SIZE + hdr->len + M1P_CRC_SIZE;
    if (total > buf_len || hdr->len > M1P_MAX_PAYLOAD_SIZE)
        return -1;

    uint16_t expected = m1p_crc16(buf, M1P_HEADER_SIZE + hdr->len);
    uint16_t received = ((uint16_t)buf[M1P_HEADER_SIZE + hdr->len] << 8)
                      | buf[M1P_HEADER_SIZE + hdr->len + 1];
    if (expected != received)
        return -1;

    if (hdr_out)
        *hdr_out = *hdr;
    if (payload_out)
        *payload_out = (hdr->len > 0) ? (buf + M1P_HEADER_SIZE) : NULL;
    if (payload_len_out)
        *payload_len_out = hdr->len;

    return 0;
}
