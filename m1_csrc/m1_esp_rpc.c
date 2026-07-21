/*
 * m1esp.c -- Shared implementation of the M1 <-> ESP32-C6 binary RPC protocol.
 * Portable C (no ESP-IDF or STM32 HAL deps) so BOTH firmwares compile it.
 */

#include "m1_esp_rpc.h"
#include <string.h>

/* ---------- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect) ---------- */
uint16_t m1esp_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---------- Capability bitmap <-> uint64 (wire is LE uint8[8]) ---------- */
uint64_t m1esp_caps_get(const uint8_t bitmap[8])
{
    uint64_t c = 0;
    for (int i = 0; i < 8; i++) {
        c |= (uint64_t)bitmap[i] << (8 * i);
    }
    return c;
}

void m1esp_caps_set(uint8_t bitmap[8], uint64_t caps)
{
    for (int i = 0; i < 8; i++) {
        bitmap[i] = (uint8_t)(caps >> (8 * i));
    }
}

/* ---------- Frame build ---------- */
size_t m1esp_build(uint8_t *buf, size_t buf_size,
                    m1esp_msgtype_t type, uint16_t msg_id,
                    const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len > M1ESP_MAX_PAYLOAD) return 0;
    size_t total = (size_t)M1ESP_HEADER_SIZE + payload_len + M1ESP_CRC_SIZE;
    if (buf_size < total) return 0;
    if (payload_len && !payload) return 0;

    m1esp_header_t hdr;
    hdr.magic       = M1ESP_MAGIC;
    hdr.version     = M1ESP_VERSION;
    hdr.msg_type    = (uint8_t)type;
    hdr.msg_id      = msg_id;
    hdr.payload_len = payload_len;

    memcpy(buf, &hdr, M1ESP_HEADER_SIZE);
    if (payload_len) {
        memcpy(buf + M1ESP_HEADER_SIZE, payload, payload_len);
    }

    uint16_t crc = m1esp_crc16(buf, M1ESP_HEADER_SIZE + payload_len);
    buf[M1ESP_HEADER_SIZE + payload_len]     = (uint8_t)(crc & 0xFF);
    buf[M1ESP_HEADER_SIZE + payload_len + 1] = (uint8_t)(crc >> 8);

    return total;
}

/* ---------- Frame parse / validate ---------- */
int m1esp_parse(const uint8_t *buf, size_t buf_len,
                 m1esp_header_t *hdr_out,
                 const uint8_t **payload_out, uint16_t *payload_len_out)
{
    if (buf_len < (size_t)M1ESP_HEADER_SIZE + M1ESP_CRC_SIZE) return -2;

    m1esp_header_t hdr;
    memcpy(&hdr, buf, M1ESP_HEADER_SIZE);

    if (hdr.magic != M1ESP_MAGIC)     return -1;
    if (hdr.version != M1ESP_VERSION) return -1;
    if (hdr.payload_len > M1ESP_MAX_PAYLOAD) return -4;

    size_t total = (size_t)M1ESP_HEADER_SIZE + hdr.payload_len + M1ESP_CRC_SIZE;
    if (buf_len < total) return -2;

    uint16_t rx_crc = (uint16_t)buf[M1ESP_HEADER_SIZE + hdr.payload_len]
                    | (uint16_t)buf[M1ESP_HEADER_SIZE + hdr.payload_len + 1] << 8;
    uint16_t calc = m1esp_crc16(buf, M1ESP_HEADER_SIZE + hdr.payload_len);
    if (rx_crc != calc) return -3;

    if (hdr_out)         *hdr_out = hdr;
    if (payload_out)     *payload_out = buf + M1ESP_HEADER_SIZE;
    if (payload_len_out) *payload_len_out = hdr.payload_len;
    return 0;
}
