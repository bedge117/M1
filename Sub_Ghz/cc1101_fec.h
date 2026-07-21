/* See COPYING.txt for license details. */

/*
 * cc1101_fec.h
 *
 * CC1101 FEC (Forward Error Correction) encode / decode utility.
 *
 * The TI CC1101 Sub-GHz transceiver has a built-in FEC feature that applies
 * rate-1/2 convolutional encoding (Viterbi code) + bit interleaving + CRC-16/IBM
 * to each packet.  Many IoT sensors, weather stations, and home-automation
 * remotes use this feature.  The M1 receives their raw GFSK/OOK packets via the
 * SI4463, so FEC decoding must be done in software to recover the payload.
 *
 * Hardware-independent — only depends on <stdint.h> and <string.h>.
 *
 * Algorithm origin:
 *   TI CC1101 reference implementation (aTrellisSourceStateLut / fecDecode)
 *   URH plugin port by SpaceTeddy:
 *     https://github.com/SpaceTeddy/urh-cc1101_FEC_encode_decode_plugin
 *   Adapted to C (from C++) for embedded use in the M1 Hapax firmware.
 */

#ifndef CC1101_FEC_H
#define CC1101_FEC_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------
 * Limits
 * ------------------------------------------------------------------ */

/** Maximum payload bytes that can be FEC-encoded in one call. */
#define CC1101_FEC_MAX_PAYLOAD 60u

/* ------------------------------------------------------------------
 * Decoder context (Viterbi state across 4-byte chunks)
 *
 * Initialise with cc1101_fec_decode_init() before each new packet.
 * Feed 4 encoded bytes at a time to cc1101_fec_decode().
 * ------------------------------------------------------------------ */
typedef struct {
    uint8_t  nCost[2][8];   /**< Accumulated path cost per destination state */
    uint32_t aPath[2][8];   /**< 32-bit path history per destination state */
    uint8_t  iLastBuf;      /**< Index of the "last" cost/path buffer (0 or 1) */
    uint8_t  iCurrBuf;      /**< Index of the "current" cost/path buffer (0 or 1) */
    uint8_t  nPathBits;     /**< Number of bits currently accumulated in aPath */
} CC1101_FEC_DecCtx_t;

/* ------------------------------------------------------------------
 * Size helpers
 * ------------------------------------------------------------------ */

/**
 * @brief  Number of FEC-encoded bytes produced from @p data_len payload bytes.
 *
 * CC1101 FEC doubles the data (rate 1/2) and rounds to a 4-byte boundary after
 * prepending a length byte and appending 2 CRC bytes + 1 trellis-terminator byte.
 * Formula: 4 * ((data_len + 3) / 2 + 1)  (integer division).
 */
uint16_t cc1101_fec_encoded_size(uint8_t data_len);

/**
 * @brief  Number of decoded bytes expected from @p encoded_len FEC bytes.
 *
 * Formula: ((encoded_len - 4) / 2) + 1
 *
 * Returns 0 if @p encoded_len is less than 8 or not a multiple of 4 — the
 * same conditions that cc1101_fec_decode_packet() enforces.
 *
 * The decoded output includes a leading length byte and trailing 2 CRC bytes.
 * The caller extracts the payload from decoded[1 .. decoded[0]].
 */
uint16_t cc1101_fec_decoded_size(uint16_t encoded_len);

/* ------------------------------------------------------------------
 * Encode
 * ------------------------------------------------------------------ */

/**
 * @brief  Encode @p payload_len bytes with CC1101 FEC.
 *
 * The function prepends a length byte, appends a CRC-16/IBM, appends a trellis
 * terminator, then applies rate-1/2 convolutional encoding followed by
 * 4-byte block interleaving — exactly as the CC1101 hardware does it.
 *
 * @param  pEncData    Output buffer. Must be at least
 *                     cc1101_fec_encoded_size(payload_len) bytes.
 * @param  payload     Input payload bytes (read-only).
 * @param  payload_len Number of payload bytes (max CC1101_FEC_MAX_PAYLOAD).
 * @return             Number of bytes written to @p pEncData, or 0 on error.
 */
uint16_t cc1101_fec_encode(uint8_t *pEncData, const uint8_t *payload, uint8_t payload_len);

/* ------------------------------------------------------------------
 * Decode (low-level — 4 bytes at a time)
 * ------------------------------------------------------------------ */

/**
 * @brief  Initialise a decode context before processing a new packet.
 *
 * Must be called once before the first cc1101_fec_decode() call for each
 * new packet.
 *
 * @param  ctx  Decode context to initialise.
 */
void cc1101_fec_decode_init(CC1101_FEC_DecCtx_t *ctx);

/**
 * @brief  Decode 4 bytes of interleaved FEC data using the Viterbi algorithm.
 *
 * Call cc1101_fec_decode_init() once before the first call per packet, then
 * feed exactly 4 encoded bytes per call, passing the remaining decoded-byte
 * count so the decoder knows when to flush the trellis.
 *
 * @param  ctx       Decode context (persistent across calls for one packet).
 * @param  pDecData  Output buffer for decoded bytes.
 * @param  pInData   Exactly 4 bytes of interleaved FEC-encoded input.
 * @param  nRemBytes Remaining decoded bytes expected (decremented by caller
 *                   by the return value of each call).
 * @return           Number of decoded bytes written to @p pDecData.
 */
uint16_t cc1101_fec_decode(CC1101_FEC_DecCtx_t *ctx,
                            uint8_t             *pDecData,
                            const uint8_t       *pInData,
                            uint16_t             nRemBytes);

/* ------------------------------------------------------------------
 * Decode (high-level — whole packet at once)
 * ------------------------------------------------------------------ */

/**
 * @brief  Decode a complete FEC-encoded packet in one call.
 *
 * Convenience wrapper around cc1101_fec_decode_init() + cc1101_fec_decode()
 * that handles the 4-byte chunking loop internally.
 *
 * The decoded output layout is:
 *   decoded[0]              = payload length (N)
 *   decoded[1 .. N]         = payload bytes
 *   decoded[N+1 .. N+2]     = CRC-16/IBM (big-endian)
 *
 * To verify the CRC, compute CRC-16/IBM (init=0xFFFF) over decoded[0..N]
 * and compare with (decoded[N+1] << 8) | decoded[N+2].
 *
 * @param  pDecData      Output buffer, at least
 *                       cc1101_fec_decoded_size(encoded_len) bytes.
 * @param  encoded_data  FEC-encoded input bytes (without preamble / sync word).
 * @param  encoded_len   Number of encoded bytes (must be a multiple of 4,
 *                       and at least 8).
 * @return               Total decoded bytes written, or 0 on error.
 */
uint16_t cc1101_fec_decode_packet(uint8_t       *pDecData,
                                   const uint8_t *encoded_data,
                                   uint16_t       encoded_len);

/**
 * @brief  Compute CRC-16/IBM (polynomial 0x8005, init 0xFFFF) over one byte.
 *
 * Matches the CRC appended by the CC1101 hardware and by cc1101_fec_encode().
 * Accumulate over a sequence of bytes by passing the result of the previous
 * call as @p crc_reg:
 *
 *   uint16_t crc = 0xFFFF;
 *   for (i = 0; i < n; i++) crc = cc1101_fec_crc16(data[i], crc);
 *
 * @param  data_byte  Byte to process.
 * @param  crc_reg    Running CRC (start with 0xFFFF).
 * @return            Updated CRC value.
 */
uint16_t cc1101_fec_crc16(uint8_t data_byte, uint16_t crc_reg);

#ifdef __cplusplus
}
#endif

#endif /* CC1101_FEC_H */
