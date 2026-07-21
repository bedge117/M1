/* See COPYING.txt for license details. */

/*
 * cc1101_fec.c
 *
 * CC1101 FEC (Forward Error Correction) encode / decode utility.
 *
 * Implements the rate-1/2 convolutional Viterbi codec and 4-byte block
 * interleaver used by the TI CC1101 Sub-GHz transceiver when its FEC
 * feature is enabled.  Hardware-independent: only <stdint.h> and <string.h>.
 *
 * Algorithm origin:
 *   TI CC1101 reference implementation (aTrellisSourceStateLut / fecDecode)
 *   URH plugin port by SpaceTeddy:
 *     https://github.com/SpaceTeddy/urh-cc1101_FEC_encode_decode_plugin
 *   Adapted to C (from C++) with re-entrant context struct for embedded use.
 *
 * See cc1101_fec.h for the public API.
 */

#include "cc1101_fec.h"

/* ==========================================================================
 * Private look-up tables
 * ========================================================================== */

/* FEC convolutional encoder output table (2 output bits per input nibble) */
static const uint8_t fecEncodeTable[16] = {
    0, 3, 1, 2,
    3, 0, 2, 1,
    3, 0, 2, 1,
    0, 3, 1, 2
};

/* Source states for each destination state (2 possible predecessors) */
static const uint8_t aTrellisSourceStateLut[8][2] = {
    {0, 4}, /* State {0,4} -> State 0 */
    {0, 4}, /* State {0,4} -> State 1 */
    {1, 5}, /* State {1,5} -> State 2 */
    {1, 5}, /* State {1,5} -> State 3 */
    {2, 6}, /* State {2,6} -> State 4 */
    {2, 6}, /* State {2,6} -> State 5 */
    {3, 7}, /* State {3,7} -> State 6 */
    {3, 7}, /* State {3,7} -> State 7 */
};

/* Expected encoder output bits (2b symbol) for each transition */
static const uint8_t aTrellisTransitionOutput[8][2] = {
    {0, 3}, /* State {0,4} -> State 0: {"00","11"} */
    {3, 0}, /* State {0,4} -> State 1: {"11","00"} */
    {1, 2}, /* State {1,5} -> State 2: {"01","10"} */
    {2, 1}, /* State {1,5} -> State 3: {"10","01"} */
    {3, 0}, /* State {2,6} -> State 4: {"11","00"} */
    {0, 3}, /* State {2,6} -> State 5: {"00","11"} */
    {2, 1}, /* State {3,7} -> State 6: {"10","01"} */
    {1, 2}, /* State {3,7} -> State 7: {"01","10"} */
};

/* Encoder input bit that causes each destination-state transition */
static const uint8_t aTrellisTransitionInput[8] = {0, 1, 0, 1, 0, 1, 0, 1};

/* ==========================================================================
 * Private helpers
 * ========================================================================== */

/** Hamming weight: number of set bits in a byte. */
static uint8_t cc1101_fec_hamm_weight(uint8_t a)
{
    a = (uint8_t)(((a & 0xAAu) >> 1u) + (a & 0x55u));
    a = (uint8_t)(((a & 0xCCu) >> 2u) + (a & 0x33u));
    a = (uint8_t)(((a & 0xF0u) >> 4u) + (a & 0x0Fu));
    return a;
}

static uint8_t cc1101_fec_min_u8(uint8_t a, uint8_t b)
{
    return (a <= b) ? a : b;
}

/* ==========================================================================
 * Public API — size helpers
 * ========================================================================== */

uint16_t cc1101_fec_encoded_size(uint8_t data_len)
{
    /* inputNum = data_len + 3 (length byte + data + 2 CRC)
     * fecNum  = 2 * ((inputNum / 2) + 1)
     * encoded = fecNum * 2  */
    uint16_t inputNum = (uint16_t)data_len + 3u;
    uint16_t fecNum   = 2u * ((inputNum / 2u) + 1u);
    return fecNum * 2u;
}

uint16_t cc1101_fec_decoded_size(uint16_t encoded_len)
{
    if ((encoded_len < 8u) || ((encoded_len % 4u) != 0u)) return 0u;
    return ((encoded_len - 4u) / 2u) + 1u;
}

/* ==========================================================================
 * Public API — CRC
 * ========================================================================== */

uint16_t cc1101_fec_crc16(uint8_t data_byte, uint16_t crc_reg)
{
    uint8_t i;
    for (i = 0u; i < 8u; i++) {
        if (((crc_reg & 0x8000u) >> 8u) ^ (data_byte & 0x80u))
            crc_reg = (uint16_t)((crc_reg << 1u) ^ 0x8005u);
        else
            crc_reg = (uint16_t)(crc_reg << 1u);
        data_byte = (uint8_t)(data_byte << 1u);
    }
    return crc_reg;
}

/* ==========================================================================
 * Public API — encode
 * ========================================================================== */

uint16_t cc1101_fec_encode(uint8_t *pEncData, const uint8_t *payload, uint8_t payload_len)
{
    /* Working buffer: [0]=length, [1..N]=data, [N+1..N+2]=CRC, [N+3..N+4]=term */
    uint8_t  buf[CC1101_FEC_MAX_PAYLOAD + 5u];
    uint16_t inputNum;
    uint16_t fecNum;
    uint16_t checksum;
    uint16_t fec[CC1101_FEC_MAX_PAYLOAD * 4u + 20u]; /* interleaved 2b symbols */
    uint16_t i, j;
    uint16_t fecReg, fecOutput;
    uint32_t intOutput;

    if ((pEncData == NULL) || (payload == NULL) || (payload_len > CC1101_FEC_MAX_PAYLOAD))
        return 0u;

    /* Build the pre-FEC buffer */
    buf[0] = payload_len;                       /* length byte     */
    for (i = 0u; i < payload_len; i++)
        buf[1u + i] = payload[i];               /* payload bytes   */

    inputNum = (uint16_t)payload_len + 1u;      /* index after data */

    /* CRC-16/IBM over [0 .. payload_len] inclusive */
    checksum = 0xFFFFu;
    for (i = 0u; i <= payload_len; i++)
        checksum = cc1101_fec_crc16(buf[i], checksum);

    buf[inputNum++] = (uint8_t)(checksum >> 8u);        /* CRC high */
    buf[inputNum++] = (uint8_t)(checksum & 0x00FFu);    /* CRC low  */

    /* Trellis terminator */
    buf[inputNum]      = 0x0Bu;
    buf[inputNum + 1u] = 0x0Bu;

    /* fecNum = total bytes to encode (includes one terminator byte) */
    fecNum = 2u * ((inputNum / 2u) + 1u);

    /* Convolutional encode: each input byte → 2 output bytes (rate 1/2) */
    fecReg = 0u;
    for (i = 0u; i < fecNum; i++) {
        fecReg = (fecReg & 0x700u) | (uint16_t)(buf[i] & 0xFFu);
        fecOutput = 0u;
        for (j = 0u; j < 8u; j++) {
            fecOutput = (fecOutput << 2u) | fecEncodeTable[fecReg >> 7u];
            fecReg    = (fecReg << 1u) & 0x7FFu;
        }
        fec[i * 2u]      = (uint8_t)(fecOutput >> 8u);
        fec[i * 2u + 1u] = (uint8_t)(fecOutput & 0xFFu);
    }

    /* 4-byte block interleave */
    for (i = 0u; i < fecNum * 2u; i += 4u) {
        intOutput = 0u;
        for (j = 0u; j < 4u * 4u; j++) {
            intOutput = (intOutput << 2u) |
                        ((fec[i + (~j & 0x03u)] >> (2u * ((j & 0x0Cu) >> 2u))) & 0x03u);
        }
        pEncData[i]      = (uint8_t)((intOutput >> 24u) & 0xFFu);
        pEncData[i + 1u] = (uint8_t)((intOutput >> 16u) & 0xFFu);
        pEncData[i + 2u] = (uint8_t)((intOutput >>  8u) & 0xFFu);
        pEncData[i + 3u] = (uint8_t)( intOutput         & 0xFFu);
    }

    return fecNum * 2u;
}

/* ==========================================================================
 * Public API — decode (low-level)
 * ========================================================================== */

void cc1101_fec_decode_init(CC1101_FEC_DecCtx_t *ctx)
{
    uint8_t n;
    if (ctx == NULL) return;
    memset(ctx, 0, sizeof(*ctx));
    /* Only state 0 has cost 0 at start; all other states start at max cost */
    for (n = 1u; n < 8u; n++)
        ctx->nCost[0][n] = 100u;
    ctx->iLastBuf  = 0u;
    ctx->iCurrBuf  = 1u;
    ctx->nPathBits = 0u;
}

uint16_t cc1101_fec_decode(CC1101_FEC_DecCtx_t *ctx,
                            uint8_t             *pDecData,
                            const uint8_t       *pInData,
                            uint16_t             nRemBytes)
{
    uint8_t  aDeintData[4];
    uint8_t  nIterations;
    uint16_t nOutputBytes = 0u;
    uint8_t  nMinCost;
    int8_t   iBit = 6;           /* MSB-first: start at bit-pair offset 6 */
    uint8_t  iOut;
    const uint8_t *pIn;

    if ((ctx == NULL) || (pDecData == NULL) || (pInData == NULL))
        return 0u;

    /* De-interleave 4 input bytes.  Loop uses a signed index counting down
     * from 3 to 0 to avoid relying on unsigned underflow for termination. */
    for (iOut = 0u; iOut < 4u; iOut++) {
        uint8_t dataByte = 0u;
        int8_t  iInS;
        for (iInS = 3; iInS >= 0; iInS--)
            dataByte = (uint8_t)((dataByte << 2u) |
                       ((pInData[(uint8_t)iInS] >> (2u * iOut)) & 0x03u));
        aDeintData[iOut] = dataByte;
    }
    pIn = aDeintData;

    /* Process 16 encoder symbols (4 bytes × 4 symbols/byte) */
    for (nIterations = 16u; nIterations > 0u; nIterations--) {
        uint8_t iDestState;
        uint8_t symbol = (uint8_t)((*pIn >> iBit) & 0x03u);

        nMinCost = 0xFFu;

        iBit -= 2;
        if (iBit < 0) {
            iBit = 6;
            pIn++;
        }

        for (iDestState = 0u; iDestState < 8u; iDestState++) {
            uint8_t  nCost0, nCost1;
            uint8_t  iSrcState0, iSrcState1;
            uint8_t  nInputBit;

            nInputBit = aTrellisTransitionInput[iDestState];

            iSrcState0 = aTrellisSourceStateLut[iDestState][0];
            nCost0 = ctx->nCost[ctx->iLastBuf][iSrcState0];
            nCost0 = (uint8_t)(nCost0 + cc1101_fec_hamm_weight(
                         (uint8_t)(symbol ^ aTrellisTransitionOutput[iDestState][0])));

            iSrcState1 = aTrellisSourceStateLut[iDestState][1];
            nCost1 = ctx->nCost[ctx->iLastBuf][iSrcState1];
            nCost1 = (uint8_t)(nCost1 + cc1101_fec_hamm_weight(
                         (uint8_t)(symbol ^ aTrellisTransitionOutput[iDestState][1])));

            if (nCost0 <= nCost1) {
                ctx->nCost[ctx->iCurrBuf][iDestState] = nCost0;
                nMinCost = cc1101_fec_min_u8(nMinCost, nCost0);
                ctx->aPath[ctx->iCurrBuf][iDestState] =
                    (ctx->aPath[ctx->iLastBuf][iSrcState0] << 1u) | nInputBit;
            } else {
                ctx->nCost[ctx->iCurrBuf][iDestState] = nCost1;
                nMinCost = cc1101_fec_min_u8(nMinCost, nCost1);
                ctx->aPath[ctx->iCurrBuf][iDestState] =
                    (ctx->aPath[ctx->iLastBuf][iSrcState1] << 1u) | nInputBit;
            }
        }

        ctx->nPathBits++;

        /* Once 32 path bits accumulated, output one byte */
        if (ctx->nPathBits == 32u) {
            if (nRemBytes == 0u) {
                return nOutputBytes;
            }
            *pDecData++ = (uint8_t)((ctx->aPath[ctx->iCurrBuf][0] >> 24u) & 0xFFu);
            nOutputBytes++;
            ctx->nPathBits -= 8u;
            nRemBytes--;
            if (nRemBytes == 0u) {
                return nOutputBytes;
            }
        }

        /* Flush remaining bytes when near end of packet (3-symbol terminator) */
        if ((nRemBytes <= 3u) &&
            (ctx->nPathBits == (uint8_t)((8u * nRemBytes) + 3u))) {
            while ((ctx->nPathBits >= 8u) && (nRemBytes > 0u)) {
                *pDecData++ = (uint8_t)((ctx->aPath[ctx->iCurrBuf][0] >>
                               (ctx->nPathBits - 8u)) & 0xFFu);
                nOutputBytes++;
                ctx->nPathBits -= 8u;
                nRemBytes--;
                if (nRemBytes == 0u) {
                    return nOutputBytes;
                }
            }
            return nOutputBytes;
        }

        /* Swap buffers */
        ctx->iLastBuf = ctx->iCurrBuf;
        ctx->iCurrBuf = (uint8_t)(1u - ctx->iCurrBuf);
    }

    /* Normalise costs (subtract minimum so no state overflows) */
    {
        uint8_t iState;
        for (iState = 0u; iState < 8u; iState++)
            ctx->nCost[ctx->iLastBuf][iState] -= nMinCost;
    }

    return nOutputBytes;
}

/* ==========================================================================
 * Public API — decode (high-level)
 * ========================================================================== */

uint16_t cc1101_fec_decode_packet(uint8_t       *pDecData,
                                   const uint8_t *encoded_data,
                                   uint16_t       encoded_len)
{
    CC1101_FEC_DecCtx_t ctx;
    uint16_t nBytes;
    uint16_t count;
    uint8_t  rxBuf[4];
    uint16_t total = 0u;
    uint8_t  rxBufIdx;

    if ((pDecData == NULL) || (encoded_data == NULL) || (encoded_len < 8u) ||
        ((encoded_len & 0x03u) != 0u))
        return 0u;

    nBytes = cc1101_fec_decoded_size(encoded_len);
    if (nBytes == 0u) return 0u;

    cc1101_fec_decode_init(&ctx);

    count = 0u;
    /* Feed 4 bytes at a time.  The Viterbi decoder accumulates path history
     * before emitting bytes — it is normal for early calls to return 0 bytes.
     * Keep iterating until all encoded bytes are consumed or nBytes reaches 0. */
    while ((count < encoded_len) && (nBytes > 0u)) {
        uint16_t nBytesOut;
        if (count + 4u > encoded_len) return 0u; /* truncated input */
        for (rxBufIdx = 0u; rxBufIdx < 4u; rxBufIdx++)
            rxBuf[rxBufIdx] = encoded_data[count++];

        nBytesOut = cc1101_fec_decode(&ctx, pDecData, rxBuf, nBytes);
        pDecData += nBytesOut;
        total    += nBytesOut;
        nBytes    = (nBytes > nBytesOut) ? (uint16_t)(nBytes - nBytesOut) : 0u;
    }

    if ((nBytes != 0u) || (count != encoded_len)) return 0u;
    return total;
}
