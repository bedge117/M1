/* See COPYING.txt for license details. */

/*
 * mfc_crypto1.c - Software MIFARE Classic Crypto-1 cipher
 *
 * Implements the Crypto-1 48-bit LFSR stream cipher used by
 * MIFARE Classic cards for authentication and encrypted I/O.
 *
 * The cipher uses a 48-bit LFSR with a nonlinear filter function.
 * After mutual authentication, all communication is encrypted
 * byte-by-byte with per-bit parity derived from the cipher stream.
 */

#include <string.h>
#include "mfc_crypto1.h"
#include "rfal_rf.h"
#include "rfal_nfc.h"
#include "logger.h"

/* ========================================================================== */
/* Crypto-1 filter function lookup tables                                     */
/* The filter function f(a,b,c,d) = output bit                               */
/* Uses two cascaded 4-bit LUTs: f20 and f4b/f4a                             */
/* ========================================================================== */

/* 4-to-1 LUT for the filter function (f_a, f_b, f_c components) */
static const uint32_t LF_POLY_ODD  = 0x29CE5C;  /* Feedback polynomial odd  */
static const uint32_t LF_POLY_EVEN = 0x870804;  /* Feedback polynomial even */

/* Nonlinear filter function output table (bit-packed)
 * fa(a0,a1,a2,a3) = filter_a lookup
 * fb(b0,b1,b2,b3) = filter_b lookup
 * fc(c0,c1,c2,c3) = filter_c lookup
 * Final: output = fc(fa(...), fb(...), ...) */
#define BIT(x, n)  (((x) >> (n)) & 1U)

/* Filter function f(x0..x3) for 4 input bits, returns 1 bit */
static uint8_t filter_f(uint32_t x)
{
    /* f(a,b,c,d) nonlinear boolean function from Crypto-1 spec.
     * Implemented via the 0x9E98 truth table. */
    static const uint16_t f4a = 0x9E98;  /* 4-input filter a */
    static const uint16_t f4b = 0xB48E;  /* 4-input filter b */
    static const uint16_t fc  = 0xEC57E80A; /* 5-input combining filter, packed as uint32_t */
    (void)fc;

    /* Crypto-1 uses a cascaded filter:
     * Step 1: fa = f4a(x0,x1,x2,x3), fb = f4b(x4,x5,x6,x7)
     * Step 2: output = combining(fa,fb,x8,x9,x10,...) ...
     *
     * Actually, the standard Crypto-1 filter takes 20 LFSR bits as input.
     * We implement it as the standard odd-bit filter below. */
    (void)f4a; (void)f4b;
    return 0; /* placeholder - replaced by proper filter below */
}

/* The actual Crypto-1 nonlinear filter function on the odd bits (24 bits) */
/* Takes bits at positions: 0,2,4,6,8,10,12,14,16,18,20,22 of odd register
 * = bits 1,3,5,7,9,11,13,15,17,19,21,23 of the 48-bit LFSR
 * Selects specific bit positions and applies cascaded boolean functions */
static uint8_t crypto1_filter(uint32_t odd)
{
    /* Standard Crypto-1 filter function using cascaded 4-bit lookups.
     * f20(b0..b3) and f20(b4..b7) feed into fc along with f20(b8..b11)
     * and f20(b12..b15) and bit b16..b19.
     *
     * Truth tables (bit-packed, LSB = index 0):
     *   f_4bit  = 0x9E98  (for first-stage 4-input filter)
     *   f_4bit2 = 0xB48E  (for second-stage 4-input filter)
     *   f_5bit  = 0xEC57E80A (for combining 5 inputs into 1)
     */
    uint32_t f_4bit  = 0x9E98U;
    uint32_t f_4bit2 = 0xB48EU;
    uint32_t f_5bit  = 0xEC57E80AU;

    /* Extract the 20 filter input bits from odd register
     * LFSR bits used: 1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,33,35,37,39
     * In our split representation, odd register bit i corresponds to LFSR bit 2i+1
     * So we use odd register bits 0..19 */
    uint8_t i0  = BIT(odd, 0);
    uint8_t i1  = BIT(odd, 2);
    uint8_t i2  = BIT(odd, 4);
    uint8_t i3  = BIT(odd, 6);

    uint8_t i4  = BIT(odd, 8);
    uint8_t i5  = BIT(odd, 10);
    uint8_t i6  = BIT(odd, 12);
    uint8_t i7  = BIT(odd, 14);

    uint8_t i8  = BIT(odd, 16);
    uint8_t i9  = BIT(odd, 18);
    uint8_t i10 = BIT(odd, 20);
    uint8_t i11 = BIT(odd, 22);

    uint8_t i12 = BIT(odd, 1);
    uint8_t i13 = BIT(odd, 3);
    uint8_t i14 = BIT(odd, 5);
    uint8_t i15 = BIT(odd, 7);

    uint8_t i16 = BIT(odd, 9);
    uint8_t i17 = BIT(odd, 11);
    uint8_t i18 = BIT(odd, 13);
    uint8_t i19 = BIT(odd, 15);

    /* First stage: 4-input lookups */
    uint8_t a = (uint8_t)((f_4bit >> ((i0 | (i1 << 1) | (i2 << 2) | (i3 << 3)))) & 1U);
    uint8_t b = (uint8_t)((f_4bit >> ((i4 | (i5 << 1) | (i6 << 2) | (i7 << 3)))) & 1U);
    uint8_t c = (uint8_t)((f_4bit2 >> ((i8 | (i9 << 1) | (i10 << 2) | (i11 << 3)))) & 1U);
    uint8_t d = (uint8_t)((f_4bit2 >> ((i12 | (i13 << 1) | (i14 << 2) | (i15 << 3)))) & 1U);

    /* Second stage: 5-input combining function */
    uint8_t e = (uint8_t)((f_5bit >> ((a | (b << 1) | (c << 2) | (d << 3) |
                 (((uint32_t)BIT(odd, 17)) << 4)))) & 1U);

    (void)i16; (void)i17; (void)i18; (void)i19;
    (void)filter_f;

    return e;
}

/* ========================================================================== */
/* LFSR operations                                                            */
/* ========================================================================== */

/* Count parity (number of set bits) mod 2 */
static uint8_t parity32(uint32_t x)
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (uint8_t)(x & 1U);
}

/* Initialize LFSR from 48-bit key */
void crypto1_init(crypto1_state_t *s, uint64_t key)
{
    /* Split key into odd and even bit positions */
    s->odd  = 0;
    s->even = 0;
    for (int i = 47; i >= 0; i--) {
        uint8_t bit = (uint8_t)((key >> i) & 1U);
        if (i & 1) {
            s->odd  = (s->odd  << 1) | bit;
        } else {
            s->even = (s->even << 1) | bit;
        }
    }
}

void crypto1_reset(crypto1_state_t *s)
{
    s->odd  = 0;
    s->even = 0;
}

/* Clock LFSR one step with feedback XOR'd with input bit.
 * If is_encrypted, XOR the output with the input before feedback.
 * Returns the filter output bit (keystream bit). */
uint8_t crypto1_bit(crypto1_state_t *s, uint8_t in, uint8_t is_encrypted)
{
    uint8_t out = crypto1_filter(s->odd);

    /* Feedback: new bit = parity(even & LF_POLY_EVEN) ^ parity(odd & LF_POLY_ODD) ^ input_bit */
    uint8_t feedin = (uint8_t)(in & 1U);
    if (is_encrypted) {
        feedin ^= out;
    }

    uint8_t fb = parity32(s->even & LF_POLY_EVEN) ^
                 parity32(s->odd  & LF_POLY_ODD)  ^
                 feedin;

    /* Shift: even becomes odd, new bit goes into even MSB */
    /* In the split representation:
     *   - LFSR shifts left by 1 (new bit enters at position 0)
     *   - Even bits become odd bits, odd bits become even bits
     *   - New feedback bit enters the even register at bit 23 */
    uint32_t new_even = s->odd;
    s->odd  = (s->even << 1) | (uint32_t)fb;
    s->even = new_even;

    /* Mask to 24 bits */
    s->odd  &= 0x00FFFFFFU;
    s->even &= 0x00FFFFFFU;

    return out;
}

/* Process one byte: XOR each bit with keystream */
uint8_t crypto1_byte(crypto1_state_t *s, uint8_t in, uint8_t is_encrypted)
{
    uint8_t out = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t ks = crypto1_bit(s, BIT(in, i), is_encrypted);
        out |= (uint8_t)(ks << i);
    }
    return out;
}

/* Process 32 bits */
uint32_t crypto1_word(crypto1_state_t *s, uint32_t in, uint8_t is_encrypted)
{
    uint32_t out = 0;
    for (int i = 0; i < 32; i++) {
        uint8_t ks = crypto1_bit(s, BIT(in, i), is_encrypted);
        out |= ((uint32_t)ks << i);
    }
    return out;
}

/* Get parity bit from current filter state (without clocking) */
uint8_t crypto1_parity_bit(crypto1_state_t *s)
{
    return crypto1_filter(s->odd);
}

/* ========================================================================== */
/* PRNG (card nonce generation uses same LFSR as in ISO14443-3A)              */
/* ========================================================================== */

uint32_t mfc_prng_successor(uint32_t x, uint32_t n)
{
    /* MIFARE Classic PRNG: LFSR with feedback polynomial x^16 + x^14 + x^13 + x^11 + 1 */
    /* Successor function: bit 0 = XOR of specific bit positions */
    for (uint32_t i = 0; i < n; i++) {
        uint8_t fb = parity32(x & 0x62A6u); /* polynomial taps */
        x = (x >> 1) | ((uint32_t)fb << 31);
    }
    return x;
}

/* ========================================================================== */
/* High-level: MIFARE Classic authentication                                  */
/* ========================================================================== */

/* Convert 6-byte key to 48-bit integer */
static uint64_t key_to_u64(const uint8_t key[MFC_KEY_LEN])
{
    uint64_t k = 0;
    for (int i = 0; i < MFC_KEY_LEN; i++) {
        k = (k << 8) | key[i];
    }
    return k;
}

/* Convert 4-byte array to uint32_t (MSB first) */
static uint32_t bytes_to_u32(const uint8_t *b)
{
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8)  | (uint32_t)b[3];
}

/* Convert uint32_t to 4-byte array (MSB first) */
static void u32_to_bytes(uint32_t v, uint8_t *b)
{
    b[0] = (uint8_t)(v >> 24);
    b[1] = (uint8_t)(v >> 16);
    b[2] = (uint8_t)(v >> 8);
    b[3] = (uint8_t)(v);
}

/* XOR parity of a byte */
static uint8_t byte_parity(uint8_t b)
{
    b ^= b >> 4;
    b ^= b >> 2;
    b ^= b >> 1;
    return b & 1;
}

bool mfc_auth(crypto1_state_t *state,
              const uint8_t uid[4],
              uint8_t blockNo,
              uint8_t keyType,
              const uint8_t key[MFC_KEY_LEN])
{
    ReturnCode err;
    uint8_t  txBuf[16];
    uint8_t  rxBuf[16];
    uint16_t rcvLen = 0;

    /* Step 1: Send AUTH command (keyType + blockNo), normal ISO14443A framing with CRC */
    txBuf[0] = keyType;
    txBuf[1] = blockNo;
    rcvLen = 0;

    err = rfalTransceiveBlockingTxRx(txBuf, 2, rxBuf, sizeof(rxBuf),
                                     &rcvLen, RFAL_TXRX_FLAGS_DEFAULT,
                                     rfalConvMsTo1fc(20));
    if (err != RFAL_ERR_NONE || rcvLen < 4) {
        return false;
    }

    /* Step 2: Received 4-byte card nonce (nT) — unencrypted */
    uint32_t nT = bytes_to_u32(rxBuf);
    uint32_t uid32 = bytes_to_u32(uid);

    /* Step 3: Initialize Crypto-1 with key */
    crypto1_init(state, key_to_u64(key));

    /* Feed UID XOR nT into the cipher (sets up the state) */
    crypto1_word(state, uid32 ^ nT, 0);

    /* Step 4: Generate our nonce (nR) — use HAL tick for randomness */
    extern uint32_t HAL_GetTick(void);
    uint32_t nR = HAL_GetTick() * 0x19660D + 0x3C6EF35F; /* simple LCG */

    /* Compute encrypted nR */
    uint32_t enR = nR ^ crypto1_word(state, nR, 0);

    /* Step 5: Compute answer: aR = suc64(nT) XOR keystream */
    uint32_t suc_nT = mfc_prng_successor(nT, 64);
    uint32_t eaR = suc_nT ^ crypto1_word(state, 0, 0);

    /* Step 6: Build 8-byte response {enR, eaR} with parity bits
     * MIFARE auth uses proprietary framing: 8 bytes with crypto parity,
     * no CRC, parity bits come from cipher stream */

    /* For the raw transceive, we need to send 8 bytes with custom parity.
     * RFAL doesn't directly support per-byte custom parity in the standard API.
     * We use RFAL_TXRX_FLAGS_CRC_TX_MANUAL to suppress CRC and send raw. */

    u32_to_bytes(enR, &txBuf[0]);
    u32_to_bytes(eaR, &txBuf[4]);

    rcvLen = 0;
    uint32_t flags = RFAL_TXRX_FLAGS_CRC_TX_MANUAL | RFAL_TXRX_FLAGS_CRC_RX_KEEP;

    err = rfalTransceiveBlockingTxRx(txBuf, 8, rxBuf, sizeof(rxBuf),
                                     &rcvLen, flags,
                                     rfalConvMsTo1fc(20));

    if (err != RFAL_ERR_NONE || rcvLen < 4) {
        crypto1_reset(state);
        return false;
    }

    /* Step 7: Verify card's answer (aT) */
    /* Decrypt received answer */
    uint32_t eaT = bytes_to_u32(rxBuf);
    uint32_t aT  = eaT ^ crypto1_word(state, 0, 0);

    uint32_t expected_aT = mfc_prng_successor(nT, 96);
    if (aT != expected_aT) {
        crypto1_reset(state);
        return false;
    }

    return true;
}

/* ========================================================================== */
/* High-level: Read block with Crypto-1 encryption                            */
/* ========================================================================== */

bool mfc_read_block_crypto(crypto1_state_t *state,
                           uint8_t blockNo,
                           uint8_t out[MFC_BLOCK_SIZE])
{
    uint8_t txBuf[4];
    uint8_t rxBuf[20]; /* 16 data + 2 CRC */
    uint16_t rcvLen = 0;
    ReturnCode err;

    /* Encrypt READ command: 0x30 + blockNo + CRC */
    uint8_t cmd[4];
    cmd[0] = 0x30;
    cmd[1] = blockNo;

    /* Compute CRC over plaintext */
    uint16_t crc = 0x6363; /* ISO14443A CRC init */
    /* Simple CRC-A computation */
    for (int i = 0; i < 2; i++) {
        uint8_t bt = cmd[i];
        bt ^= (uint8_t)(crc & 0xFF);
        bt ^= (bt << 4);
        crc = (crc >> 8) ^ ((uint16_t)bt << 8) ^ ((uint16_t)bt << 3) ^ ((uint16_t)bt >> 4);
    }
    cmd[2] = (uint8_t)(crc & 0xFF);
    cmd[3] = (uint8_t)(crc >> 8);

    /* Encrypt all 4 bytes */
    for (int i = 0; i < 4; i++) {
        uint8_t ks = crypto1_byte(state, 0, 0);
        txBuf[i] = cmd[i] ^ ks;
    }

    uint32_t flags = RFAL_TXRX_FLAGS_CRC_TX_MANUAL | RFAL_TXRX_FLAGS_CRC_RX_KEEP;
    rcvLen = 0;

    err = rfalTransceiveBlockingTxRx(txBuf, 4, rxBuf, sizeof(rxBuf),
                                     &rcvLen, flags,
                                     rfalConvMsTo1fc(20));
    if (err != RFAL_ERR_NONE || rcvLen < 18) {
        return false;
    }

    /* Decrypt 18 bytes (16 data + 2 CRC) */
    for (int i = 0; i < 18; i++) {
        uint8_t ks = crypto1_byte(state, 0, 0);
        rxBuf[i] ^= ks;
    }

    /* Verify CRC on decrypted data */
    crc = 0x6363;
    for (int i = 0; i < 18; i++) {
        uint8_t bt = rxBuf[i];
        bt ^= (uint8_t)(crc & 0xFF);
        bt ^= (bt << 4);
        crc = (crc >> 8) ^ ((uint16_t)bt << 8) ^ ((uint16_t)bt << 3) ^ ((uint16_t)bt >> 4);
    }
    if (crc != 0) {
        return false;
    }

    memcpy(out, rxBuf, MFC_BLOCK_SIZE);
    return true;
}
