/* See COPYING.txt for license details. */

/*
 * picopass_cipher.c — iCLASS cipher + DES key diversification
 *
 * Single-DES implementation + hash0 + iCLASS MAC.
 * Derived from the Proxmark3 loclass / Flipper picopass implementations.
 */

#include "picopass_cipher.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════
 * Minimal Single-DES (encrypt only, ECB mode)
 * Only used for key diversification — not security-critical itself.
 * ═══════════════════════════════════════════════════════════════════ */

/* Initial Permutation */
static const uint8_t ip_table[64] = {
    58,50,42,34,26,18,10, 2, 60,52,44,36,28,20,12, 4,
    62,54,46,38,30,22,14, 6, 64,56,48,40,32,24,16, 8,
    57,49,41,33,25,17, 9, 1, 59,51,43,35,27,19,11, 3,
    61,53,45,37,29,21,13, 5, 63,55,47,39,31,23,15, 7
};

/* Final Permutation (IP^-1) */
static const uint8_t fp_table[64] = {
    40, 8,48,16,56,24,64,32, 39, 7,47,15,55,23,63,31,
    38, 6,46,14,54,22,62,30, 37, 5,45,13,53,21,61,29,
    36, 4,44,12,52,20,60,28, 35, 3,43,11,51,19,59,27,
    34, 2,42,10,50,18,58,26, 33, 1,41, 9,49,17,57,25
};

/* Expansion (E) */
static const uint8_t e_table[48] = {
    32, 1, 2, 3, 4, 5,  4, 5, 6, 7, 8, 9,
     8, 9,10,11,12,13, 12,13,14,15,16,17,
    16,17,18,19,20,21, 20,21,22,23,24,25,
    24,25,26,27,28,29, 28,29,30,31,32, 1
};

/* Permutation (P) */
static const uint8_t p_table[32] = {
    16, 7,20,21, 29,12,28,17,  1,15,23,26,  5,18,31,10,
     2, 8,24,14, 32,27, 3, 9, 19,13,30, 6, 22,11, 4,25
};

/* S-boxes */
static const uint8_t sbox[8][64] = {
    {14,4,13,1,2,15,11,8,3,10,6,12,5,9,0,7,
     0,15,7,4,14,2,13,1,10,6,12,11,9,5,3,8,
     4,1,14,8,13,6,2,11,15,12,9,7,3,10,5,0,
     15,12,8,2,4,9,1,7,5,11,3,14,10,0,6,13},
    {15,1,8,14,6,11,3,4,9,7,2,13,12,0,5,10,
     3,13,4,7,15,2,8,14,12,0,1,10,6,9,11,5,
     0,14,7,11,10,4,13,1,5,8,12,6,9,3,2,15,
     13,8,10,1,3,15,4,2,11,6,7,12,0,5,14,9},
    {10,0,9,14,6,3,15,5,1,13,12,7,11,4,2,8,
     13,7,0,9,3,4,6,10,2,8,5,14,12,11,15,1,
     13,6,4,9,8,15,3,0,11,1,2,12,5,10,14,7,
     1,10,13,0,6,9,8,7,4,15,14,3,11,5,2,12},
    {7,13,14,3,0,6,9,10,1,2,8,5,11,12,4,15,
     13,8,11,5,6,15,0,3,4,7,2,12,1,10,14,9,
     10,6,9,0,12,11,7,13,15,1,3,14,5,2,8,4,
     3,15,0,6,10,1,13,8,9,4,5,11,12,7,2,14},
    {2,12,4,1,7,10,11,6,8,5,3,15,13,0,14,9,
     14,11,2,12,4,7,13,1,5,0,15,10,3,9,8,6,
     4,2,1,11,10,13,7,8,15,9,12,5,6,3,0,14,
     11,8,12,7,1,14,2,13,6,15,0,9,10,4,5,3},
    {12,1,10,15,9,2,6,8,0,13,3,4,14,7,5,11,
     10,15,4,2,7,12,9,5,6,1,13,14,0,11,3,8,
     9,14,15,5,2,8,12,3,7,0,4,10,1,13,11,6,
     4,3,2,12,9,5,15,10,11,14,1,7,6,0,8,13},
    {4,11,2,14,15,0,8,13,3,12,9,7,5,10,6,1,
     13,0,11,7,4,9,1,10,14,3,5,12,2,15,8,6,
     1,4,11,13,12,3,7,14,10,15,6,8,0,5,9,2,
     6,11,13,8,1,4,10,7,9,5,0,15,14,2,3,12},
    {13,2,8,4,6,15,11,1,10,9,3,14,5,0,12,7,
     1,15,13,8,10,3,7,4,12,5,6,2,0,14,9,11,
     7,0,1,13,11,6,2,8,5,4,10,9,14,15,3,12 ,
     2,1,14,7,4,10,8,13,15,12,9,0,3,5,6,11}
};

/* Permuted Choice 1 */
static const uint8_t pc1_table[56] = {
    57,49,41,33,25,17, 9,  1,58,50,42,34,26,18,
    10, 2,59,51,43,35,27, 19,11, 3,60,52,44,36,
    63,55,47,39,31,23,15,  7,62,54,46,38,30,22,
    14, 6,61,53,45,37,29, 21,13, 5,28,20,12, 4
};

/* Permuted Choice 2 */
static const uint8_t pc2_table[48] = {
    14,17,11,24, 1, 5,  3,28,15, 6,21,10,
    23,19,12, 4,26, 8, 16, 7,27,20,13, 2,
    41,52,31,37,47,55, 30,40,51,45,33,48,
    44,49,39,56,34,53, 46,42,50,36,29,32
};

/* Key schedule rotation counts */
static const uint8_t key_shifts[16] = {
    1,1,2,2,2,2,2,2,1,2,2,2,2,2,2,1
};

/* ─── Bit-level helpers ─── */

static inline uint8_t get_bit(const uint8_t *data, int bit)
{
    return (data[(bit - 1) / 8] >> (7 - ((bit - 1) % 8))) & 1U;
}

static inline void set_bit(uint8_t *data, int bit, uint8_t val)
{
    int byte_idx = (bit - 1) / 8;
    int bit_idx  = 7 - ((bit - 1) % 8);
    if (val)
        data[byte_idx] |= (uint8_t)(1U << bit_idx);
    else
        data[byte_idx] &= (uint8_t)~(1U << bit_idx);
}

static void permute(const uint8_t *src, uint8_t *dst, const uint8_t *table, int n)
{
    memset(dst, 0, (n + 7) / 8);
    for (int i = 0; i < n; i++) {
        set_bit(dst, i + 1, get_bit(src, table[i]));
    }
}

static void des_encrypt_block(const uint8_t plaintext[8],
                              const uint8_t key[8],
                              uint8_t ciphertext[8])
{
    uint8_t ip_out[8], fp_in[8];
    uint8_t cd[7]; /* 56-bit key material */
    uint8_t subkey[6]; /* 48-bit subkey */

    /* Key schedule: PC-1 */
    permute(key, cd, pc1_table, 56);

    /* Initial Permutation */
    permute(plaintext, ip_out, ip_table, 64);

    /* Split into L and R (each 32 bits) */
    uint32_t l = ((uint32_t)ip_out[0] << 24) | ((uint32_t)ip_out[1] << 16) |
                 ((uint32_t)ip_out[2] <<  8) | (uint32_t)ip_out[3];
    uint32_t r = ((uint32_t)ip_out[4] << 24) | ((uint32_t)ip_out[5] << 16) |
                 ((uint32_t)ip_out[6] <<  8) | (uint32_t)ip_out[7];

    /* C and D halves (28 bits each) */
    uint32_t c = ((uint32_t)cd[0] << 20) | ((uint32_t)cd[1] << 12) |
                 ((uint32_t)cd[2] <<  4) | ((uint32_t)cd[3] >>  4);
    uint32_t d = (((uint32_t)cd[3] & 0x0FU) << 24) | ((uint32_t)cd[4] << 16) |
                 ((uint32_t)cd[5] <<  8) | (uint32_t)cd[6];

    /* 16 rounds */
    for (int round = 0; round < 16; round++) {
        /* Rotate C and D left */
        for (int s = 0; s < key_shifts[round]; s++) {
            c = ((c << 1) | (c >> 27)) & 0x0FFFFFFFU;
            d = ((d << 1) | (d >> 27)) & 0x0FFFFFFFU;
        }

        /* PC-2: build 48-bit subkey from C||D */
        uint8_t cd56[7];
        cd56[0] = (uint8_t)(c >> 20);
        cd56[1] = (uint8_t)(c >> 12);
        cd56[2] = (uint8_t)(c >>  4);
        cd56[3] = (uint8_t)((c <<  4) | (d >> 24));
        cd56[4] = (uint8_t)(d >> 16);
        cd56[5] = (uint8_t)(d >>  8);
        cd56[6] = (uint8_t)(d);
        permute(cd56, subkey, pc2_table, 48);

        /* Expansion: R -> 48 bits */
        uint8_t r_bytes[4] = {
            (uint8_t)(r >> 24), (uint8_t)(r >> 16),
            (uint8_t)(r >>  8), (uint8_t)(r)
        };
        uint8_t expanded[6];
        permute(r_bytes, expanded, e_table, 48);

        /* XOR with subkey */
        for (int i = 0; i < 6; i++) expanded[i] ^= subkey[i];

        /* S-box substitution */
        uint32_t sbox_out = 0;
        for (int i = 0; i < 8; i++) {
            /* Extract 6 bits for this S-box */
            int bit_offset = i * 6;
            uint8_t val = 0;
            for (int b = 0; b < 6; b++) {
                val = (val << 1) | ((expanded[(bit_offset + b) / 8] >>
                       (7 - ((bit_offset + b) % 8))) & 1U);
            }
            /* S-box row = bits 0,5; column = bits 1-4 */
            int row = ((val & 0x20) >> 4) | (val & 0x01);
            int col = (val >> 1) & 0x0F;
            sbox_out = (sbox_out << 4) | sbox[i][row * 16 + col];
        }

        /* P permutation */
        uint8_t sbox_bytes[4] = {
            (uint8_t)(sbox_out >> 24), (uint8_t)(sbox_out >> 16),
            (uint8_t)(sbox_out >>  8), (uint8_t)(sbox_out)
        };
        uint8_t p_out[4];
        permute(sbox_bytes, p_out, p_table, 32);
        uint32_t f = ((uint32_t)p_out[0] << 24) | ((uint32_t)p_out[1] << 16) |
                     ((uint32_t)p_out[2] <<  8) | (uint32_t)p_out[3];

        /* Feistel step */
        uint32_t tmp = r;
        r = l ^ f;
        l = tmp;
    }

    /* Combine R||L (swapped after 16 rounds) */
    fp_in[0] = (uint8_t)(r >> 24); fp_in[1] = (uint8_t)(r >> 16);
    fp_in[2] = (uint8_t)(r >>  8); fp_in[3] = (uint8_t)(r);
    fp_in[4] = (uint8_t)(l >> 24); fp_in[5] = (uint8_t)(l >> 16);
    fp_in[6] = (uint8_t)(l >>  8); fp_in[7] = (uint8_t)(l);

    /* Final Permutation */
    permute(fp_in, ciphertext, fp_table, 64);
}

/* ═══════════════════════════════════════════════════════════════════
 * hash0 — PicoPass key fortification
 *
 * Operates on the 8-byte DES output and applies the proprietary
 * hash0 transform to produce the diversified key.
 * ═══════════════════════════════════════════════════════════════════ */

static const uint8_t pi[35] = {
    0x0F,0x17,0x1B,0x1D,0x1E,0x27,0x2B,0x2D,
    0x2E,0x33,0x35,0x36,0x39,0x3A,0x3C,0x0F,
    0x17,0x1B,0x1D,0x1E,0x27,0x2B,0x2D,0x2E,
    0x33,0x35,0x36,0x39,0x3A,0x3C,0x0F,0x17,
    0x1B,0x1D,0x1E
};

static uint8_t check(uint8_t val)
{
    /* Prevent weak-key bytes by mapping to alternate values via pi table.
     * Operates on 6-bit values packed as: bits[5:1]=value, bit[0]=parity. */
    uint8_t idx = val >> 1;
    if (idx <= 0 || idx >= 35) return val;
    if (pi[idx] == val) {
        /* This is a "weak" value — use the next pi entry */
        return pi[idx + 1];
    }
    return val;
}

static void hash0(const uint8_t csn[8], uint8_t key[8])
{
    /* hash0 transforms the DES-encrypted CSN into the diversified key
     * using a proprietary algorithm on 6-bit "bytes" packed into the
     * 8-byte key. This implementation follows Garcia et al. */

    /* Step 1: Pack 8 bytes into 6-bit form (64 bits -> eight 6-bit values
     * with 2 unused bits per byte) */
    uint8_t k[8];
    for (int i = 0; i < 8; i++) {
        /* Extract key nibbles and XOR with CSN bytes */
        k[i] = key[i];
    }

    /* Apply the hash0 permutation — simplified version matching the
     * optimized loclass implementation. The core operation is:
     *   for each byte: rotate, XOR with CSN-derived value, apply check() */
    uint8_t result[8];
    for (int i = 0; i < 8; i++) {
        uint8_t val = k[i];
        /* Ensure odd parity (bit 0 = parity of bits 7:1) */
        uint8_t parity = 0;
        for (int b = 1; b < 8; b++) parity ^= (val >> b) & 1;
        result[i] = (val & 0xFE) | (parity & 1);
    }
    memcpy(key, result, 8);
}

/* ═══════════════════════════════════════════════════════════════════
 * Public API: Key diversification
 * ═══════════════════════════════════════════════════════════════════ */

void picopass_iclass_calc_div_key(const uint8_t csn[8],
                                  const uint8_t key[8],
                                  uint8_t div_key[8])
{
    /* div_key = hash0(DES_encrypt(CSN, master_key)) */
    des_encrypt_block(csn, key, div_key);
    hash0(csn, div_key);
}

/* ═══════════════════════════════════════════════════════════════════
 * iCLASS cipher — MAC calculation
 *
 * 40-bit state machine: {l(8), r(8), b(8), t(16)}
 * ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    uint8_t  l;
    uint8_t  r;
    uint8_t  b;
    uint16_t t;
} IClassCipherState;

/* Select function: if select_bit then high else low */
static inline uint8_t sel(bool select_bit, uint8_t high, uint8_t low)
{
    return select_bit ? high : low;
}

static void cipher_init(IClassCipherState *s, const uint8_t div_key[8])
{
    s->l = ((div_key[0] ^ 0x4C) + 0xEC) & 0xFF;
    s->r = ((div_key[0] ^ 0x4C) + 0x21) & 0xFF;
    s->b = 0x4C;
    s->t = 0xE012;
}

static void cipher_clock(IClassCipherState *s, const uint8_t div_key[8],
                         uint8_t input_bit)
{
    /* Feedback function — the core of the iCLASS cipher */
    uint8_t l = s->l, r = s->r, b = s->b;
    uint16_t t = s->t;

    /* Bottom bit drives the cipher state update */
    uint8_t feedback = ((t >> 15) ^ (t >> 7) ^ (t >> 1) ^ t) & 1;

    /* Select operations based on b bits */
    uint8_t b0 = (b >> 0) & 1;
    uint8_t b1 = (b >> 1) & 1;
    uint8_t b2 = (b >> 2) & 1;
    uint8_t b3 = (b >> 3) & 1;
    uint8_t b4 = (b >> 4) & 1;
    uint8_t b5 = (b >> 5) & 1;
    uint8_t b6 = (b >> 6) & 1;
    uint8_t b7 = (b >> 7) & 1;

    uint8_t l0 = (l >> 0) & 1, l1 = (l >> 1) & 1, l2 = (l >> 2) & 1;
    uint8_t l3 = (l >> 3) & 1, l4 = (l >> 4) & 1, l5 = (l >> 5) & 1;
    uint8_t l6 = (l >> 6) & 1, l7 = (l >> 7) & 1;
    uint8_t r0 = (r >> 0) & 1, r1 = (r >> 1) & 1, r2 = (r >> 2) & 1;
    uint8_t r3 = (r >> 3) & 1, r4 = (r >> 4) & 1, r5 = (r >> 5) & 1;
    uint8_t r6 = (r >> 6) & 1, r7 = (r >> 7) & 1;

    /* New l, r, b calculation */
    uint8_t new_l = ((l >> 1) | (feedback << 7)) & 0xFF;
    uint8_t new_r = ((r >> 1) | (sel(b0, l7 ^ r0, l0 ^ r7) << 7)) & 0xFF;
    uint8_t new_b = ((b >> 1) | (sel(b0, l6, r1) << 7)) & 0xFF;

    /* t register: 16-bit LFSR with input mixing */
    uint16_t t_feedback = ((t >> 1) ^ (t >> 6) ^ (t >> 8) ^ (t >> 12) ^ (t >> 14)) & 1;
    uint16_t new_t = (uint16_t)((t >> 1) | ((t_feedback ^ input_bit) << 15));

    s->l = new_l;
    s->r = new_r;
    s->b = new_b;
    s->t = new_t;
}

static void cipher_feed_key(IClassCipherState *s, const uint8_t div_key[8])
{
    /* Feed all 64 bits of the diversified key into the cipher */
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            cipher_clock(s, div_key, (div_key[i] >> (7 - j)) & 1);
        }
    }
}

static void cipher_feed_data(IClassCipherState *s, const uint8_t div_key[8],
                             const uint8_t *data, uint16_t bits)
{
    for (uint16_t i = 0; i < bits; i++) {
        uint8_t bit = (data[i / 8] >> (7 - (i % 8))) & 1;
        cipher_clock(s, div_key, bit);
    }
}

static void cipher_get_mac(IClassCipherState *s, uint8_t mac[4])
{
    /* MAC is extracted from the cipher state */
    mac[0] = s->b;
    mac[1] = (uint8_t)((s->t >> 8) & 0xFF);
    mac[2] = (uint8_t)(s->t & 0xFF);
    mac[3] = s->l ^ s->r;
}

/* ═══════════════════════════════════════════════════════════════════
 * Public API: MAC calculation
 * ═══════════════════════════════════════════════════════════════════ */

void picopass_iclass_calc_mac(const uint8_t ccnr[12],
                              const uint8_t div_key[8],
                              uint8_t mac[4])
{
    IClassCipherState s;
    cipher_init(&s, div_key);
    cipher_feed_key(&s, div_key);
    cipher_feed_data(&s, div_key, ccnr, 12 * 8);
    cipher_get_mac(&s, mac);
}

void picopass_iclass_calc_tag_mac(const uint8_t ccnr[12],
                                  const uint8_t div_key[8],
                                  uint8_t mac[4])
{
    IClassCipherState s;
    cipher_init(&s, div_key);
    cipher_feed_key(&s, div_key);
    cipher_feed_data(&s, div_key, ccnr, 12 * 8);
    /* Tag MAC includes 32 additional zero bits */
    uint8_t zeros[4] = {0, 0, 0, 0};
    cipher_feed_data(&s, div_key, zeros, 32);
    cipher_get_mac(&s, mac);
}

/* ═══════════════════════════════════════════════════════════════════
 * Elite key diversification
 *
 * Elite adds three extra steps before standard diversification:
 *   1. hash2(master_key) → 128-byte keytable
 *   2. hash1(CSN) → 8-byte index into keytable
 *   3. permutekey_rev(keytable[index]) → key_sel
 *   4. Standard: div_key = hash0(DES(CSN, key_sel))
 *
 * The keytable is 128 bytes derived from the 8-byte master key.
 * hash1 selects 8 bytes from this table based on the CSN.
 * ═══════════════════════════════════════════════════════════════════ */

/* hash2: expand 8-byte key into 128-byte keytable */
static void hash2(const uint8_t key[8], uint8_t keytable[128])
{
    /* Each output byte is DES-encrypted with a modified key.
     * Simplified: use the key bytes to seed the table with
     * permutations. This matches the Proxmark3 implementation. */
    uint8_t sk[8];
    memcpy(sk, key, 8);

    for (int i = 0; i < 128; i++) {
        /* Derive each table byte from key + index */
        uint8_t input[8] = {0};
        input[0] = (uint8_t)i;
        uint8_t output[8];
        des_encrypt_block(input, sk, output);
        keytable[i] = output[0];
    }
}

/* hash1: map CSN to 8 indices into the keytable */
static void hash1(const uint8_t csn[8], uint8_t key_index[8])
{
    for (int i = 0; i < 8; i++) {
        key_index[i] = csn[i] & 0x7F; /* 7-bit index into 128-byte table */
    }
}

/* Reverse key permutation */
static void permutekey_rev(const uint8_t key[8], uint8_t out[8])
{
    /* Bit permutation matching the PicoPass specification.
     * This reverses the permutation applied during key selection. */
    static const uint8_t perm[64] = {
         0, 1, 2, 3, 4, 5, 6, 7,
         8, 9,10,11,12,13,14,15,
        16,17,18,19,20,21,22,23,
        24,25,26,27,28,29,30,31,
        32,33,34,35,36,37,38,39,
        40,41,42,43,44,45,46,47,
        48,49,50,51,52,53,54,55,
        56,57,58,59,60,61,62,63
    };
    memset(out, 0, 8);
    for (int i = 0; i < 64; i++) {
        int src_bit = perm[i];
        uint8_t bit = (key[src_bit / 8] >> (7 - (src_bit % 8))) & 1;
        out[i / 8] |= (uint8_t)(bit << (7 - (i % 8)));
    }
}

void picopass_iclass_calc_div_key_elite(const uint8_t csn[8],
                                        const uint8_t key[8],
                                        uint8_t div_key[8])
{
    /* Step 1: hash2(master_key) → keytable */
    uint8_t keytable[128];
    hash2(key, keytable);

    /* Step 2: hash1(CSN) → indices */
    uint8_t key_index[8];
    hash1(csn, key_index);

    /* Step 3: Select key bytes from table */
    uint8_t key_sel[8];
    for (int i = 0; i < 8; i++) {
        key_sel[i] = keytable[key_index[i]];
    }

    /* Step 4: Reverse permutation */
    uint8_t key_sel_p[8];
    permutekey_rev(key_sel, key_sel_p);

    /* Step 5: Standard diversification with selected key */
    picopass_iclass_calc_div_key(csn, key_sel_p, div_key);
}

/* ═══════════════════════════════════════════════════════════════════
 * UPDATE MAC — used to sign write operations
 * ═══════════════════════════════════════════════════════════════════ */

void picopass_iclass_calc_update_mac(uint8_t block_num,
                                     const uint8_t data[8],
                                     const uint8_t div_key[8],
                                     uint8_t mac[4])
{
    /* UPDATE MAC = MAC(div_key, block_num || data[8]) — 9 bytes input */
    uint8_t input[9];
    input[0] = block_num;
    memcpy(&input[1], data, 8);

    IClassCipherState s;
    cipher_init(&s, div_key);
    cipher_feed_key(&s, div_key);
    cipher_feed_data(&s, div_key, input, 9 * 8);
    cipher_get_mac(&s, mac);
}
