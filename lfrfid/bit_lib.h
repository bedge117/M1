/*
 * bit_lib.h — bit manipulation helpers.
 *
 * Ported from Flipper Zero firmware (lib/bit_lib/bit_lib.h, GPLv3). The M1
 * lfrfid protocol decoders/encoders were derived from the same upstream; this
 * brings over the shared bit-array toolbox needed by the T5577 write encoders.
 * The two printf-based debug helpers (bit_lib_print_bits/regions) are omitted
 * for the embedded target.
 */
#ifndef M1_LFRFID_BIT_LIB_H_
#define M1_LFRFID_BIT_LIB_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOPBIT(X) (1 << ((X) - 1))

typedef enum {
    BitLibParityEven,
    BitLibParityOdd,
    BitLibParityAlways0,
    BitLibParityAlways1,
} BitLibParity;

#define bit_lib_increment_index(index, length) (index = (((index) + 1) % (length)))
#define bit_lib_bit_is_set(data, index)     (((data) & (1 << (index))) != 0)
#define bit_lib_bit_is_not_set(data, index) (((data) & (1 << (index))) == 0)

void     bit_lib_push_bit(uint8_t* data, size_t data_size, bool bit);
void     bit_lib_set_bit(uint8_t* data, size_t position, bool bit);
void     bit_lib_set_bits(uint8_t* data, size_t position, uint8_t byte, uint8_t length);
bool     bit_lib_get_bit(const uint8_t* data, size_t position);
uint8_t  bit_lib_get_bits(const uint8_t* data, size_t position, uint8_t length);
uint16_t bit_lib_get_bits_16(const uint8_t* data, size_t position, uint8_t length);
uint32_t bit_lib_get_bits_32(const uint8_t* data, size_t position, uint8_t length);
uint64_t bit_lib_get_bits_64(const uint8_t* data, size_t position, uint8_t length);
bool     bit_lib_test_parity_32(uint32_t bits, BitLibParity parity);
bool     bit_lib_test_parity(const uint8_t* bits, size_t position, uint8_t length,
                             BitLibParity parity, uint8_t parity_length);
size_t   bit_lib_add_parity(const uint8_t* data, size_t position, uint8_t* dest,
                            size_t dest_position, uint8_t source_length,
                            uint8_t parity_length, BitLibParity parity);
size_t   bit_lib_remove_bit_every_nth(uint8_t* data, size_t position, uint8_t length, uint8_t n);
void     bit_lib_copy_bits(uint8_t* data, size_t position, size_t length,
                           const uint8_t* source, size_t source_position);
void     bit_lib_reverse_bits(uint8_t* data, size_t position, uint8_t length);
uint8_t  bit_lib_get_bit_count(uint32_t data);
uint16_t bit_lib_reverse_16_fast(uint16_t data);
uint8_t  bit_lib_reverse_8_fast(uint8_t byte);
uint16_t bit_lib_crc8(uint8_t const* data, size_t data_size, uint8_t polynom,
                      uint8_t init, bool ref_in, bool ref_out, uint8_t xor_out);
uint16_t bit_lib_crc16(uint8_t const* data, size_t data_size, uint16_t polynom,
                       uint16_t init, bool ref_in, bool ref_out, uint16_t xor_out);
void     bit_lib_num_to_bytes_be(uint64_t src, uint8_t len, uint8_t* dest);
void     bit_lib_num_to_bytes_le(uint64_t src, uint8_t len, uint8_t* dest);
uint64_t bit_lib_bytes_to_num_be(const uint8_t* src, uint8_t len);
uint64_t bit_lib_bytes_to_num_le(const uint8_t* src, uint8_t len);
uint64_t bit_lib_bytes_to_num_bcd(const uint8_t* src, uint8_t len, bool* is_bcd);

#ifdef __cplusplus
}
#endif

#endif /* M1_LFRFID_BIT_LIB_H_ */
