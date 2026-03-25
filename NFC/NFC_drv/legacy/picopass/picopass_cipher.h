/* See COPYING.txt for license details. */

/*
 * picopass_cipher.h — iCLASS cipher + DES key diversification
 *
 * Implements:
 *   - Minimal single-DES (encrypt only — no library dependency)
 *   - hash0 key fortification (PicoPass proprietary)
 *   - Key diversification: div_key = hash0(DES(CSN, master_key))
 *   - iCLASS MAC calculation for READCHECK/CHECK authentication
 *
 * Based on Garcia et al. "Exposing iClass Key Diversification" and
 * the Proxmark3/Flipper loclass implementations.
 */

#ifndef PICOPASS_CIPHER_H_
#define PICOPASS_CIPHER_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Derive the diversified key from CSN + master key.
 *        div_key = hash0( DES_encrypt(CSN, master_key) )
 *
 * @param csn     Card Serial Number (8 bytes, block 0)
 * @param key     Master key (8 bytes)
 * @param div_key Output diversified key (8 bytes)
 */
void picopass_iclass_calc_div_key(const uint8_t csn[8],
                                  const uint8_t key[8],
                                  uint8_t div_key[8]);

/**
 * @brief Compute the 4-byte reader MAC for CHECK command.
 *
 * @param ccnr    Challenge: ePurse(8) + NR(4) = 12 bytes
 * @param div_key Diversified key (8 bytes)
 * @param mac     Output MAC (4 bytes)
 */
void picopass_iclass_calc_mac(const uint8_t ccnr[12],
                              const uint8_t div_key[8],
                              uint8_t mac[4]);

/**
 * @brief Compute the 4-byte tag MAC for verifying CHECK response.
 *
 * @param ccnr    Challenge: ePurse(8) + NR(4) = 12 bytes
 * @param div_key Diversified key (8 bytes)
 * @param mac     Output tag MAC (4 bytes)
 */
void picopass_iclass_calc_tag_mac(const uint8_t ccnr[12],
                                  const uint8_t div_key[8],
                                  uint8_t mac[4]);

/**
 * @brief Elite key diversification.
 *        Applies hash1/hash2/permutekey before standard diversification.
 *
 * @param csn     Card Serial Number (8 bytes)
 * @param key     Master key (8 bytes)
 * @param div_key Output diversified key (8 bytes)
 */
void picopass_iclass_calc_div_key_elite(const uint8_t csn[8],
                                        const uint8_t key[8],
                                        uint8_t div_key[8]);

/**
 * @brief Compute UPDATE command MAC (for write operations).
 *
 * @param block_num  Block number being written
 * @param data       8 bytes of data being written
 * @param div_key    Diversified key (8 bytes)
 * @param mac        Output MAC (4 bytes)
 */
void picopass_iclass_calc_update_mac(uint8_t block_num,
                                     const uint8_t data[8],
                                     const uint8_t div_key[8],
                                     uint8_t mac[4]);

#endif /* PICOPASS_CIPHER_H_ */
