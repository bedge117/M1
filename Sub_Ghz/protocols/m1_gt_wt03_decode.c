/* See COPYING.txt for license details. */

/*
*  m1_gt_wt03_decode.c
*
*  M1 sub-ghz GT-WT03 40-bit weather station decoding
*  te=500, ratio 1:2, tol 25%
*/

#include "m1_sub_ghz_decenc.h"

uint8_t subghz_decode_generic_pwm(uint16_t p, uint16_t pulsecount);

uint8_t subghz_decode_gt_wt03(uint16_t p, uint16_t pulsecount)
{
    return subghz_decode_generic_pwm(p, pulsecount);
}
