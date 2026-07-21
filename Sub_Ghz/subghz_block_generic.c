/* See COPYING.txt for license details. */

/*
 * subghz_block_generic.c
 *
 * Bridge between Flipper's SubGhzBlockGeneric and M1's global decode state.
 *
 * M1 Project — Hapax fork
 */

#include "subghz_block_generic.h"
#include "m1_sub_ghz_decenc.h"

void subghz_block_generic_commit_to_m1(const SubGhzBlockGeneric *instance,
                                        uint16_t proto_idx)
{
    subghz_decenc_ctl.n64_decodedvalue  = instance->data;
    subghz_decenc_ctl.ndecodedbitlength = instance->data_count_bit;
    subghz_decenc_ctl.ndecodedprotocol  = proto_idx;
    subghz_decenc_ctl.n32_serialnumber  = instance->serial;
    subghz_decenc_ctl.n32_rollingcode   = instance->cnt;
    subghz_decenc_ctl.n8_buttonid       = instance->btn;
}
