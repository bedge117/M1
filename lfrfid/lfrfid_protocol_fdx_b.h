/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_FDX_B_H_
#define LFRFID_PROTOCOL_FDX_B_H_

#define FDX_B_ENCODED_SIZE     18  /* 144 bits: 128 data + 16 preamble overlap */
#define FDX_B_DECODED_SIZE     11  /* full decoded payload */
#define FDX_B_HALF_BIT_US      128 /* RF/32: 32 * 4us = 128us half-bit */

extern const LFRFIDProtocolBase protocol_fdx_b;

#endif /* LFRFID_PROTOCOL_FDX_B_H_ */
