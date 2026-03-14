/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_FDX_A_H_
#define LFRFID_PROTOCOL_FDX_A_H_

#define FDX_A_PREAMBLE_SIZE    2
#define FDX_A_DATA_SIZE        10
#define FDX_A_ENCODED_SIZE     (FDX_A_PREAMBLE_SIZE + FDX_A_DATA_SIZE + FDX_A_PREAMBLE_SIZE)  /* 14 */
#define FDX_A_DECODED_SIZE     5

extern const LFRFIDProtocolBase protocol_fdx_a;

#endif /* LFRFID_PROTOCOL_FDX_A_H_ */
