/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_PAC_STANLEY_H_
#define LFRFID_PROTOCOL_PAC_STANLEY_H_

#define PAC_STANLEY_US_PER_CYCLE     256
#define PAC_STANLEY_HALF_BIT_US      128  /* half of RF/64 cycle */
#define PAC_STANLEY_ENCODED_BIT_SIZE 128
#define PAC_STANLEY_ENCODED_SIZE     ((PAC_STANLEY_ENCODED_BIT_SIZE / 8) + 1)  /* 17 bytes */
#define PAC_STANLEY_DECODED_SIZE     4

extern const LFRFIDProtocolBase protocol_pac_stanley;

#endif /* LFRFID_PROTOCOL_PAC_STANLEY_H_ */
