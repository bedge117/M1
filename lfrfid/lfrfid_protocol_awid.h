/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_AWID_H_
#define LFRFID_PROTOCOL_AWID_H_

/* Flipper: AWID_ENCODED_DATA_SIZE = (96/8)+1 = 13 bytes */
#define AWID_ENCODED_SIZE    13  /* 96-bit frame + 1 for preamble overlap */
#define AWID_ENCODED_BIT_SIZE 96
#define AWID_DECODED_SIZE    9   /* 66 decoded bits (matches Flipper) */

extern const LFRFIDProtocolBase protocol_awid;

#endif /* LFRFID_PROTOCOL_AWID_H_ */
