/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_NEXWATCH_H_
#define LFRFID_PROTOCOL_NEXWATCH_H_

#define NEXWATCH_US_PER_BIT          255
#define NEXWATCH_ENCODED_BIT_SIZE    96
#define NEXWATCH_ENCODED_DATA_SIZE   (NEXWATCH_ENCODED_BIT_SIZE / 8)  /* 12 bytes */
#define NEXWATCH_DECODED_SIZE        8

extern const LFRFIDProtocolBase protocol_nexwatch;

#endif /* LFRFID_PROTOCOL_NEXWATCH_H_ */
