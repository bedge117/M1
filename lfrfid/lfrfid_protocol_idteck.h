/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_IDTECK_H_
#define LFRFID_PROTOCOL_IDTECK_H_

#define IDTECK_US_PER_BIT          255
#define IDTECK_ENCODED_BIT_SIZE    64
#define IDTECK_PREAMBLE_DATA_SIZE  8
#define IDTECK_ENCODED_SIZE        ((IDTECK_ENCODED_BIT_SIZE / 8) + IDTECK_PREAMBLE_DATA_SIZE)  /* 16 bytes */
#define IDTECK_DECODED_SIZE        8

extern const LFRFIDProtocolBase protocol_idteck;

#endif /* LFRFID_PROTOCOL_IDTECK_H_ */
