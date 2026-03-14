/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_PARADOX_H_
#define LFRFID_PROTOCOL_PARADOX_H_

#define PARADOX_ENCODED_SIZE    13  /* 96-bit frame + 1 preamble overlap */
#define PARADOX_DECODED_SIZE    6   /* 48 decoded bits */

extern const LFRFIDProtocolBase protocol_paradox;

#endif /* LFRFID_PROTOCOL_PARADOX_H_ */
