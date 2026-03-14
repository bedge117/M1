/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_GALLAGHER_H_
#define LFRFID_PROTOCOL_GALLAGHER_H_

#define GALLAGHER_ENCODED_SIZE    14  /* 112 bits: 96 encoded + 16 preamble overlap */
#define GALLAGHER_DECODED_SIZE    8   /* 8 bytes decoded */
#define GALLAGHER_HALF_BIT_US     128 /* RF/32 */

extern const LFRFIDProtocolBase protocol_gallagher;

#endif /* LFRFID_PROTOCOL_GALLAGHER_H_ */
