/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_VIKING_H_
#define LFRFID_PROTOCOL_VIKING_H_

#define VIKING_ENCODED_SIZE    11  /* 88 bits: 64 encoded + 24 preamble overlap */
#define VIKING_DECODED_SIZE    4   /* 32-bit card data */
#define VIKING_HALF_BIT_US     128 /* RF/32: 32 * 8us = 256us full, 128us half */

extern const LFRFIDProtocolBase protocol_viking;

#endif /* LFRFID_PROTOCOL_VIKING_H_ */
