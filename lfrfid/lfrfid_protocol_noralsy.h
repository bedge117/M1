/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_NORALSY_H_
#define LFRFID_PROTOCOL_NORALSY_H_

#define NORALSY_ENCODED_SIZE    12  /* 96 bits */
#define NORALSY_DECODED_SIZE    12  /* copy full frame */
#define NORALSY_HALF_BIT_US     128 /* RF/32 */

extern const LFRFIDProtocolBase protocol_noralsy;

#endif /* LFRFID_PROTOCOL_NORALSY_H_ */
