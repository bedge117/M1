/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_PYRAMID_H_
#define LFRFID_PROTOCOL_PYRAMID_H_

/* Flipper: PYRAMID_ENCODED_DATA_SIZE = preamble(3) + data(13) + preamble(3) = 19 */
#define PYRAMID_PREAMBLE_SIZE  3
#define PYRAMID_DATA_SIZE      13
#define PYRAMID_ENCODED_SIZE   (PYRAMID_PREAMBLE_SIZE + PYRAMID_DATA_SIZE + PYRAMID_PREAMBLE_SIZE)
#define PYRAMID_ENCODED_BIT_SIZE ((PYRAMID_PREAMBLE_SIZE + PYRAMID_DATA_SIZE) * 8)  /* 128 */
#define PYRAMID_DECODED_SIZE   4   /* format(1) + FC(1) + Card(2) */

extern const LFRFIDProtocolBase protocol_pyramid;

#endif /* LFRFID_PROTOCOL_PYRAMID_H_ */
