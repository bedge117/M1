/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_KERI_H_
#define LFRFID_PROTOCOL_KERI_H_

#define KERI_US_PER_BIT          255
#define KERI_ENCODED_BIT_SIZE    64
#define KERI_ENCODED_DATA_SIZE   (((KERI_ENCODED_BIT_SIZE) / 8) + 5)
#define KERI_DECODED_SIZE        4

extern const LFRFIDProtocolBase protocol_keri;

#endif /* LFRFID_PROTOCOL_KERI_H_ */
