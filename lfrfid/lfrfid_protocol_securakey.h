/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_SECURAKEY_H_
#define LFRFID_PROTOCOL_SECURAKEY_H_

#define SECURAKEY_ENCODED_SIZE    12  /* 96 bits */
#define SECURAKEY_DECODED_SIZE    6   /* 48 bits */
#define SECURAKEY_HALF_BIT_US     160 /* RF/40 */

extern const LFRFIDProtocolBase protocol_securakey;

#endif /* LFRFID_PROTOCOL_SECURAKEY_H_ */
