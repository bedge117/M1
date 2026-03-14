/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_JABLOTRON_H_
#define LFRFID_PROTOCOL_JABLOTRON_H_

#define JABLOTRON_ENCODED_SIZE    10  /* 80 bits: 64 encoded + 16 preamble overlap */
#define JABLOTRON_DECODED_SIZE    5   /* 5 bytes decoded */
#define JABLOTRON_SHORT_US        256 /* RF/64: bi-phase short period */
#define JABLOTRON_LONG_US         512 /* RF/64: bi-phase long period */
#define JABLOTRON_JITTER_US       120 /* +/- tolerance */

extern const LFRFIDProtocolBase protocol_jablotron;

#endif /* LFRFID_PROTOCOL_JABLOTRON_H_ */
