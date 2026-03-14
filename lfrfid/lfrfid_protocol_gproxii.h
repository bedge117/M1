/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_GPROXII_H_
#define LFRFID_PROTOCOL_GPROXII_H_

#define GPROXII_ENCODED_SIZE    12  /* 96 bits */
#define GPROXII_DECODED_SIZE    12  /* 12 bytes for compatibility */
#define GPROXII_SHORT_US        256 /* RF/64: bi-phase short period */
#define GPROXII_LONG_US         512 /* RF/64: bi-phase long period */
#define GPROXII_JITTER_US       120 /* +/- tolerance */

extern const LFRFIDProtocolBase protocol_gproxii;

#endif /* LFRFID_PROTOCOL_GPROXII_H_ */
