/* See COPYING.txt for license details. */
#ifndef LFRFID_PROTOCOL_HID_EX_H_
#define LFRFID_PROTOCOL_HID_EX_H_

#define HID_EX_PREAMBLE         0x1D
#define HID_EX_DATA_BYTES       23
#define HID_EX_ENCODED_SIZE     (1 + HID_EX_DATA_BYTES + 1)  /* 25 */
#define HID_EX_DECODED_SIZE     12

extern const LFRFIDProtocolBase protocol_hid_ex;

#endif /* LFRFID_PROTOCOL_HID_EX_H_ */
