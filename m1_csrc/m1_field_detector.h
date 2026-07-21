/* See COPYING.txt for license details. */
/*
 * m1_field_detector.h — NFC/RFID reader-field detector (native feature).
 *
 * Senses whether a nearby card *reader* is emitting a 13.56 MHz (NFC) or
 * 125 kHz (LF RFID) field — useful for locating hidden readers/skimmers.
 * Native port of the SDK example app "Field Detector".
 */
#ifndef M1_FIELD_DETECTOR_H_
#define M1_FIELD_DETECTOR_H_

void m1_field_detector_run(void);

#endif /* M1_FIELD_DETECTOR_H_ */
