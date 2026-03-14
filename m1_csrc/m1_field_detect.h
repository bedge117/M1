/* See COPYING.txt for license details. */

/*
 * m1_field_detect.h
 *
 * Passive field detection for NFC (13.56 MHz) and LF RFID (125 kHz).
 * Detects external reader fields without generating a carrier.
 */

#ifndef M1_FIELD_DETECT_H_
#define M1_FIELD_DETECT_H_

#include <stdint.h>
#include <stdbool.h>

/* Initialize hardware for field detection.
 * Returns 0 on success, -1 if NFC chip init failed. */
int m1_field_detect_start(void);

/* Shut down field detection hardware */
void m1_field_detect_stop(void);

/* Check for external 13.56 MHz NFC field.
 * Returns true if field is present. */
bool m1_field_detect_nfc(void);

/* Check for external ~125 kHz RFID field.
 * Returns true if field is present.
 * If frequency is non-NULL, writes the estimated carrier frequency in Hz.
 * If raw_transitions is non-NULL, writes the raw transition count (for debug). */
bool m1_field_detect_rfid(uint32_t *frequency);

/* Debug: returns last raw transition count from RFID sampling */
int m1_field_detect_rfid_raw(void);

/* Debug: returns ST25R3916 AUX_DISPLAY register value (bit 6 = efd_o) */
int m1_field_detect_nfc_raw(void);

/* Debug: returns ST25R3916 OP_CONTROL register value.
 * Bit 7 = oscillator ON, bits 1:0 = EFD mode (02=manual PDT). */
int m1_field_detect_nfc_opctl(void);

#endif /* M1_FIELD_DETECT_H_ */
