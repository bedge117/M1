/* See COPYING.txt for license details. */

/*
 * picopass_listener.h — PicoPass / iCLASS card emulation
 *
 * Emulates a PicoPass card responding to reader commands over
 * ISO15693/NFC-V transport via the ST25R3916 in listen mode.
 */

#ifndef PICOPASS_LISTENER_H_
#define PICOPASS_LISTENER_H_

#include "picopass.h"

/**
 * @brief Initialize PicoPass emulation from the current nfc_ctx dump.
 *
 * Sets up RFAL in NFC-V transparent mode and populates the emulation
 * state from the loaded PicoPass card data in nfc_ctx.
 *
 * @return true if dump is valid and emulation is ready
 */
bool picopass_listener_init(void);

/**
 * @brief Process one iteration of PicoPass emulation.
 *
 * Non-blocking — called periodically from the NFC worker task.
 * Polls for incoming reader commands and responds.
 */
void picopass_listener_process(void);

/**
 * @brief Stop PicoPass emulation and clean up.
 */
void picopass_listener_stop(void);

#endif /* PICOPASS_LISTENER_H_ */
