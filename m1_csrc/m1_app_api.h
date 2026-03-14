/* See COPYING.txt for license details. */

/*
*
*  m1_app_api.h
*
*  Firmware API interface for M1 external apps
*
* M1 Project
*
*/

#ifndef M1_APP_API_H_
#define M1_APP_API_H_

#include "m1_elf_loader.h"

/* Initialize the API table (compute hashes, sort). Call once at startup. */
void m1_app_api_init(void);

/* Get the firmware's API interface for symbol resolution */
const m1_api_interface_t *m1_app_get_api(void);

#endif /* M1_APP_API_H_ */
