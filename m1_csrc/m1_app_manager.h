/* See COPYING.txt for license details. */

/*
*
*  m1_app_manager.h
*
*  App manager for loading and running external .m1app files from SD card
*
* M1 Project
*
*/

#ifndef M1_APP_MANAGER_H_
#define M1_APP_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

/* Initialize the app manager (call once at startup) */
void m1_app_manager_init(void);

/* Browse and run apps from SD card — called as menu sub_func */
void game_apps_browser_run(void);

#endif /* M1_APP_MANAGER_H_ */
