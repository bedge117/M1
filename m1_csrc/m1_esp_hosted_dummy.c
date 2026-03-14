/* See COPYING.txt for license details. */

/*
 *   m1_esp_hosted_dummy.c
 *
 *  Dummy functions for ESP32 package when porting to STM32 MCU
 *
 * M1 Project
 */

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_app_main.h"

/*************************** D E F I N E S ************************************/

//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/


/********************* F U N C T I O N   P R O T O T Y P E S ******************/

bool get_esp32_ready_status(void);
void esp_restart(void);
void esp32_app_init(void);
void test_get_available_wifi(void);
void esp32_app_init(void);
/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/*
 *
 */
/*============================================================================*/
bool get_esp32_ready_status(void)
{
	return get_esp32_main_init_status();
}



/*============================================================================*/
/*
 *
 */
/*============================================================================*/
void test_get_available_wifi(void)
{
	;
}


/*============================================================================*/
/*
 *
 */
/*============================================================================*/
void esp32_app_init(void)
{
	;
}

