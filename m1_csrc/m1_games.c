/* See COPYING.txt for license details. */

/*
*
* m1_games.c
*
* Common utilities for built-in games
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "m1_games.h"

/*************************** D E F I N E S ************************************/

//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Poll for a button press with timeout
 * @param  timeout_ms  Maximum time to wait in milliseconds
 * @retval game_button_t  The button pressed, or GAME_BTN_NONE on timeout
 */
/*============================================================================*/
game_button_t game_poll_button(uint32_t timeout_ms)
{
    S_M1_Main_Q_t q_item;
    S_M1_Buttons_Status btn_status;
    BaseType_t ret;
    game_button_t result = GAME_BTN_NONE;

    ret = xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(timeout_ms));
    if (ret != pdTRUE)
    {
        return GAME_BTN_NONE;
    }

    if (q_item.q_evt_type != Q_EVENT_KEYPAD)
    {
        return GAME_BTN_NONE;
    }

    ret = xQueueReceive(button_events_q_hdl, &btn_status, 0);
    if (ret != pdTRUE)
    {
        return GAME_BTN_NONE;
    }

    if (btn_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
    {
        result = GAME_BTN_UP;
    }
    else if (btn_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
    {
        result = GAME_BTN_DOWN;
    }
    else if (btn_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
    {
        result = GAME_BTN_LEFT;
    }
    else if (btn_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
    {
        result = GAME_BTN_RIGHT;
    }
    else if (btn_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
    {
        result = GAME_BTN_OK;
    }
    else if (btn_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
    {
        result = GAME_BTN_BACK;
        xQueueReset(main_q_hdl);
    }

    return result;
}


/*============================================================================*/
/*
 * @brief  Seed the game random number generator from hardware tick
 */
/*============================================================================*/
void game_rand_seed(void)
{
    srand(HAL_GetTick());
}


/*============================================================================*/
/*
 * @brief  Get a random number within a range (inclusive)
 * @param  min  Minimum value
 * @param  max  Maximum value
 * @retval int  Random number in [min, max]
 */
/*============================================================================*/
int game_rand_range(int min, int max)
{
    if (min >= max)
    {
        return min;
    }
    return min + (rand() % (max - min + 1));
}
