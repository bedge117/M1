/* See COPYING.txt for license details. */

/*
 ******************************************************************************
 * nfc_driver.c - NFC Role Management and Worker Task
 ******************************************************************************
 * 
 * [Purpose]
 * - Top-level NFC role management (Poller/Listener switching)
 * - NFC worker task execution and state machine management
 * - Event-based NFC operation control via queue
 * 
 * [State Machine]
 * NFC_STATE_WAIT
 *   ↓ (Q_EVENT_NFC_START_READ / Q_EVENT_NFC_START_EMULATE)
 * NFC_STATE_INITIALIZE
 *   ↓ (NfcRole.nfc_init_func() called)
 * NFC_STATE_PROCESS
 *   ↓ (NfcRole.nfc_process_func() called periodically)
 * NFC_STATE_DONE
 *   ↓ (NfcRole.nfc_deinit_func() called)
 * NFC_STATE_WAIT (loop)
 * 
 * [Role Switching]
 * - NFC_ROLE_POLLER: Reader mode (card reading)
 *   → Uses NFC_Polling_Init/Process/DeInit functions
 * - NFC_ROLE_LISTENER: Emulation mode (card emulation)
 *   → Uses NFC_Listening_Init/Process/DeInit functions
 * 
 * [Worker Task]
 * - nfc_worker_task(): FreeRTOS task running periodically
 * - Receives events via queue (nfc_worker_q_hdl)
 * - Calls appropriate NFC functions based on current state
 * 
 ******************************************************************************
 */
#include <stdint.h>
#include "m1_nfc.h"
#include "main.h"
#include "nfc_driver.h"

#include "common/nfc_ctx.h"

#include "app_x-cube-nfcx.h"
#include "m1_log_debug.h"
#include "rfal_platform.h"
#include "m1_tasks.h"
#include "uiView.h"


S_M1_NfcFunc_t NfcRole = {
    //"NFC Poller Role",      //title
    NFC_Polling_Init,       //nfc_init_func
    NFC_Polling_Process,    //nfc_process_func
    NFC_Polling_DeInit,     //nfc_deinit_func
    0,                      //req_Evnet
};

static NfcRole_e s_currentRole = NFC_ROLE_POLLER;
static NfcStateMachine NfcState = NFC_STATE_WAIT;
EmuNfcA_t g_emuA = {0};

TaskHandle_t nfc_worker_task_hdl = NULL;
QueueHandle_t nfc_worker_q_hdl = NULL;


/*============================================================================*/
/**
 * @brief Emu_SetNfcA - Set NFC-A emulation data
 * 
 * Sets the NFC-A emulation data (UID, ATQA, SAK) in a thread-safe manner.
 * The UID length is set to 7 bytes if uid_len is 7, otherwise defaults to 4 bytes.
 * 
 * @param[in] uid Pointer to UID data
 * @param[in] uid_len UID length (4 or 7 bytes)
 * @param[in] atqa0 ATQA byte 0
 * @param[in] atqa1 ATQA byte 1
 * @param[in] sak SAK byte
 * @retval None
 */
/*============================================================================*/
void Emu_SetNfcA(const uint8_t* uid, uint8_t uid_len, uint8_t atqa0, uint8_t atqa1, uint8_t sak)
{
    taskENTER_CRITICAL();
    memset(&g_emuA, 0, sizeof(g_emuA));
    if (uid_len == 7) g_emuA.uid_len = 7;
    else               g_emuA.uid_len = 4;      // Default 4 bytes

    memcpy(g_emuA.uid, uid, g_emuA.uid_len);
    g_emuA.atqa[0] = atqa0;
    g_emuA.atqa[1] = atqa1;
    g_emuA.sak     = sak;
    g_emuA.valid   = true;
    taskEXIT_CRITICAL();
}

/*============================================================================*/
/**
 * @brief Emu_GetNfcA - Get NFC-A emulation data
 * 
 * Retrieves the current NFC-A emulation data in a thread-safe manner.
 * 
 * @param[out] out Pointer to output structure to store emulation data
 * @retval true If emulation data is valid and copied
 * @retval false If output pointer is NULL or data is invalid
 */
/*============================================================================*/
bool Emu_GetNfcA(EmuNfcA_t* out)
{
    if (!out) return false;
    bool ok;
    taskENTER_CRITICAL();
    ok = g_emuA.valid;
    if (ok) memcpy(out, &g_emuA, sizeof(g_emuA));
    taskEXIT_CRITICAL();

#if 0
    //platformLog("g_emuA.atqa[0] = %x g_emuA.atqa[1] = %x g_emuA.sak = %x g_emuA.valid = %x\r\n"
    //                                ,g_emuA.atqa[0], g_emuA.atqa[1],g_emuA.sak,g_emuA.valid);
    //platformLog("out.atqa[0] = %x out.atqa[1] = %x out.sak = %x out.valid = %x\r\n"
    //                                ,out->atqa[0], out->atqa[1],out->sak,out->valid);
#endif

    return ok;
}

/*============================================================================*/
/**
 * @brief Emu_Clear - Clear NFC-A emulation data
 * 
 * Clears and invalidates the NFC-A emulation data in a thread-safe manner.
 * 
 * @retval None
 */
/*============================================================================*/
void Emu_Clear(void)
{
    taskENTER_CRITICAL();
    memset(&g_emuA, 0, sizeof(g_emuA));
    taskEXIT_CRITICAL();
}

/*============================================================================*/
/**
 * @brief apply_poller_role - Apply poller role configuration
 * 
 * Sets the NFC role function pointers to poller (reader) mode functions.
 * 
 * @retval None
 */
/*============================================================================*/
static inline void apply_poller_role(void)
{
    //NfcRole.title            = "NFC Poller Role";
    NfcRole.nfc_init_func    = NFC_Polling_Init;
    NfcRole.nfc_process_func = NFC_Polling_Process;
    NfcRole.nfc_deinit_func  = NFC_Polling_DeInit;
    NfcRole.tick_ms          = 0;
    s_currentRole            = NFC_ROLE_POLLER;
}

/*============================================================================*/
/**
 * @brief apply_listener_role - Apply listener role configuration
 * 
 * Sets the NFC role function pointers to listener (emulation) mode functions.
 * 
 * @retval None
 */
/*============================================================================*/
static inline void apply_listener_role(void)
{
    //NfcRole.title            = "NFC Listener Role";
    NfcRole.nfc_init_func    = NFC_Listening_Init;
    NfcRole.nfc_process_func = NFC_Listening_Process;
    NfcRole.nfc_deinit_func  = NFC_Listening_DeInit;
    NfcRole.tick_ms          = 0;
    s_currentRole            = NFC_ROLE_LISTENER;
}

static inline void apply_picopass_listener_role(void)
{
    NfcRole.nfc_init_func    = NFC_PicoPass_Listening_Init;
    NfcRole.nfc_process_func = NFC_PicoPass_Listening_Process;
    NfcRole.nfc_deinit_func  = NFC_PicoPass_Listening_DeInit;
    NfcRole.tick_ms          = 0;
    s_currentRole            = NFC_ROLE_PICOPASS_LISTENER;
}

/*============================================================================*/
/**
 * @brief NFC_SetRole - Set NFC role (simple pointer replacement)
 * 
 * Sets the NFC role by replacing function pointers only.
 * Does not perform initialization or cleanup.
 * 
 * @param[in] role NFC role to set (NFC_ROLE_POLLER or NFC_ROLE_LISTENER)
 * @retval true If role was set successfully
 * @retval false If invalid role specified
 */
/*============================================================================*/
bool NFC_SetRole(NfcRole_e role)
{
    switch (role) {
        case NFC_ROLE_POLLER:              apply_poller_role();              return true;
        case NFC_ROLE_LISTENER:            apply_listener_role();            return true;
        case NFC_ROLE_PICOPASS_LISTENER:   apply_picopass_listener_role();   return true;
        default: return false;
    }
}

/*============================================================================*/
/**
 * @brief NFC_SwitchRole - Switch NFC role with cleanup and initialization
 * 
 * Switches the NFC role by deinitializing the current role, setting new role,
 * and optionally initializing the new role. If the role is already set, no action is taken.
 * 
 * @param[in] role NFC role to switch to (NFC_ROLE_POLLER or NFC_ROLE_LISTENER)
 * @retval true If role switch was successful
 * @retval false If role switch failed (rolls back to poller role)
 */
/*============================================================================*/
bool NFC_SwitchRole(NfcRole_e role)
{
    if (s_currentRole == role) {
        // Already in the same role, no need to switch
        //platformLog("No Need Switching Role\r\n");
        return true;
    }

    // 1) Cleanup existing role
    if (NfcRole.nfc_deinit_func) {
        NfcRole.nfc_deinit_func();
    }

    // 2) Reset function pointers
    if (!NFC_SetRole(role)) {
        // If pointer restoration fails, safely rollback to poller role
        apply_poller_role();
        return false;
    }

    // 3) Initialize new role
    if (NfcRole.nfc_init_func) {
        NfcRole.nfc_init_func();
        return true;
    }
    return true;
}



/*============================================================================*/
/**
 * @brief nfc_worker_task - NFC worker task main function
 * 
 * FreeRTOS task that manages NFC operations through a state machine.
 * Handles role switching (Poller/Listener), initialization, processing, and cleanup.
 * Receives events via queue (nfc_worker_q_hdl) to control NFC operations.
 * 
 * State machine flow:
 * - NFC_STATE_WAIT: Waits for events (Q_EVENT_NFC_START_READ or Q_EVENT_NFC_START_EMULATE)
 * - NFC_STATE_INITIALIZE: Initializes NFC role
 * - NFC_STATE_PROCESS: Processes NFC operations (reading/emulating)
 * - NFC_STATE_DONE: Deinitializes and returns to WAIT state
 * 
 * @param[in] arg Task argument (unused)
 * @retval None (infinite loop)
 */
/*============================================================================*/
void nfc_worker_task(void *arg)
{
    S_M1_Main_Q_t q_item;
	BaseType_t ret;

	platformLog("NFC Worker Task Started!\r\n");
    nfc_ctx_module_init(); // NFC context initialization

	for(;;)
	{
        m1_wdt_reset();
        switch (NfcState)
        {
            case NFC_STATE_WAIT:
                ret = xQueueReceive(nfc_worker_q_hdl, &q_item, pdMS_TO_TICKS(100));//portMAX_DELAY);
                if (ret==pdTRUE)
                {
                    if ( q_item.q_evt_type==Q_EVENT_NFC_START_READ )
                    {
                        NfcState = NFC_STATE_INITIALIZE;
                        //platformLog("NFC Worker Task: Received NFC_START_READ event, moving to Init state\r\n");

                        (void)NFC_SwitchRole(NFC_ROLE_POLLER); // Switch to poller role

                    }
                    else if ( q_item.q_evt_type==Q_EVENT_NFC_START_EMULATE )
                    {
                        NfcState = NFC_STATE_INITIALIZE;
                        //platformLog("NFC Worker Task: Received NFC_START_READ event, moving to Init state\r\n");

                        (void)NFC_SwitchRole(NFC_ROLE_LISTENER); // Switch to listener (emulation) role

                    }
                }
                break;

            case NFC_STATE_INITIALIZE:
                //platformLog("NFC Worker Task: rfal_init, moving to Process state\r\n");
                NfcRole.nfc_init_func();
                NfcState = NFC_STATE_PROCESS;
                vTaskDelay(5);
                break;

            case NFC_STATE_PROCESS:
                //platformLog("NFC Worker Task: Processing\r\n");
                m1_wdt_reset();
                NfcRole.nfc_process_func();
                ret = xQueueReceive(nfc_worker_q_hdl, &q_item, 0);//worker tick delay & Q wait
                if (ret==pdTRUE)
                {
                    if ( q_item.q_evt_type==Q_EVENT_NFC_READ_COMPLETE )
                    {
                        NfcState = NFC_STATE_DONE;
                        m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
                        //platformLog("NFC Worker Task: Process Complete, moving to Done state\r\n");
                    }
                    else if ( q_item.q_evt_type==Q_EVENT_NFC_EMULATE_STOP )
                    {
                        NfcState = NFC_STATE_DONE;
                        m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
                        //platformLog("NFC Worker Task: Process Complete, moving to Done state\r\n");
                    }                    
                    
                }
                vTaskDelay(5);
                break;

            case NFC_STATE_DONE:
                NfcRole.nfc_deinit_func();
                HAL_GPIO_WritePin(EN_EXT_5V_GPIO_Port, EN_EXT_5V_Pin, GPIO_PIN_RESET);
                NfcState = NFC_STATE_WAIT;
                //platformLog("NFC Worker Task: De-init NFC, state initialize\r\n");
                break;

            default:
		        ret = xQueueReceive(nfc_worker_q_hdl, &q_item, portMAX_DELAY);
                if (ret==pdTRUE)
                {
                    platformLog("default\r\n");
                }
                break;
        } // switch (NfcState)
        //xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	}
}

