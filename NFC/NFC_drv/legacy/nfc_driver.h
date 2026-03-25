/* See COPYING.txt for license details. */

#include <stdint.h>
#include "common/nfc_ctx.h"
#include "logger.h"


#define FUNCTION_TITLE_LEN    64


typedef enum
{
    NFC_STATE_INITIALIZE = 0,
    NFC_STATE_PROCESS,
    NFC_STATE_DONE,
    NFC_STATE_DEINITIALIZE,
    NFC_STATE_WAIT
}NfcStateMachine;

typedef struct S_M1_NfcFunc
{
	//char title[FUNCTION_TITLE_LEN];
    void (*nfc_init_func)(void); 
    void (*nfc_process_func)(void); 
    void (*nfc_deinit_func)(void); 

    uint32_t tick_ms;   //worker loop tick
} S_M1_NfcFunc_t;

typedef enum {
    NFC_ROLE_POLLER = 0,
    NFC_ROLE_LISTENER,
    NFC_ROLE_PICOPASS_LISTENER
} NfcRole_e;


typedef struct {
    uint8_t uid[10];
    uint8_t uid_len;     // 4 or 7
    uint8_t atqa[2];     // SENS_RES(ATQA)
    uint8_t sak;         // SEL_RES(SAK)
    bool    valid;
} EmuNfcA_t;



extern EmuNfcA_t g_emuA;

extern S_M1_NfcFunc_t NfcRole;

extern TaskHandle_t nfc_worker_task_hdl;
extern QueueHandle_t nfc_worker_q_hdl;


/**
 * @brief Emu_SetNfcA - Store NFC-A data from poller
 * 
 * Stores NFC-A emulation data (UID, ATQA, SAK) from poller reading.
 * 
 * @param[in] uid Pointer to UID data
 * @param[in] uid_len UID length
 * @param[in] atqa0 ATQA byte 0
 * @param[in] atqa1 ATQA byte 1
 * @param[in] sak SAK byte
 * @retval None
 */
void Emu_SetNfcA(const uint8_t* uid, uint8_t uid_len,
                 uint8_t atqa0, uint8_t atqa1, uint8_t sak);

/**
 * @brief Emu_GetNfcA - Get NFC-A data for listener (returns true if available)
 * 
 * Retrieves NFC-A emulation data for listener mode.
 * 
 * @param[out] out Pointer to output structure
 * @retval true If data is available
 * @retval false If data is not available
 */
bool Emu_GetNfcA(EmuNfcA_t* out);

/**
 * @brief Emu_Clear - Clear (invalidate) emulation data
 * 
 * Clears and invalidates the NFC-A emulation data.
 * 
 * @retval None
 */
void Emu_Clear(void);

/**
 * @brief nfc_worker_task - NFC worker task main function
 * 
 * @param[in] arg Task argument (unused)
 * @retval None
 */
void nfc_worker_task(void *arg);

/**
 * @brief NFC_SetRole - Set NFC role (simple pointer replacement only)
 * 
 * @param[in] role NFC role to set
 * @retval true If successful
 * @retval false If failed
 */
bool NFC_SetRole(NfcRole_e role);

/**
 * @brief NFC_SwitchRole - Switch NFC role with cleanup and initialization
 * 
 * @param[in] role NFC role to switch to
 * @retval true If successful
 * @retval false If failed
 */
bool NFC_SwitchRole(NfcRole_e role);

/**
 * @brief nfc_run_ctx_init - Initialize NFC context
 * 
 * @param[in] c Pointer to NFC context structure
 * @retval None
 */
void nfc_run_ctx_init(nfc_run_ctx_t* c);
