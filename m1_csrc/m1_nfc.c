/* See COPYING.txt for license details. */

/*************************** I N C L U D E S **********************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_nfc.h"
#include "m1_storage.h"
#include "m1_sdcard.h"
#include "m1_virtual_kb.h"
#include "app_x-cube-nfcx.h"
#include "uiView.h"
#include "m1_tasks.h"
#include "logger.h"
#include "legacy/nfc_driver.h"
#include "legacy/nfc_listener.h"
#include "legacy/nfc_poller.h"
#include "common/nfc_storage.h"
#include "common/nfc_ctx.h"
#include "m1_file_browser.h"
#include "m1_file_util.h"
#include "privateprofilestring.h"
#include "common/nfc_file.h"
#include "res_string.h"
#include "rfal_t2t.h"

/*************************** D E F I N E S ************************************/
#define M1_LOGDB_TAG					"NFC"

#define NFC_READ_MORE_OPTIONS			4
#define NFC_READ_MORE_OPTIONS_FILE    	6 // Emulate, Edit UID, Utils, Info, Rename, Delete

#define NFC_FILEPATH					"/NFC"
#define NFC_FILE_EXTENSION				".nfc"
#define NFC_FILE_EXTENSION_TMP			"nfc"  // For IsValidFileSpec (without dot)

#define CONCAT_FILEPATH_FILENAME(fpath, fname) fpath fname

#define NFC_WORKER_TASK_PRIORITY   		(tskIDLE_PRIORITY + 1)

#define NFC_INFO_LINES_PER_SCREEN   	5

//#define SEE_DUMP_MEMORY //READ or Load file dump memory view
/************************** C O N S T A N T **********************************/
/* Menu for LIVE_CARD (including Save) */
const char *m1_nfc_more_options[] = {
		"Save",
		"Emulate UID",
		"Utils",
		"Info"
};

/* Menu for LOAD_FILE (Excluding Save - does not require Save as it was retrieved from a file) */
const char *m1_nfc_more_options_file[] = {
		"Emulate UID",
		"Edit UID",
		"Utils",
		"Info",
		"Rename",
		"Delete"
};

/* Menu for NFC Tools (both top-level nfc_tools() and Utils view) */
#define NFC_TOOL_OPTIONS_COUNT  6
static const char *m1_nfc_tool_options[] = {
	"Tag Info",
	"Clone Emulate",
	"NFC Fuzzer",
	"Write UID",
	"Wipe Tag",
	"Unlock Read"
};

//************************** S T R U C T U R E S *******************************

typedef enum
{
	NFC_READ_DISPLAY_PARAM_READING_READY = 0,
	NFC_READ_DISPLAY_PARAM_READING_COMPLETE,
	NFC_READ_DISPLAY_PARAM_READING_EOL
} S_M1_nfc_read_display_mode_t;

enum {
    VIEW_MODE_NFC_NONE = 0,
    VIEW_MODE_NFC_READ,
    VIEW_MODE_NFC_READ_MORE,
    VIEW_MODE_NFC_SAVE,     //sub menu option
    VIEW_MODE_NFC_EMULATE,
    VIEW_MODE_NFC_UTILS,
	VIEW_MODE_NFC_INFO,
	VIEW_MODE_NFC_EDIT_UID, // Edit UID for loaded file
	VIEW_MODE_NFC_RENAME,   // Rename file for loaded file
	VIEW_MODE_NFC_SAVED_BROWSE, // Browse and load saved NFC file
    VIEW_MODE_NFC_END
};

typedef enum {
    NFC_MODE_NONE=0,
    NFC_MODE_READ,
    NFC_MODE_EMULATE,
    NFC_MODE_WRITE //utils mode
} nfc_mode_t;

typedef enum {
	NFC_RECORD_IDLE = 0,
	NFC_RECORD_ACTIVE,
	NFC_RECORD_STANDBY,
	NFC_RECORD_REPLAY,
	NFC_RECORD_UNKNOWN
} S_M1_NFC_Record_t;

/***************************** V A R I A B L E S ******************************/

static uint16_t s_page_scroll = 0;
static uint16_t s_info_mode   = 0;
static bool s_edit_uid_started = false;  // Edit UID 시작 플래그
static uint8_t nfc_uiview_gui_latest_param;
static S_M1_NFC_Record_t record_stat;
//static FIL nfc_file;
//static DIR nfc_dir;
static S_M1_file_info *f_info = NULL;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/
void nfc_read(void);
void nfc_tools(void);
void nfc_saved(void);
static void nfc_tool_unlock_read(void);
static uint8_t nfc_read_more_options_save(void);
static uint8_t nfc_read_more_options_delete(void);
void m1_nfc_info_more_draw(void);

/* For each mode init/create/update/destroy/message prototype */
static void nfc_read_gui_init(void);
static void nfc_read_gui_create(uint8_t param);
static void nfc_read_gui_destroy(uint8_t param);
static void nfc_read_gui_update(uint8_t param);
static int  nfc_read_gui_message(void);
static int nfc_read_kp_handler(void);

static void nfc_read_more_gui_init(void);
static void nfc_read_more_gui_create(uint8_t param);
static void nfc_read_more_gui_destroy(uint8_t param);
static void nfc_read_more_gui_update(uint8_t param);
static int  nfc_read_more_gui_message(void);
static int nfc_read_more_kp_handler(void);

static void nfc_save_gui_init(void);
static void nfc_save_gui_create(uint8_t param);
static void nfc_save_gui_destroy(uint8_t param);
static void nfc_save_gui_update(uint8_t param);
static int  nfc_save_gui_message(void);
static int nfc_save_kp_handler(void);

static void nfc_emulate_gui_init(void);
static void nfc_emulate_gui_create(uint8_t param);
static void nfc_emulate_gui_destroy(uint8_t param);
static void nfc_emulate_gui_update(uint8_t param);
static int  nfc_emulate_gui_message(void);
static int nfc_emulate_kp_handler(void);

static void nfc_utils_gui_init(void);
static void nfc_utils_gui_create(uint8_t param);
static void nfc_utils_gui_destroy(uint8_t param);
static void nfc_utils_gui_update(uint8_t param);
static int  nfc_utils_gui_message(void);
static int nfc_utils_kp_handler(void);

/* Forward declarations for NFC tools (used by nfc_utils view and nfc_tools) */
static const char* nfc_tool_manufacturer_name(uint8_t mfr_byte);
static const char* nfc_tool_sak_meaning(uint8_t sak, const uint8_t atqa[2]);
static void nfc_tool_fuzzer(void);

static void nfc_info_gui_init(void);
static void nfc_info_gui_create(uint8_t param);
static void nfc_info_gui_destroy(uint8_t param);
static void nfc_info_gui_update(uint8_t param);
static int  nfc_info_gui_message(void);
static int nfc_info_kp_handler(void);
static void nfc_info_drawing(void);

static void nfc_edit_uid_gui_init(void);
static void nfc_edit_uid_gui_create(uint8_t param);
static void nfc_edit_uid_gui_destroy(uint8_t param);
static void nfc_edit_uid_gui_update(uint8_t param);
static int  nfc_edit_uid_gui_message(void);
static int nfc_edit_uid_kp_handler(void);

static void nfc_rename_gui_init(void);
static void nfc_rename_gui_create(uint8_t param);
static void nfc_rename_gui_destroy(uint8_t param);
static void nfc_rename_gui_update(uint8_t param);
static int  nfc_rename_gui_message(void);
static int nfc_rename_kp_handler(void);

static void nfc_saved_browse_gui_init(void);
static void nfc_saved_browse_gui_create(uint8_t param);
static void nfc_saved_browse_gui_destroy(uint8_t param);
static void nfc_saved_browse_gui_update(uint8_t param);
static int  nfc_saved_browse_gui_message(void);
static int nfc_saved_browse_kp_handler(void);

/*============================================================================*/
/*                            table of ui view                                */
/*============================================================================*/
static const view_func_t view_nfc_read_table[] = {
    NULL,               // Empty
    nfc_read_gui_init,      // VIEW_MODE_NFC_READ
    nfc_read_more_gui_init,   // VIEW_MODE_NFC_READ_MORE
    nfc_save_gui_init,      // VIEW_MODE_NFC_SAVE
    nfc_emulate_gui_init,   // VIEW_MODE_NFC_EMULATE
    nfc_utils_gui_init,     // VIEW_MODE_NFC_UTILS
    nfc_info_gui_init,      // VIEW_MODE_NFC_INFO
    nfc_edit_uid_gui_init,  // VIEW_MODE_NFC_EDIT_UID
    nfc_rename_gui_init,    // VIEW_MODE_NFC_RENAME
    nfc_saved_browse_gui_init, // VIEW_MODE_NFC_SAVED_BROWSE
};
#if 0
static const view_func_t view_nfc_tools_table[] = {
    NULL,               // Empty

};

static const view_func_t view_nfc_saved_table[] = {
    NULL,               // Empty

};
#endif

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
 * @brief menu_nfc_init - Initialize NFC menu and create worker task
 * 
 * This function initializes the NFC sub-menu by creating the NFC worker task
 * and message queue. It prevents duplicate initialization by checking if
 * the task and queue handles are already created.
 * 
 * @note This function checks free heap before and after task creation
 *       and logs a warning if heap is low.
 * 
 * @retval None
 */
/*============================================================================*/
void menu_nfc_init(void)
{
    platformLog("menu_nfc_init and Task Create\r\n");

    /* Do not recreate if already created (prevent duplication) */
    if (nfc_worker_task_hdl != NULL || nfc_worker_q_hdl != NULL)
    {
        platformLog("[NFC] menu_nfc_init: already initialized. task=%p, q=%p\r\n",
                    nfc_worker_task_hdl, nfc_worker_q_hdl);
        return;
    }

    BaseType_t ret;
    size_t free_heap_before = xPortGetFreeHeapSize();
    platformLog("[NFC] free heap before create: %u bytes\r\n", (unsigned)free_heap_before);
	osDelay(100);

    ret = xTaskCreate(nfc_worker_task,
                      "nfc_worker",
                      M1_TASK_STACK_SIZE_4096,
                      NULL,
                      NFC_WORKER_TASK_PRIORITY, //TASK_PRIORITY_SUBFUNC_HANDLER + 1,
                      &nfc_worker_task_hdl);

    if (ret != pdPASS || nfc_worker_task_hdl==NULL)
    {
        platformLog("[NFC] nfc_worker_task create failed! ret=%d, hdl=%p\r\n",
                    (int)ret, nfc_worker_task_hdl);
        nfc_worker_task_hdl = NULL;
        return;    
    }
	platformLog("[NFC] xTaskCreate ret=%ld, handle=%p\r\n",
            (long)ret, nfc_worker_task_hdl);

    
    nfc_worker_q_hdl = xQueueCreate(10, sizeof(S_M1_Main_Q_t));
    if (nfc_worker_q_hdl==NULL)
    {
        platformLog("[NFC] nfc_worker_q create failed! delete task and return\r\n");
        vTaskDelete(nfc_worker_task_hdl);
        nfc_worker_task_hdl = NULL;
        return;
    }

    size_t free_heap_after = xPortGetFreeHeapSize();
    platformLog("[NFC] free heap after create: %u bytes\r\n", (unsigned)free_heap_after);

    if (free_heap_after < M1_LOW_FREE_HEAP_WARNING_SIZE)
    {
        platformLog("[NFC][WARN] free heap is low! %u < %u\r\n",
                    (unsigned)free_heap_after,
                    (unsigned)M1_LOW_FREE_HEAP_WARNING_SIZE);
    }
}

/*============================================================================*/
/**
 * @brief menu_nfc_deinit - Deinitialize NFC menu and destroy worker task
 * 
 * This function cleans up the NFC sub-menu by deleting the NFC worker task
 * and message queue. It safely handles NULL handles.
 * 
 * @retval None
 */
 /*============================================================================*/
void menu_nfc_deinit(void)
{
    platformLog("menu_nfc_deinit and Task Destroy\r\n");

    if (nfc_worker_task_hdl != NULL)
    {
        platformLog("[NFC] delete worker task\r\n");
        vTaskDelete(nfc_worker_task_hdl);
        nfc_worker_task_hdl = NULL;
    }

    if (nfc_worker_q_hdl != NULL)
    {
        platformLog("[NFC] delete worker queue\r\n");
        vQueueDelete(nfc_worker_q_hdl);
        nfc_worker_q_hdl = NULL;
    }
}



/*============================================================================*/
/**
 * @brief nfc_read - Main NFC read function
 * 
 * This function handles the NFC card reading workflow. It registers
 * the NFC read view table, switches to the read view mode, and enters
 * a message loop until the view exits.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_read(void)
{
	platformLog("nfc_read()\r\n");
	m1_gui_submenu_update(NULL, 0, 0, X_MENU_UPDATE_INIT);
	nfc_uiview_gui_latest_param = 0xFF; // Initialize with an invalid parameter
	// init
	m1_uiView_functions_init(VIEW_MODE_NFC_END, view_nfc_read_table);
	m1_uiView_display_switch(VIEW_MODE_NFC_READ, NFC_READ_DISPLAY_PARAM_READING_READY);

	// loop
	while( m1_uiView_q_message_process() )
	{
		;
	}
	platformLog("nfc_read()-exit\r\n");
}

/*============================================================================*/
/**
 * @brief nfc_read_kp_handler - Handle keypad input for NFC read view
 * 
 * Processes button events in the NFC read view:
 * - BACK: Exit to idle view
 * - LEFT: Retry reading (restart read process)
 * - RIGHT: Switch to submenu view
 * 
 * @retval 0 Exit requested (BACK button)
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_read_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0); 
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) 		// exit, return
		{
			platformLog("nfc_read_kp_handler[BUTTON_BACK_KP_ID]\r\n");
			m1_uiView_display_switch(VIEW_MODE_IDLE, 0);
			xQueueReset(main_q_hdl); // Reset main q before return
			return 0;
			//break; // Exit and return to the calling task (subfunc_handler_task)
		} // if ( m1_buttons_status[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )	// retry
		{
			if ( nfc_uiview_gui_latest_param==NFC_READ_DISPLAY_PARAM_READING_COMPLETE )
			{
				platformLog("nfc_read_kp_handler[BUTTON_LEFT_KP_ID]\r\n");
				m1_uiView_display_switch(VIEW_MODE_NFC_READ, NFC_READ_DISPLAY_PARAM_READING_READY);
			} // if ( nfc_uiview_gui_latest_param==NFC_READ_DISPLAY_PARAM_READING_COMPLETE )
		}
		else if(this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK )	// more
		{
			if ( nfc_uiview_gui_latest_param==NFC_READ_DISPLAY_PARAM_READING_COMPLETE )
			{
				platformLog("nfc_read_kp_handler[BUTTON_RIGHT_KP_ID]\r\n");
				m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_RESET);
				nfc_uiview_gui_latest_param = X_MENU_UPDATE_RESET; // Update latest param
			} // if ( nfc_uiview_gui_latest_param==NFC_READ_DISPLAY_PARAM_READING_COMPLETE )
		} // else if(this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK )
	}

	return 1;
}


/*============================================================================*/
/**
 * @brief nfc_read_gui_create - Create and initialize NFC read view
 * 
 * Initializes the NFC read view. If param is 0, starts the NFC reading
 * process by sending a start read event to the worker queue and enabling
 * LED blink indication.
 * 
 * @param[in] param View parameter (0 = start reading, other = update only)
 * @retval None
 */
/*============================================================================*/
static void nfc_read_gui_create(uint8_t param)
{
	platformLog("nfc_read_gui_create param[%d]\r\n", param);
	if( param==NFC_READ_DISPLAY_PARAM_READING_READY )
	{
		record_stat = NFC_RECORD_IDLE;
		m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
		m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_START_READ);
		vTaskDelay(50);
	}

	m1_uiView_display_update(param);
}

/*============================================================================*/
/**
 * @brief nfc_read_gui_destroy - Destroy NFC read view and cleanup resources
 * 
 * Cleans up the NFC read view by turning off LED blink indication
 * and sending a read completion event to the worker queue.
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_read_gui_destroy(uint8_t param)
{
	platformLog("nfc_read_gui_destroy param[%d]\r\n", param);
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_READ_COMPLETE);
}


/*============================================================================*/
 /* @brief nfc_read_gui_update - Update NFC read view display
 * 
 * Updates the display based on the read state:
 * - param 0: Shows "Reading" screen with instructions
 * - param 1: Shows read complete screen with card information (Type, Family, UID)
 * 
 * @param[in] param View update parameter (0 = reading, 1 = read done)
 * @retval None
 */
/*============================================================================*/
static void nfc_read_gui_update(uint8_t param)
{
    if ( nfc_uiview_gui_latest_param==X_MENU_UPDATE_RESET )
    {
    	m1_gui_submenu_update(NULL, 0, 0, X_MENU_UPDATE_RESTORE);
    }
    nfc_uiview_gui_latest_param = param; // Update new param

    /* Graphic work starts here */
    u8g2_FirstPage(&m1_u8g2); // This call required for page drawing in mode 1

    if( param==NFC_READ_DISPLAY_PARAM_READING_READY )	// reading
    {
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_DrawXBMP(&m1_u8g2, 0, 0, 48, 48, nfc_read_48x48);
		u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
		u8g2_DrawStr(&m1_u8g2, 70, 20, "Reading");
		u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 50, 30, "Hold card next");
		u8g2_DrawStr(&m1_u8g2, 50, 40, "to M1's back");
    }
    else if( param==NFC_READ_DISPLAY_PARAM_READING_COMPLETE )	// read done
    {
		platformLog("NFC Read Done UI Display\r\n");
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
		u8g2_DrawStr(&m1_u8g2, 2, 12, NFC_Type);
		u8g2_DrawStr(&m1_u8g2, 2, 22, NFC_Family);
		u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 2, 32, "UID:");
		u8g2_DrawStr(&m1_u8g2, 25, 32, NFC_UID);

		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
		u8g2_DrawXBMP(&m1_u8g2, 1, 53, 8, 8, arrowleft_8x8); // draw arrowleft icon
		u8g2_DrawStr(&m1_u8g2, 11, 61, "Retry");
		u8g2_DrawXBMP(&m1_u8g2, 119, 53, 8, 8, arrowright_8x8); // draw arrowright icon
		u8g2_DrawStr(&m1_u8g2, 97, 61, "More");
#ifdef SEE_DUMP_MEMORY
        m1_wdt_reset();
		nfc_run_ctx_t * c = nfc_ctx_get(); //DBG NFC Context(UID) Data 
		platformLog("nfc_ctx[source_kind]:%d\r\n",c->file.source_kind); //LIVE_CARD   0   LOAD_FILE   1
		platformLog("nfc_ctx[tech]:%d\r\n",c->head.tech); //enum NFC_TX_A 0, NFC_TX_B 1, NFC_TX_F 2, NFC_TX_V 3
		platformLog("nfc_ctx[uid_len]:%d\r\n",c->head.uid_len);
		platformLog("nfc_ctx[uid]: %s\r\n", hex2Str(c->head.uid, c->head.uid_len));
		platformLog("nfc_ctx[atqa0]:%02X\r\n",c->head.a.atqa[0]);
		platformLog("nfc_ctx[atqa1]:%02X\r\n",c->head.a.atqa[1]);
		platformLog("nfc_ctx[sak]:%02X\r\n",c->head.a.sak);
		platformLog("nfc_ctx[family]:%d\r\n",c->head.family);
		platformLog("nfc_ctx[unit_size]:%d\r\n",c->dump.unit_size);
		platformLog("nfc_ctx[unit_count]:%d\r\n",c->dump.unit_count);
		platformLog("nfc_ctx[has_dump]:%d\r\n",c->dump.has_dump);
		platformLog("nfc_ctx[max_seen_unit]:%lu\r\n",c->dump.max_seen_unit);
		nfc_ctx_dump_t2t_pages();
#endif
    }
	m1_u8g2_nextpage(); // Update display RAM
}


/*============================================================================*/
 /* @brief nfc_read_gui_message - Process messages for NFC read view
 * 
 * Handles messages from the main queue:
 * - Q_EVENT_KEYPAD: Processes button events
 * - Q_EVENT_NFC_READ_COMPLETE: Updates view to show read complete state
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_read_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_read_kp_handler();
		} 
		else if ( q_item.q_evt_type==Q_EVENT_NFC_READ_COMPLETE )
		{
			// Do other things for this task
			record_stat = NFC_RECORD_ACTIVE;
			m1_buzzer_notification();
			m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
			m1_uiView_display_update(NFC_READ_DISPLAY_PARAM_READING_COMPLETE);
		} 
	} // if (ret==pdTRUE)

	return ret_val;
} // static int nfc_read_gui_message(void)



/*============================================================================*/
 /* @brief nfc_read_gui_init - Initialize and register NFC read view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC read view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_read_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_READ, nfc_read_gui_create, nfc_read_gui_update, nfc_read_gui_destroy, nfc_read_gui_message);
}

/*============================================================================*/
/**
 * @brief nfc_read_more_kp_handler - Handle keypad input for NFC submenu view
 * 
 * Processes button events in the NFC submenu view. The menu options
 * differ based on whether the card was read live (LIVE_CARD) or loaded
 * from file (LOAD_FILE). For LOAD_FILE, the "Save" option is excluded.
 * 
 * Button actions:
 * - BACK: Exit (returns to read view for LIVE_CARD, full exit for LOAD_FILE)
 * - LEFT: Return to read view (LIVE_CARD only)
 * - OK: Select menu item and switch to corresponding view
 * - UP/DOWN: Navigate menu items
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_read_more_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;
	nfc_run_ctx_t* c = nfc_ctx_get();
	bool is_load_file = (c && c->file.source_kind==LOAD_FILE);
	uint8_t menu_index, view_id;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // user wants to exit?
		{
			if (is_load_file)
			{
				// Card from SD → SUBMENU requested full termination
				platformLog("LOAD_FILE exit (submenu)\r\n");
				return 0; // Return to previous menu
			}
			else
			{
				// Card read live → Return to the Read Complete Screen
				platformLog("LIVE_CARD exit (submenu)\r\n");
				m1_uiView_display_switch(VIEW_MODE_NFC_READ, NFC_READ_DISPLAY_PARAM_READING_COMPLETE);
			}
		} // if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if (!is_load_file)
			{
				m1_uiView_display_switch(VIEW_MODE_NFC_READ, NFC_READ_DISPLAY_PARAM_READING_COMPLETE);
			}
		} // else if(this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			menu_index = m1_gui_submenu_update(NULL, 0, 0, MENU_UPDATE_NONE); // Get current index
			if (is_load_file)
			{
				/* LOAD_FILE: Except Save, index mapping */
				view_id = 0xFF;
				switch ( menu_index )
				{
					case 0:
						view_id = VIEW_MODE_NFC_EMULATE;
						break;

					case 1:
						view_id = VIEW_MODE_NFC_EDIT_UID;
						break;

					case 2:
						view_id = VIEW_MODE_NFC_UTILS;
						break;

					case 3:
						view_id = VIEW_MODE_NFC_INFO;
						break;

					case 4:
						view_id = VIEW_MODE_NFC_RENAME;
						break;

					case 5: // Delete
						if (nfc_read_more_options_delete()==0)
						{
							return 0; // exit (파일 삭제됨)
						}
						m1_uiView_display_update(X_MENU_UPDATE_REFRESH);
						break;

					default:
						break;
				} // switch ( menu_index )
				if (view_id != 0xFF)
				{
					m1_uiView_display_switch(view_id, 0);
				}
			} // if (is_load_file)
			else
			{
				/* LIVE_CARD: Including Save, existing index */
				view_id = 0xFF;
				switch ( menu_index )
				{
					case 0:
						view_id = VIEW_MODE_NFC_SAVE;
						break;

					case 1:
						view_id = VIEW_MODE_NFC_EMULATE;
						break;

					case 2:
						view_id = VIEW_MODE_NFC_UTILS;
						break;

					case 3:
						view_id = VIEW_MODE_NFC_INFO;
						break;

					default:
						break;
				} // switch ( menu_index )
				if (view_id != 0xFF)
				{
					m1_uiView_display_switch(view_id, 0);
				}
			} // else
		} // else if(this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
		{
			m1_uiView_display_update(X_MENU_UPDATE_MOVE_UP);

		}
		else if(this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
		{
			m1_uiView_display_update(X_MENU_UPDATE_MOVE_DOWN);
		}
	} // if (ret==pdTRUE)

	return 1;
} // static int nfc_read_more_kp_handler(void)



/*============================================================================*/
 /* @brief nfc_read_more_gui_create - Create and initialize NFC submenu view
 * 
 * Initializes the submenu view and sets the cursor index based on param.
 * If param is out of range, resets cursor to 0.
 * 
 * @param[in] param Initial cursor index (or 0xFF for refresh)
 * @retval None
 */
/*============================================================================*/
static void nfc_read_more_gui_create(uint8_t param)
{
	m1_uiView_display_update(param);
}

/*============================================================================*/
 /* @brief nfc_read_more_gui_destroy - Destroy NFC submenu view
 * 
 * Cleanup function for submenu view (currently empty).
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_read_more_gui_destroy(uint8_t param)
{
	;
}

/*============================================================================*/
 /* @brief nfc_read_more_gui_update - Update NFC submenu view display
 * 
 * Updates the submenu display based on the card source (LIVE_CARD or LOAD_FILE).
 * Uses different menu arrays depending on the source kind and updates
 * the display accordingly.
 * 
 * @param[in] param Update type (0 = reset, 1 = move up, 2 = move down, 0xFF = refresh)
 * @retval None
 */
/*============================================================================*/
static void nfc_read_more_gui_update(uint8_t param)
{
	nfc_run_ctx_t* c = nfc_ctx_get();
	bool is_load_file = (c && c->file.source_kind==LOAD_FILE);

	// Use different menu arrangements depending on source_kind 
	const char **menu_options = is_load_file ? m1_nfc_more_options_file : m1_nfc_more_options;
	uint8_t menu_count = is_load_file ? NFC_READ_MORE_OPTIONS_FILE : NFC_READ_MORE_OPTIONS;

	m1_gui_submenu_update(menu_options, menu_count, 0, param);
}


/*============================================================================*/
 /* @brief nfc_read_more_gui_message - Process messages for NFC submenu view
 * 
 * Handles messages from the main queue, primarily keypad events.
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */	
/*============================================================================*/
static int nfc_read_more_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_read_more_kp_handler();
		} 
		else 
		{

		}
	} 

	return ret_val;
}

/*============================================================================*/
 /* @brief nfc_read_more_gui_init - Initialize and register NFC submenu view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC submenu view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_read_more_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_READ_MORE, nfc_read_more_gui_create, nfc_read_more_gui_update, nfc_read_more_gui_destroy, nfc_read_more_gui_message);
}

/*============================================================================*/
/**
 * @brief nfc_save_kp_handler - Handle keypad input for NFC save view
 * 
 * Processes button events in the NFC save view:
 * - BACK: Return to submenu
 * - LEFT: Return to read view
 * 
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_save_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // user wants to exit?
		{
			// Do extra tasks here if needed
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		} 
		else if(this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )
		{
			//m1_uiView_display_switch(VIEW_MODE_NFC_READ, NFC_READ_DISPLAY_PARAM_READING_COMPLETE);
		}
	}

	return 1;
}

/*============================================================================*/
/**
 * @brief nfc_save_gui_create - Create and initialize NFC save view
 * 
 * Initializes the save view and triggers an update.
 * 
 * @param[in] param View parameter
 * @retval None
 */
/*============================================================================*/
static void nfc_save_gui_create(uint8_t param)
{
	m1_uiView_display_update(param);
}

/*============================================================================*/
/**
 * @brief nfc_save_gui_destroy - Destroy NFC save view
 * 
 * Cleanup function for save view (currently empty).
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_save_gui_destroy(uint8_t param)
{
	;
}


/*============================================================================*/
/**
 * @brief nfc_save_gui_update - Update NFC save view display
 * 
 * Calls the save function and switches back to submenu if save
 * was cancelled by user (return value 3).
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_save_gui_update(uint8_t param)
{
	BaseType_t ret;

	ret = nfc_read_more_options_save();
	if ( ret==3 )
	{
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
	}
}

/*============================================================================*/
/**
 * @brief nfc_save_gui_message - Process messages for NFC save view
 * 
 * Handles messages from the main queue, primarily keypad events.
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_save_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_save_kp_handler();
		} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		else
		{
			; // Do other things for this task
		}
	} 

	return ret_val;
}

/*============================================================================*/
/**
 * @brief nfc_save_gui_init - Initialize and register NFC save view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC save view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_save_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_SAVE, nfc_save_gui_create, nfc_save_gui_update, nfc_save_gui_destroy, nfc_save_gui_message);
}

/*============================================================================*/
/**
 * @brief nfc_emulate_kp_handler - Handle keypad input for NFC emulate view
 * 
 * Processes button events in the NFC emulate view:
 * - BACK: Stop emulation and return to submenu with saved cursor index
 * 
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_emulate_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			// Do extra tasks here if needed
			ListenerRequestStop();
			/* Return submenu to stored index */
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		}
	}

	return 1;
}

/*============================================================================*/
 /* @brief nfc_emulate_gui_create - Create and initialize NFC emulate view
 * 
 * Initializes the emulate view. If param is 0, synchronizes the NFC context
 * to the emulator, enables LED blink, and starts the emulation process.
 * 
 * @param[in] param View parameter (0 = start emulation, other = update only)
 * @retval None
 */
/*============================================================================*/
static void nfc_emulate_gui_create(uint8_t param)
{
	if(param==0)
	{
        nfc_ctx_sync_emu();//Current nfc_ctx.head content reflected in emulator context (g_emuA)
        m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
		m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_START_EMULATE);
		osDelay(50);
	}

    m1_uiView_display_update(param);
}

/*============================================================================*/
 /* @brief nfc_emulate_gui_destroy - Destroy NFC emulate view and cleanup resources
 * 
 * Cleans up the emulate view by turning off LED blink and sending
 * a stop emulation event to the worker queue.
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_emulate_gui_destroy(uint8_t param)
{
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_EMULATE_STOP);
}


/*============================================================================*/
 /* @brief nfc_emulate_gui_update - Update NFC emulate view display
 * 
 * Updates the display to show the emulation status screen with
 * instructions for the user.
 * 
 * Displays "Emulate UID" if only UID is available for emulation,
 * or "Emulate" if Page/Block dump data is available for full emulation.
 * 
 * @param[in] param View update parameter (0 = emulating)
 * @retval None
 */
/*============================================================================*/
static void nfc_emulate_gui_update(uint8_t param)
{
    if ( param==0 )	// emulating
    {
		nfc_run_ctx_t* c = nfc_ctx_get();
		const char* emu_text = "Emulate UID";  // 기본값: UID만 에뮬레이션
		
		// T2T (Ultralight/NTAG) + Page dump 데이터가 있는 경우
		if (c && 
			c->head.family==M1NFC_FAM_ULTRALIGHT &&
			c->dump.has_dump && 
			c->dump.data != NULL && 
			c->dump.unit_size==4 &&
			c->dump.unit_count > 0) {
			emu_text = "Emulate";
		}
		// MFC (Classic) + Block dump 데이터가 있는 경우 (나중을 위해)
		else if (c && 
				 c->head.family==M1NFC_FAM_CLASSIC &&
				 c->dump.has_dump && 
				 c->dump.data != NULL && 
				 c->dump.unit_size==16 &&
				 c->dump.unit_count > 0) {
			emu_text = "Emulate";
		}
		
	    /* Graphic work starts here */
	    u8g2_FirstPage(&m1_u8g2); // This call required for page drawing in mode 1
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_DrawXBMP(&m1_u8g2, 0, 0, 48, 48, nfc_emit_48x48);

		u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
		u8g2_DrawStr(&m1_u8g2, 50, 20, emu_text);  // 약간 왼쪽으로 이동 (70 -> 65)
		u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 50, 30, "Tag M1's Back");
		u8g2_DrawStr(&m1_u8g2, 50, 40, "on the reader");

		u8g2_NextPage(&m1_u8g2); // Update display RAM
    } // if ( param==0 )
}

/*============================================================================*/
 /* @brief nfc_emulate_gui_message - Process messages for NFC emulate view
 * 
 * Handles messages from the main queue, primarily keypad events.
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_emulate_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_emulate_kp_handler();
		}
		else
		{
			; // Do other things for this task
		}
	}

	return ret_val;
}

/*============================================================================*/
 /* @brief nfc_emulate_gui_init - Initialize and register NFC emulate view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC emulate view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_emulate_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_EMULATE, nfc_emulate_gui_create, nfc_emulate_gui_update, nfc_emulate_gui_destroy, nfc_emulate_gui_message);
}


/*============================================================================*/
/**
 * @brief Write UID to a magic/Gen1a T2T card
 */
/*============================================================================*/
static void nfc_utils_write_uid_run(void)
{
	nfc_run_ctx_t *c = nfc_ctx_get();
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;

	if (!c || c->head.uid_len == 0)
	{
		u8g2_FirstPage(&m1_u8g2);
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
		u8g2_DrawStr(&m1_u8g2, 4, 25, "No UID data");
		u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 4, 40, "Read a card first");
		m1_u8g2_nextpage();
		vTaskDelay(pdMS_TO_TICKS(2000));
		return;
	}

	char uid_str[32];
	snprintf(uid_str, sizeof(uid_str), "%s", hex2Str(c->head.uid, c->head.uid_len));

	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 4, 12, "Write UID");
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
	u8g2_DrawStr(&m1_u8g2, 4, 26, uid_str);
	u8g2_DrawStr(&m1_u8g2, 4, 40, "Hold target to back");
	u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	u8g2_DrawStr(&m1_u8g2, 2, 61, "OK=Write  Back=Cancel");
	m1_u8g2_nextpage();

	uint8_t confirmed = 0;
	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;
		if (q_item.q_evt_type != Q_EVENT_KEYPAD) continue;

		ret = xQueueReceive(button_events_q_hdl, &btn, 0);
		if (ret != pdTRUE) continue;

		if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			return;

		if (btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
		{
			if (!confirmed)
			{
				confirmed = 1;
				u8g2_FirstPage(&m1_u8g2);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
				u8g2_DrawStr(&m1_u8g2, 4, 25, "Writing UID...");
				u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
				u8g2_DrawStr(&m1_u8g2, 4, 40, "Hold card steady");
				m1_u8g2_nextpage();

				ReturnCode err = rfalT2TPollerWrite(0, c->head.uid);
				if (err == RFAL_ERR_NONE && c->head.uid_len > 4)
				{
					err = rfalT2TPollerWrite(1, &c->head.uid[4]);
				}

				u8g2_FirstPage(&m1_u8g2);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
				if (err == RFAL_ERR_NONE) {
					u8g2_DrawStr(&m1_u8g2, 4, 25, "Write Success!");
					m1_buzzer_notification();
				} else {
					u8g2_DrawStr(&m1_u8g2, 4, 25, "Write Failed!");
					u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
					u8g2_DrawStr(&m1_u8g2, 4, 40, "Use Magic/Gen1a card");
				}
				m1_u8g2_nextpage();
				vTaskDelay(pdMS_TO_TICKS(2000));
				return;
			}
		}
	}
}

/*============================================================================*/
/**
 * @brief Wipe all user pages on a T2T tag (zeros pages 4 through end)
 */
/*============================================================================*/
static void nfc_utils_wipe_tag_run(void)
{
	S_M1_Buttons_Status btn;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t confirmed = 0;

	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 4, 12, "Wipe Tag");
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
	u8g2_DrawStr(&m1_u8g2, 4, 26, "Zeros all user pages");
	u8g2_DrawStr(&m1_u8g2, 4, 38, "Hold tag to back");
	u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	u8g2_DrawStr(&m1_u8g2, 2, 61, "OK=Wipe  Back=Cancel");
	m1_u8g2_nextpage();

	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;
		if (q_item.q_evt_type != Q_EVENT_KEYPAD) continue;

		ret = xQueueReceive(button_events_q_hdl, &btn, 0);
		if (ret != pdTRUE) continue;

		if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			return;

		if (btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
		{
			if (!confirmed)
			{
				confirmed = 1;
				u8g2_FirstPage(&m1_u8g2);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
				u8g2_DrawStr(&m1_u8g2, 4, 12, "Are you sure?");
				u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
				u8g2_DrawStr(&m1_u8g2, 4, 28, "This will erase all");
				u8g2_DrawStr(&m1_u8g2, 4, 40, "user data on the tag");
				u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
				u8g2_DrawStr(&m1_u8g2, 2, 61, "OK=Confirm Back=Cancel");
				m1_u8g2_nextpage();
			}
			else
			{
				u8g2_FirstPage(&m1_u8g2);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
				u8g2_DrawStr(&m1_u8g2, 4, 25, "Wiping...");
				m1_u8g2_nextpage();

				uint8_t zeros[4] = {0, 0, 0, 0};
				uint16_t pages = nfc_ctx_get_t2t_page_count();
				if (pages == 0) pages = 45;
				uint16_t end_page = pages - 5;
				if (end_page > 200) end_page = 40;

				ReturnCode err = RFAL_ERR_NONE;
				uint16_t wiped = 0;
				for (uint16_t p = 4; p <= end_page; p++)
				{
					err = rfalT2TPollerWrite((uint8_t)p, zeros);
					if (err != RFAL_ERR_NONE) break;
					wiped++;
					m1_wdt_reset();
				}

				u8g2_FirstPage(&m1_u8g2);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
				if (err == RFAL_ERR_NONE) {
					char line[32];
					snprintf(line, sizeof(line), "Wiped %u pages!", wiped);
					u8g2_DrawStr(&m1_u8g2, 4, 25, line);
					m1_buzzer_notification();
				} else {
					char line[32];
					snprintf(line, sizeof(line), "Failed at page %u", wiped + 4);
					u8g2_DrawStr(&m1_u8g2, 4, 25, "Wipe Failed!");
					u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
					u8g2_DrawStr(&m1_u8g2, 4, 40, line);
				}
				m1_u8g2_nextpage();
				vTaskDelay(pdMS_TO_TICKS(2000));
				return;
			}
		}
	}
}

/*============================================================================*/
/**
 * @brief nfc_utils_kp_handler - Handle keypad input for NFC utils view
 *
 * Processes button events in the NFC utils view:
 * - BACK: Return to submenu with saved cursor index
 *
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_utils_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		}
		else if (this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK)
		{
			uint8_t sel = m1_gui_submenu_update(NULL, 0, 0, MENU_UPDATE_NONE);
			switch (sel)
			{
				case 0: /* Tag Info — show enhanced info from current context */
				{
					nfc_run_ctx_t *c = nfc_ctx_get();
					if (c && c->head.uid_len > 0)
					{
						uint16_t info_pg = 0;
						uint16_t pg_scroll = 0;
						bool show_info = true;

						while (1)
						{
							if (show_info)
							{
								show_info = false;
								if (info_pg == 0)
								{
									char line[40];
									u8g2_FirstPage(&m1_u8g2);
									u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
									u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
									u8g2_DrawStr(&m1_u8g2, 2, 10, c->ui.title_text);
									u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
									snprintf(line, sizeof(line), "Mfr: %s (0x%02X)",
									         nfc_tool_manufacturer_name(c->head.uid[0]), c->head.uid[0]);
									u8g2_DrawStr(&m1_u8g2, 2, 20, line);
									snprintf(line, sizeof(line), "UID: %s",
									         hex2Str(c->head.uid, c->head.uid_len));
									u8g2_DrawStr(&m1_u8g2, 2, 30, line);
									if (c->head.a.has_atqa && c->head.a.has_sak)
									{
										snprintf(line, sizeof(line), "ATQA:%02X %02X SAK:%02X",
										         c->head.a.atqa[0], c->head.a.atqa[1], c->head.a.sak);
										u8g2_DrawStr(&m1_u8g2, 2, 40, line);
										snprintf(line, sizeof(line), "Type: %s",
										         nfc_tool_sak_meaning(c->head.a.sak, c->head.a.atqa));
										u8g2_DrawStr(&m1_u8g2, 2, 50, line);
									}
									u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
									u8g2_DrawBox(&m1_u8g2, 0, 54, 128, 10);
									u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
									u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
									u8g2_DrawXBMP(&m1_u8g2, 119, 55, 8, 8, arrowright_8x8);
									u8g2_DrawStr(&m1_u8g2, 97, 62, "More");
									m1_u8g2_nextpage();
								}
								else
								{
									uint16_t total = nfc_ctx_get_t2t_page_count();
									char line[32], header[32];
									u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
									u8g2_FirstPage(&m1_u8g2);
									u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
									if (total == 0) {
										u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
										u8g2_DrawStr(&m1_u8g2, 2, 20, "No page data");
										u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
										u8g2_DrawStr(&m1_u8g2, 2, 32, "Only UID-level info");
									} else {
										uint16_t max_s = (total > NFC_INFO_LINES_PER_SCREEN)
										    ? (total - NFC_INFO_LINES_PER_SCREEN) : 0;
										if (pg_scroll > max_s) pg_scroll = max_s;
										snprintf(header, sizeof(header), "Max %03u  [hex]  | [Asc]",
										         (unsigned)(total - 1));
										u8g2_DrawStr(&m1_u8g2, 2, 8, header);
										u8g2_DrawStr(&m1_u8g2, 2, 16, "------------------------");
										for (uint8_t row = 0; row < NFC_INFO_LINES_PER_SCREEN; row++) {
											uint16_t pg = pg_scroll + row;
											if (pg >= total) break;
											if (!nfc_ctx_format_t2t_page_line(pg, line, sizeof(line)))
												continue;
											u8g2_DrawStr(&m1_u8g2, 2, 24 + row * 8, line);
										}
									}
									m1_u8g2_nextpage();
								}
							}

							S_M1_Main_Q_t qi;
							S_M1_Buttons_Status bs2;
							BaseType_t r2 = xQueueReceive(main_q_hdl, &qi, portMAX_DELAY);
							if (r2 != pdTRUE) continue;
							if (qi.q_evt_type != Q_EVENT_KEYPAD) continue;
							r2 = xQueueReceive(button_events_q_hdl, &bs2, 0);
							if (r2 != pdTRUE) continue;
							if (bs2.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK) break;
							if (bs2.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK && info_pg == 0) {
								info_pg = 1; pg_scroll = 0; show_info = true;
							}
							if (bs2.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK && info_pg == 1) {
								info_pg = 0; show_info = true;
							}
							if (bs2.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK && info_pg == 1 && pg_scroll > 0) {
								pg_scroll--; show_info = true;
							}
							if (bs2.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK && info_pg == 1) {
								uint16_t total = nfc_ctx_get_t2t_page_count();
								uint16_t max_s = (total > NFC_INFO_LINES_PER_SCREEN)
								    ? (total - NFC_INFO_LINES_PER_SCREEN) : 0;
								if (pg_scroll < max_s) { pg_scroll++; show_info = true; }
							}
						}
					}
					/* Refresh utils submenu */
					m1_uiView_display_update(X_MENU_UPDATE_REFRESH);
					break;
				}

				case 1: /* Clone Emulate — emulate the already-read card */
				{
					nfc_run_ctx_t *c = nfc_ctx_get();
					if (c && c->head.uid_len > 0)
					{
						nfc_ctx_sync_emu();
						m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
						m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_START_EMULATE);
						vTaskDelay(50);

						const char *et = "Emulate UID";
						if (c->head.family == M1NFC_FAM_ULTRALIGHT &&
						    c->dump.has_dump && c->dump.data && c->dump.unit_size == 4 && c->dump.unit_count > 0)
							et = "Emulate";
						else if (c->head.family == M1NFC_FAM_CLASSIC &&
						         c->dump.has_dump && c->dump.data && c->dump.unit_size == 16 && c->dump.unit_count > 0)
							et = "Emulate";

						u8g2_FirstPage(&m1_u8g2);
						u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
						u8g2_DrawXBMP(&m1_u8g2, 0, 0, 48, 48, nfc_emit_48x48);
						u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
						u8g2_DrawStr(&m1_u8g2, 50, 15, et);
						u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
						u8g2_DrawStr(&m1_u8g2, 50, 26, "UID:");
						u8g2_DrawStr(&m1_u8g2, 50, 36, hex2Str(c->head.uid, c->head.uid_len));
						u8g2_DrawStr(&m1_u8g2, 50, 46, "Tag M1 on reader");
						m1_u8g2_nextpage();

						while (1) {
							S_M1_Main_Q_t qi;
							S_M1_Buttons_Status bs2;
							BaseType_t r2 = xQueueReceive(main_q_hdl, &qi, portMAX_DELAY);
							if (r2 != pdTRUE) continue;
							if (qi.q_evt_type != Q_EVENT_KEYPAD) continue;
							r2 = xQueueReceive(button_events_q_hdl, &bs2, 0);
							if (r2 == pdTRUE && bs2.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
								break;
						}

						ListenerRequestStop();
						m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
						m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_EMULATE_STOP);
						vTaskDelay(50);
					}
					m1_uiView_display_update(X_MENU_UPDATE_REFRESH);
					break;
				}

				case 2: /* NFC Fuzzer */
					nfc_tool_fuzzer();
					m1_uiView_display_update(X_MENU_UPDATE_REFRESH);
					break;

				case 3: /* Write UID */
					nfc_utils_write_uid_run();
					m1_uiView_display_update(X_MENU_UPDATE_REFRESH);
					break;

				case 4: /* Wipe Tag */
					nfc_utils_wipe_tag_run();
					m1_uiView_display_update(X_MENU_UPDATE_REFRESH);
					break;

				case 5: /* Unlock Read */
					nfc_tool_unlock_read();
					m1_uiView_display_update(X_MENU_UPDATE_REFRESH);
					break;

				default:
					break;
			}
		}
		else if (this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK)
		{
			m1_uiView_display_update(X_MENU_UPDATE_MOVE_UP);
		}
		else if (this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK)
		{
			m1_uiView_display_update(X_MENU_UPDATE_MOVE_DOWN);
		}
	}

	return 1;
}


/*============================================================================*/
 /* @brief nfc_utils_gui_create - Create and initialize NFC utils view
 * 
 * Initializes the utils view and triggers an update.
 * 
 * @param[in] param View parameter
 * @retval None
 */
/*============================================================================*/
static void nfc_utils_gui_create(uint8_t param)
{
	m1_uiView_display_update(param);
}

/*============================================================================*/
 /* @brief nfc_utils_gui_destroy - Destroy NFC utils view and cleanup resources
 * 
 * Cleans up the utils view by turning off LED blink indication.
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_utils_gui_destroy(uint8_t param)
{
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
}

/*============================================================================*/
 /* @brief nfc_utils_gui_update - Update NFC utils view display
 * 
 * Updates the utils menu display.
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_utils_gui_update(uint8_t param)
{
	m1_gui_submenu_update(m1_nfc_tool_options, NFC_TOOL_OPTIONS_COUNT, 0, param);
}

/*============================================================================*/
 /* @brief nfc_utils_gui_message - Process messages for NFC utils view
 * 
 * Handles messages from the main queue, primarily keypad events.
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_utils_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_utils_kp_handler();
		}
		else
		{
			; // Do other things for this task
		}
	} 

	return ret_val;
}

/*============================================================================*/
/**
 * @brief nfc_utils_gui_init - Initialize and register NFC utils view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC utils view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_utils_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_UTILS, nfc_utils_gui_create, nfc_utils_gui_update, nfc_utils_gui_destroy, nfc_utils_gui_message);
}

/*============================================================================*/
/**
 * @brief nfc_info_kp_handler - Handle keypad input for NFC info view
 * 
 * Processes button events in the NFC info view:
 * - BACK: Return to submenu with saved cursor index
 * - RIGHT: Switch to page dump view (param = 1)
 * - UP: Scroll up in page view (if in page view mode)
 * - DOWN: Scroll down in page view (if in page view mode)
 * 
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_info_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);// Return submenu to stored index
		}
		else if (this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK)
		{
			m1_uiView_display_update(1); // Configure the next page with param up count each time you press right button
		}
		else if(this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )
		{
			// Do other things for this task, if needed
		}
		else if(this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if (s_info_mode==1)
			{
				if (s_page_scroll > 0)
				{
					s_page_scroll--;
					m1_uiView_display_update(1);
				}
			}
		}
		else if(this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
		{
			if (s_info_mode==1)   // Scroll only in Page View
			{
				uint16_t total = nfc_ctx_get_t2t_page_count();
				if (total > 0)
				{
					uint16_t max_scroll = // Maximum value of the starting index = total_pages - number of lines in Hanwha face
						(total > NFC_INFO_LINES_PER_SCREEN)
						? (total - NFC_INFO_LINES_PER_SCREEN)
						: 0;

					if (s_page_scroll < max_scroll)
					{
						s_page_scroll++;
						m1_uiView_display_update(1);
					}
				}
			}
		} // else if(this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
	} // if (ret==pdTRUE)

	return 1;
}


/*============================================================================*/
/**
 * @brief nfc_info_gui_create - Create and initialize NFC info view
 * 
 * Initializes the info view by resetting info mode to 0 (summary screen)
 * and page scroll index to 0.
 * 
 * @param[in] param View parameter
 * @retval None
 */
/*============================================================================*/
static void nfc_info_gui_create(uint8_t param)
{
	s_info_mode   = 0;  // Initially, the Summary Information Screen
    s_page_scroll = 0;  // Page Viewer Start Index 0

	m1_uiView_display_update(param);
}

/*============================================================================*/
/**
 * @brief nfc_info_gui_destroy - Destroy NFC info view and cleanup resources
 * 
 * Cleans up the info view by turning off LED blink indication.
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_info_gui_destroy(uint8_t param)
{
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
}

/*============================================================================*/
/**
 * @brief nfc_info_gui_update - Update NFC info view display
 * 
 * Updates the display based on the info mode:
 * - param 0: Shows default card information screen (summary)
 * - param 1: Shows card sector/page dump view
 * 
 * @param[in] param View update parameter (0 = summary, 1 = page dump)
 * @retval None
 */
/*============================================================================*/
static void nfc_info_gui_update(uint8_t param)
{
    if (param==0)
    {
		//Default Card Information Screen
        s_info_mode = 0;
		nfc_info_drawing();
    }
    else
    {	//Card Sector, Page Dump View
        s_info_mode = 1;
        m1_nfc_info_more_draw();
    }
}

/*============================================================================*/
/**
 * @brief nfc_info_gui_message - Process messages for NFC info view
 * 
 * Handles messages from the main queue, primarily keypad events.
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_info_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_info_kp_handler();
		}
		else
		{
			; // Do other things for this task
		}
	}

	return ret_val;
}

/*============================================================================*/
/**
 * @brief nfc_info_gui_init - Initialize and register NFC info view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC info view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_info_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_INFO, nfc_info_gui_create, nfc_info_gui_update, nfc_info_gui_destroy, nfc_info_gui_message);
}

/**
 * @brief nfc_info_drawing - Draw NFC card information summary screen
 * 
 * Displays the default card information screen showing:
 * - Technology type (ISO14443A/B/F/V)
 * - Card family/type name
 * - UID (Unique Identifier)
 * - ATQA and SAK values (for ISO14443A cards)
 * 
 * @retval None
 */
static void nfc_info_drawing(void)
{
    nfc_run_ctx_t* c = nfc_ctx_get();

    char type_str[32];
    char family_str[32];
    char uid_str[3 * 10 + 1];   // UID Up to 10 bytes "AA BB ...\0"
    char atqa_str[8];           // "44 00\0"
    char sak_str[4];            // "08\0"

    strcpy(type_str,   "No NFC context");
    family_str[0] = '\0';
    uid_str[0]    = '\0';
    strcpy(atqa_str, "-- --");
    strcpy(sak_str,  "--");

    if (c && c->head.uid_len > 0)
    {
        /* 1) Tech → Type string */
        switch (c->head.tech)
        {
        	case M1NFC_TECH_A: // enum value is based on nfc_ctx.h
        		strcpy(type_str, "ISO14443A / NFC-A");
        		break;
        	case M1NFC_TECH_B:
        		strcpy(type_str, "ISO14443B / NFC-B");
        		break;
        	case M1NFC_TECH_F:
        		strcpy(type_str, "Felica / NFC-F");
        		break;
        	case M1NFC_TECH_V:
        		strcpy(type_str, "ISO15693 / NFC-V");
        		break;
        	default:
        		strcpy(type_str, "Unknown TECH");
        		break;
        }

		/* 2) Family → title string
         * Reuse the title_text created by nfc_ctx_refresh_ui()
         */
        if (c->ui.title_text[0] != '\0')
        {
            strncpy(family_str, c->ui.title_text, sizeof(family_str) - 1);
            family_str[sizeof(family_str) - 1] = '\0';
        }
        else
        {
            strcpy(family_str, "Unknown family");
        }

        /* 3) UID: "AA BB CC ..." */
        snprintf(uid_str, sizeof(uid_str),
                 "%s", hex2Str(c->head.uid, c->head.uid_len));

        /* 4) ATQA / SAK */
        if (c->head.a.has_atqa)
        {
            snprintf(atqa_str, sizeof(atqa_str),
                     "%02X %02X", c->head.a.atqa[0], c->head.a.atqa[1]);
        }
        if (c->head.a.has_sak)
        {
            snprintf(sak_str, sizeof(sak_str),
                     "%02X", c->head.a.sak);
        }
    }

    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_FirstPage(&m1_u8g2); 

    u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
    u8g2_DrawStr(&m1_u8g2, 2, 12, type_str);

    u8g2_DrawStr(&m1_u8g2, 2, 22, family_str);

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 2, 32, "UID:");
    u8g2_DrawStr(&m1_u8g2, 25, 32, uid_str);

    u8g2_DrawStr(&m1_u8g2, 2, 42,  "ATQA:");
    u8g2_DrawStr(&m1_u8g2, 40, 42, atqa_str);
    u8g2_DrawStr(&m1_u8g2, 80, 42, "SAK:");
    u8g2_DrawStr(&m1_u8g2, 105, 42, sak_str);

    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); 
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); 
    u8g2_DrawXBMP(&m1_u8g2, 119, 53, 8, 8, arrowright_8x8); 
    u8g2_DrawStr(&m1_u8g2, 97, 61, "More");

    m1_u8g2_nextpage(); 
}

/*============================================================================*/
/**
 * @brief m1_nfc_info_more_draw - Draw NFC info more view
 * 
 * Draws the NFC info more view with the page information.
 * 
 * @retval None
 */
/*============================================================================*/	
void m1_nfc_info_more_draw(void)
{
    /******************************/
    // Max 134   [hex]   | [Asc]
    // P000: 04 B7 28 13 | ..(.
    // P001: 27 3F 61 81 | '?a.
    // P002: F8 48 00 00 | .H..
    // P003: E1 10 3E 00 | ..>.
    // P004: 03 00 FE 00 | ....
    /******************************/

    const uint16_t total_pages = nfc_ctx_get_t2t_page_count();
    char line[32];
    char header[32];

	// Correction of the current start page index (if it's over the range)
    uint16_t start = s_page_scroll;
    if (total_pages==0)
    {
        start = 0;
    }
    else
    {
        uint16_t max_scroll =
            (total_pages > NFC_INFO_LINES_PER_SCREEN)
            ? (total_pages - NFC_INFO_LINES_PER_SCREEN)
            : 0;

        if (start > max_scroll)
            start = max_scroll;
        s_page_scroll = start;
    }
	// Calculate the last page index that is actually visible on the screen
    //uint16_t last_index = 0; //temp
    if (total_pages==0)
    {
        //last_index = 0;
    }
    else
    {
        uint16_t tmp = start + NFC_INFO_LINES_PER_SCREEN - 1;
        if (tmp >= total_pages)
            tmp = total_pages - 1;
        //last_index = tmp;
    }

    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_FirstPage(&m1_u8g2);


    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    if (total_pages==0)
    {
		u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
		u8g2_DrawStr(&m1_u8g2, 2, 20, "No page data");
		u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 2, 32, "Only UID-level info");
		u8g2_DrawStr(&m1_u8g2, 2, 42, "available for this tag");
		m1_u8g2_nextpage();
    }
    else
    {
        uint16_t max_page = (uint16_t)(total_pages - 1);
        snprintf(header, sizeof(header),
                 "Max %03u  [hex]  | [Asc]",
                 (unsigned)max_page);
        u8g2_DrawStr(&m1_u8g2, 2, 8, header);
		u8g2_DrawStr(&m1_u8g2, 2, 16,  "------------------------");

    }

    if (total_pages > 0)
    {
        for (uint8_t row = 0; row < NFC_INFO_LINES_PER_SCREEN; row++)
        {
            uint16_t page = start + row;
            if (page >= total_pages)
                break;

            if (!nfc_ctx_format_t2t_page_line(page, line, sizeof(line)))
                continue;

            uint8_t y = 24 + row * 8;
            u8g2_DrawStr(&m1_u8g2, 2, y, line);
        }
    }

    m1_u8g2_nextpage();
}

/*============================================================================*/
/*                   NFC TOOLS — Helpers                                      */
/*============================================================================*/

/* NFC-A manufacturer lookup (ISO/IEC 7816-6 / JIS X 6319-4, byte 0 of UID) */
static const char* nfc_tool_manufacturer_name(uint8_t mfr_byte)
{
	switch (mfr_byte) {
		case 0x01: return "Motorola";
		case 0x02: return "STMicro";
		case 0x03: return "Hitachi";
		case 0x04: return "NXP";
		case 0x05: return "Infineon";
		case 0x06: return "Cylink";
		case 0x07: return "TI";
		case 0x08: return "Fujitsu";
		case 0x09: return "Matsushita";
		case 0x0A: return "NEC";
		case 0x0B: return "Oki";
		case 0x0C: return "Toshiba";
		case 0x0D: return "Mitsubishi";
		case 0x0E: return "Samsung";
		case 0x0F: return "Hyundai";
		case 0x10: return "LG";
		case 0x16: return "EM Micro";
		case 0x28: return "SiliconCraft";
		default:   return "Unknown";
	}
}

/* SAK meaning for NFC-A */
static const char* nfc_tool_sak_meaning(uint8_t sak, const uint8_t atqa[2])
{
	if (sak == 0x08) return "Classic 1K";
	if (sak == 0x18) return "Classic 4K";
	if (sak == 0x09) return "Classic Mini";
	if (sak == 0x10) return "Classic 2K";
	if (sak == 0x11) return "Classic 4K (Plus)";
	if (sak == 0x00 && atqa[0] == 0x44) return "Ultralight/NTAG";
	if (sak == 0x20) return "DESFire/ISO-DEP";
	if (sak == 0x28) return "Classic+ISO-DEP";
	if (sak == 0x60) return "Classic+DESFire";
	return "Other";
}

/* Fuzzer card profiles for NFC-A emulation */
#define NFC_FUZZ_PROFILE_COUNT  4
static const char *nfc_fuzz_profile_names[] = {
	"Classic 1K", "Classic 4K", "Ultralight", "DESFire"
};
static const uint8_t nfc_fuzz_profile_atqa[][2] = {
	{0x04, 0x00}, {0x02, 0x00}, {0x44, 0x00}, {0x04, 0x03}
};
static const uint8_t nfc_fuzz_profile_sak[] = {
	0x08, 0x18, 0x00, 0x20
};
static const uint8_t nfc_fuzz_profile_uid_len[] = {
	4, 4, 7, 7
};

static void nfc_fuzz_uid_step(uint8_t *uid, uint8_t uid_len, int8_t dir)
{
	if (dir > 0) {
		for (int i = uid_len - 1; i >= 0; i--) {
			uid[i]++;
			if (uid[i] != 0) break;
		}
	} else {
		for (int i = uid_len - 1; i >= 0; i--) {
			uid[i]--;
			if (uid[i] != 0xFF) break;
		}
	}
}

static void nfc_fuzz_draw_setup(uint8_t prof_sel, const uint8_t *uid, uint8_t uid_len,
                                int8_t dir, uint16_t delay_ms)
{
	char line[32];
	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 2, 10, "NFC Fuzzer Setup");

	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	snprintf(line, sizeof(line), "Type: %s", nfc_fuzz_profile_names[prof_sel]);
	u8g2_DrawStr(&m1_u8g2, 2, 22, line);

	snprintf(line, sizeof(line), "UID: %s", hex2Str((unsigned char*)uid, uid_len));
	u8g2_DrawStr(&m1_u8g2, 2, 32, line);

	snprintf(line, sizeof(line), "Dir: %s  Dly: %ums",
	         (dir > 0) ? "UP" : "DN", delay_ms);
	u8g2_DrawStr(&m1_u8g2, 2, 42, line);

	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	u8g2_DrawStr(&m1_u8g2, 4, 61, "UP/DN:adj");
	u8g2_DrawStr(&m1_u8g2, 74, 61, "OK:Start");

	m1_u8g2_nextpage();
}

static void nfc_fuzz_draw_running(uint8_t prof_sel, const uint8_t *uid, uint8_t uid_len,
                                  uint32_t count)
{
	char line[32];
	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 2, 10, "NFC Fuzzer");

	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	snprintf(line, sizeof(line), "Type: %s", nfc_fuzz_profile_names[prof_sel]);
	u8g2_DrawStr(&m1_u8g2, 2, 22, line);

	snprintf(line, sizeof(line), "UID: %s", hex2Str((unsigned char*)uid, uid_len));
	u8g2_DrawStr(&m1_u8g2, 2, 32, line);

	snprintf(line, sizeof(line), "Count: %lu", (unsigned long)count);
	u8g2_DrawStr(&m1_u8g2, 2, 42, line);

	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	u8g2_DrawStr(&m1_u8g2, 2, 61, "Emulating...");
	u8g2_DrawStr(&m1_u8g2, 86, 61, "Bk:Stop");

	m1_u8g2_nextpage();
}

/*============================================================================*/
/**
 * @brief nfc_tool_tag_info - Read NFC tag and display enhanced info
 *
 * Initiates an NFC read, then displays detailed card metadata including
 * manufacturer, card type, ATQA/SAK analysis, and memory info.
 *
 * @retval None
 */
/*============================================================================*/
static void nfc_tool_tag_info(void)
{
	S_M1_Buttons_Status bs;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;

	/* Show "Place card" screen */
	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_DrawXBMP(&m1_u8g2, 0, 0, 48, 48, nfc_read_48x48);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 55, 20, "Tag Info");
	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	u8g2_DrawStr(&m1_u8g2, 50, 30, "Hold card next");
	u8g2_DrawStr(&m1_u8g2, 50, 40, "to M1's back");
	m1_u8g2_nextpage();

	/* Start NFC read */
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_START_READ);
	vTaskDelay(50);

	/* Wait for read complete or BACK */
	bool read_done = false;
	while (!read_done)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;

		if (q_item.q_evt_type == Q_EVENT_NFC_READ_COMPLETE)
		{
			read_done = true;
			m1_buzzer_notification();
			m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
		}
		else if (q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &bs, 0);
			if (ret == pdTRUE && bs.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
				m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_READ_COMPLETE);
				return;
			}
		}
	}

	/* Display enhanced tag info */
	nfc_run_ctx_t *c = nfc_ctx_get();
	if (!c || c->head.uid_len == 0)
	{
		m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_READ_COMPLETE);
		return;
	}

	uint16_t info_page = 0;   /* 0 = summary, 1 = more detail */
	uint16_t page_scroll = 0;
	bool show = true;

	while (1)
	{
		if (show)
		{
			show = false;

			if (info_page == 0)
			{
				/* Page 0: Summary info */
				char line[40];
				u8g2_FirstPage(&m1_u8g2);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
				u8g2_DrawStr(&m1_u8g2, 2, 10, c->ui.title_text);

				u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
				snprintf(line, sizeof(line), "Mfr: %s (0x%02X)",
				         nfc_tool_manufacturer_name(c->head.uid[0]), c->head.uid[0]);
				u8g2_DrawStr(&m1_u8g2, 2, 20, line);

				snprintf(line, sizeof(line), "UID: %s",
				         hex2Str(c->head.uid, c->head.uid_len));
				u8g2_DrawStr(&m1_u8g2, 2, 30, line);

				if (c->head.a.has_atqa && c->head.a.has_sak)
				{
					snprintf(line, sizeof(line), "ATQA:%02X %02X SAK:%02X",
					         c->head.a.atqa[0], c->head.a.atqa[1], c->head.a.sak);
					u8g2_DrawStr(&m1_u8g2, 2, 40, line);

					snprintf(line, sizeof(line), "Type: %s",
					         nfc_tool_sak_meaning(c->head.a.sak, c->head.a.atqa));
					u8g2_DrawStr(&m1_u8g2, 2, 50, line);
				}

				/* Bottom bar */
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_DrawBox(&m1_u8g2, 0, 54, 128, 10);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
				u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
				u8g2_DrawXBMP(&m1_u8g2, 119, 55, 8, 8, arrowright_8x8);
				u8g2_DrawStr(&m1_u8g2, 97, 62, "More");

				m1_u8g2_nextpage();
			}
			else
			{
				/* Page 1: T2T page dump (if available) */
				uint16_t total = nfc_ctx_get_t2t_page_count();
				char line[32];
				char header[32];

				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_FirstPage(&m1_u8g2);
				u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);

				if (total == 0)
				{
					u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
					u8g2_DrawStr(&m1_u8g2, 2, 20, "No page data");
					u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
					u8g2_DrawStr(&m1_u8g2, 2, 32, "Only UID-level info");
					u8g2_DrawStr(&m1_u8g2, 2, 42, "available for this tag");
				}
				else
				{
					uint16_t max_scroll = (total > NFC_INFO_LINES_PER_SCREEN)
					    ? (total - NFC_INFO_LINES_PER_SCREEN) : 0;
					if (page_scroll > max_scroll) page_scroll = max_scroll;

					snprintf(header, sizeof(header), "Max %03u  [hex]  | [Asc]",
					         (unsigned)(total - 1));
					u8g2_DrawStr(&m1_u8g2, 2, 8, header);
					u8g2_DrawStr(&m1_u8g2, 2, 16, "------------------------");

					for (uint8_t row = 0; row < NFC_INFO_LINES_PER_SCREEN; row++)
					{
						uint16_t pg = page_scroll + row;
						if (pg >= total) break;
						if (!nfc_ctx_format_t2t_page_line(pg, line, sizeof(line)))
							continue;
						u8g2_DrawStr(&m1_u8g2, 2, 24 + row * 8, line);
					}
				}
				m1_u8g2_nextpage();
			}
		}

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;

		if (q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &bs, 0);
			if (ret != pdTRUE) continue;

			if (bs.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				break;
			}
			else if (bs.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (info_page == 0)
				{
					info_page = 1;
					page_scroll = 0;
					show = true;
				}
			}
			else if (bs.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (info_page == 1)
				{
					info_page = 0;
					show = true;
				}
			}
			else if (bs.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (info_page == 1 && page_scroll > 0)
				{
					page_scroll--;
					show = true;
				}
			}
			else if (bs.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (info_page == 1)
				{
					uint16_t total = nfc_ctx_get_t2t_page_count();
					uint16_t max_scroll = (total > NFC_INFO_LINES_PER_SCREEN)
					    ? (total - NFC_INFO_LINES_PER_SCREEN) : 0;
					if (page_scroll < max_scroll)
					{
						page_scroll++;
						show = true;
					}
				}
			}
		}
	}

	/* Cleanup: stop any active read operation */
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_READ_COMPLETE);
}


/*============================================================================*/
/**
 * @brief nfc_tool_clone_emu - Read NFC card then immediately emulate it
 *
 * One-step clone workflow: reads a source card, then starts emulating its
 * UID so another NFC reader/writer can capture it.
 *
 * @retval None
 */
/*============================================================================*/
static void nfc_tool_clone_emu(void)
{
	S_M1_Buttons_Status bs;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;

	/* Show "Place source card" screen */
	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_DrawXBMP(&m1_u8g2, 0, 0, 48, 48, nfc_read_48x48);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 50, 15, "Clone Emulate");
	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	u8g2_DrawStr(&m1_u8g2, 50, 26, "Place source");
	u8g2_DrawStr(&m1_u8g2, 50, 36, "card on M1's");
	u8g2_DrawStr(&m1_u8g2, 50, 46, "back to read");
	m1_u8g2_nextpage();

	/* Start NFC read */
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_START_READ);
	vTaskDelay(50);

	/* Wait for read complete or BACK */
	bool read_done = false;
	while (!read_done)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;

		if (q_item.q_evt_type == Q_EVENT_NFC_READ_COMPLETE)
		{
			read_done = true;
			m1_buzzer_notification();
			m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
		}
		else if (q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &bs, 0);
			if (ret == pdTRUE && bs.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
				m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_READ_COMPLETE);
				return;
			}
		}
	}

	nfc_run_ctx_t *c = nfc_ctx_get();
	if (!c || c->head.uid_len == 0)
	{
		m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_READ_COMPLETE);
		return;
	}

	/* Sync emulator context and start emulation */
	nfc_ctx_sync_emu();
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_START_EMULATE);
	vTaskDelay(50);

	/* Show emulating screen */
	const char *emu_text = "Emulate UID";
	if (c->head.family == M1NFC_FAM_ULTRALIGHT &&
	    c->dump.has_dump && c->dump.data != NULL &&
	    c->dump.unit_size == 4 && c->dump.unit_count > 0)
	{
		emu_text = "Emulate";
	}
	else if (c->head.family == M1NFC_FAM_CLASSIC &&
	         c->dump.has_dump && c->dump.data != NULL &&
	         c->dump.unit_size == 16 && c->dump.unit_count > 0)
	{
		emu_text = "Emulate";
	}

	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_DrawXBMP(&m1_u8g2, 0, 0, 48, 48, nfc_emit_48x48);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 50, 15, emu_text);
	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	u8g2_DrawStr(&m1_u8g2, 50, 26, "UID:");
	u8g2_DrawStr(&m1_u8g2, 50, 36, hex2Str(c->head.uid, c->head.uid_len));
	u8g2_DrawStr(&m1_u8g2, 50, 46, "Tag M1 on reader");
	m1_u8g2_nextpage();

	/* Wait for BACK to stop emulation */
	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;

		if (q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &bs, 0);
			if (ret == pdTRUE && bs.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				break;
			}
		}
	}

	/* Stop emulation */
	ListenerRequestStop();
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_EMULATE_STOP);
	vTaskDelay(50);
}


/*============================================================================*/
/**
 * @brief nfc_tool_fuzzer - NFC UID fuzzer via emulation
 *
 * Cycles through NFC UIDs while emulating, allowing card type selection,
 * direction (increment/decrement), and delay configuration.
 *
 * @retval None
 */
/*============================================================================*/
static void nfc_tool_fuzzer(void)
{
	S_M1_Buttons_Status bs;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;

	uint8_t prof_sel = 0;
	int8_t dir = 1;
	uint16_t delay_ms = 500;
	uint8_t setup_field = 0; /* 0=type, 1=dir, 2=delay */
	uint8_t uid[10];
	memset(uid, 0, sizeof(uid));
	uid[0] = 0x04; /* NXP manufacturer byte */

	/* --- Setup screen --- */
	nfc_fuzz_draw_setup(prof_sel, uid, nfc_fuzz_profile_uid_len[prof_sel], dir, delay_ms);

	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;

		if (q_item.q_evt_type != Q_EVENT_KEYPAD) continue;

		ret = xQueueReceive(button_events_q_hdl, &bs, 0);
		if (ret != pdTRUE) continue;

		if (bs.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
		{
			return; /* Exit to tools menu */
		}
		else if (bs.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
		{
			break; /* Start fuzzing */
		}
		else if (bs.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
		{
			if (setup_field == 0) {
				prof_sel = (prof_sel + 1) % NFC_FUZZ_PROFILE_COUNT;
				memset(uid, 0, sizeof(uid));
				uid[0] = 0x04;
			} else if (setup_field == 1) {
				dir = (dir > 0) ? -1 : 1;
			} else {
				if (delay_ms < 2000) delay_ms += 100;
			}
			nfc_fuzz_draw_setup(prof_sel, uid, nfc_fuzz_profile_uid_len[prof_sel], dir, delay_ms);
		}
		else if (bs.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
		{
			if (setup_field == 0) {
				prof_sel = (prof_sel == 0) ? (NFC_FUZZ_PROFILE_COUNT - 1) : (prof_sel - 1);
				memset(uid, 0, sizeof(uid));
				uid[0] = 0x04;
			} else if (setup_field == 1) {
				dir = (dir > 0) ? -1 : 1;
			} else {
				if (delay_ms > 100) delay_ms -= 100;
			}
			nfc_fuzz_draw_setup(prof_sel, uid, nfc_fuzz_profile_uid_len[prof_sel], dir, delay_ms);
		}
		else if (bs.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
		{
			setup_field = (setup_field + 1) % 3;
			nfc_fuzz_draw_setup(prof_sel, uid, nfc_fuzz_profile_uid_len[prof_sel], dir, delay_ms);
		}
		else if (bs.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
		{
			setup_field = (setup_field == 0) ? 2 : (setup_field - 1);
			nfc_fuzz_draw_setup(prof_sel, uid, nfc_fuzz_profile_uid_len[prof_sel], dir, delay_ms);
		}
	}

	/* --- Start emulation fuzzing --- */
	uint8_t uid_len = nfc_fuzz_profile_uid_len[prof_sel];
	uint32_t count = 0;

	/* Set initial emulator parameters */
	Emu_SetNfcA(uid, uid_len,
	            nfc_fuzz_profile_atqa[prof_sel][0],
	            nfc_fuzz_profile_atqa[prof_sel][1],
	            nfc_fuzz_profile_sak[prof_sel]);

	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_START_EMULATE);
	vTaskDelay(50);

	nfc_fuzz_draw_running(prof_sel, uid, uid_len, count);

	/* Fuzzing loop: use timed queue receive for delay between UID steps */
	bool running = true;
	while (running)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(delay_ms));
		if (ret == pdTRUE)
		{
			if (q_item.q_evt_type == Q_EVENT_KEYPAD)
			{
				ret = xQueueReceive(button_events_q_hdl, &bs, 0);
				if (ret == pdTRUE && bs.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
				{
					running = false;
					break;
				}
			}
		}

		/* Timeout = step to next UID */
		nfc_fuzz_uid_step(uid, uid_len, dir);
		count++;

		/* Update emulator with new UID (live update, no stop/start needed) */
		Emu_SetNfcA(uid, uid_len,
		            nfc_fuzz_profile_atqa[prof_sel][0],
		            nfc_fuzz_profile_atqa[prof_sel][1],
		            nfc_fuzz_profile_sak[prof_sel]);

		nfc_fuzz_draw_running(prof_sel, uid, uid_len, count);
	}

	/* Stop emulation */
	ListenerRequestStop();
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_EMULATE_STOP);
	vTaskDelay(50);
}


/*============================================================================*/
/**
 * @brief nfc_tool_unlock_read - Read NFC tag with user-entered password
 *
 * Prompts the user for a 4-byte hex password via the short keyboard,
 * stores it as the manual password, then initiates a tag read.
 * The poller will use this password for PWD_AUTH before reading pages.
 *
 * @retval None
 */
/*============================================================================*/
static void nfc_tool_unlock_read(void)
{
	S_M1_Buttons_Status bs;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	char pwd_buf[12]; /* "00 00 00 00" + null */
	uint8_t pwd_bytes[4];

	/* Check if a captured password is available and pre-fill */
	uint8_t cap_pwd[4];
	if (nfc_ctx_get_captured_pwd(cap_pwd)) {
		snprintf(pwd_buf, sizeof(pwd_buf), "%02X %02X %02X %02X",
				 cap_pwd[0], cap_pwd[1], cap_pwd[2], cap_pwd[3]);
	} else {
		strcpy(pwd_buf, "00 00 00 00");
	}

	/* Show hex keyboard for 4-byte password entry */
	uint8_t len = m1_vkbs_get_data("NFC Password", pwd_buf);
	if (len == 0) {
		return; /* User cancelled */
	}

	/* Parse "XX XX XX XX" hex string → 4 bytes */
	unsigned int b0, b1, b2, b3;
	if (sscanf(pwd_buf, "%02X %02X %02X %02X", &b0, &b1, &b2, &b3) != 4) {
		m1_message_box(&m1_u8g2, "Invalid password", NULL, " ", res_string(IDS_BACK));
		return;
	}
	pwd_bytes[0] = (uint8_t)b0;
	pwd_bytes[1] = (uint8_t)b1;
	pwd_bytes[2] = (uint8_t)b2;
	pwd_bytes[3] = (uint8_t)b3;

	/* Store as manual password — poller will use it on next read */
	nfc_ctx_set_manual_pwd(pwd_bytes);

	/* Show "Place card" screen */
	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_DrawXBMP(&m1_u8g2, 0, 0, 48, 48, nfc_read_48x48);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 50, 15, "Unlock Read");
	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	u8g2_DrawStr(&m1_u8g2, 50, 26, "PWD:");
	{
		char pwd_disp[16];
		snprintf(pwd_disp, sizeof(pwd_disp), "%02X %02X %02X %02X",
				 pwd_bytes[0], pwd_bytes[1], pwd_bytes[2], pwd_bytes[3]);
		u8g2_DrawStr(&m1_u8g2, 50, 36, pwd_disp);
	}
	u8g2_DrawStr(&m1_u8g2, 50, 46, "Hold card on M1");
	m1_u8g2_nextpage();

	/* Start NFC read */
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M);
	m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_START_READ);
	vTaskDelay(50);

	/* Wait for read complete or BACK */
	bool read_done = false;
	while (!read_done)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;

		if (q_item.q_evt_type == Q_EVENT_NFC_READ_COMPLETE)
		{
			read_done = true;
			m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);

			nfc_run_ctx_t *c = nfc_ctx_get();
			if (c && c->head.uid_len > 0) {
				/* Show result */
				u8g2_FirstPage(&m1_u8g2);
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
				u8g2_DrawStr(&m1_u8g2, 2, 12, "Unlock Read Done");
				u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
				u8g2_DrawStr(&m1_u8g2, 2, 24, c->ui.title_text);
				u8g2_DrawStr(&m1_u8g2, 2, 34, "UID:");
				u8g2_DrawStr(&m1_u8g2, 30, 34, c->ui.uid_text);
				if (c->dump.has_dump) {
					char pg_str[24];
					snprintf(pg_str, sizeof(pg_str), "Pages: %u",
							 (unsigned)nfc_ctx_get_t2t_page_count());
					u8g2_DrawStr(&m1_u8g2, 2, 44, pg_str);
				}
				m1_u8g2_nextpage();

				/* Wait for BACK */
				while (1) {
					ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
					if (ret != pdTRUE) continue;
					if (q_item.q_evt_type != Q_EVENT_KEYPAD) continue;
					ret = xQueueReceive(button_events_q_hdl, &bs, 0);
					if (ret == pdTRUE && bs.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
						break;
				}
			}
		}
		else if (q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &bs, 0);
			if (ret == pdTRUE && bs.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				/* User cancelled — stop reader and clear manual password */
				m1_app_send_q_message(nfc_worker_q_hdl, Q_EVENT_NFC_STOP);
				nfc_ctx_clear_manual_pwd();
				m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF);
				read_done = true;
			}
		}
	}
}


/*============================================================================*/
/**
 * @brief nfc_tools - NFC tools menu with Tag Info, Clone Emulate, NFC Fuzzer
 *
 * Provides a submenu of NFC utility tools accessible from the main NFC menu.
 *
 * @retval None
 */
/*============================================================================*/
void nfc_tools(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;

	/* Drain stale events from parent menu selection */
	while (xQueueReceive(main_q_hdl, &q_item, 0) == pdTRUE)
	{
		if (q_item.q_evt_type == Q_EVENT_KEYPAD)
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	}

	m1_gui_submenu_update(NULL, 0, 0, X_MENU_UPDATE_INIT);
	m1_gui_submenu_update(m1_nfc_tool_options, NFC_TOOL_OPTIONS_COUNT, 0, X_MENU_UPDATE_RESET);

	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE) continue;

		if (q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
			if (ret != pdTRUE) continue;

			if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				xQueueReset(main_q_hdl);
				break;
			}
			else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				uint8_t sel = m1_gui_submenu_update(NULL, 0, 0, MENU_UPDATE_NONE);
				switch (sel)
				{
					case 0: nfc_tool_tag_info();     break;
					case 1: nfc_tool_clone_emu();    break;
					case 2: nfc_tool_fuzzer();       break;
					case 5: nfc_tool_unlock_read();  break;
					default: break;
				}
				/* Redraw submenu after returning from a tool */
				m1_gui_submenu_update(m1_nfc_tool_options, NFC_TOOL_OPTIONS_COUNT, 0, X_MENU_UPDATE_REFRESH);
			}
			else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
			{
				m1_gui_submenu_update(m1_nfc_tool_options, NFC_TOOL_OPTIONS_COUNT, 0, X_MENU_UPDATE_MOVE_UP);
			}
			else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
			{
				m1_gui_submenu_update(m1_nfc_tool_options, NFC_TOOL_OPTIONS_COUNT, 0, X_MENU_UPDATE_MOVE_DOWN);
			}
		}
	}
} 

/*============================================================================*/
/**
 * @brief nfc_saved - Load and display NFC card data from saved file
 * 
 * This function allows the user to browse and load a previously saved
 * NFC card file from SD card. It validates the file extension (.nfc),
 * loads the file data into NFC context, and enters the submenu view
 * with the loaded card data.
 * 
 * The function handles various error conditions:
 * - File not selected
 * - Invalid file name
 * - Invalid extension
 * - File load failure
 * 
 * @retval None
 */
/*============================================================================*/
/*============================================================================*/
/**
 * @brief nfc_saved - Browse and load NFC card data from saved file
 * 
 * This function uses the uiView system to browse and load a previously saved
 * NFC card file from SD card. It follows the same pattern as RFID's saved
 * functionality for consistency.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_saved(void)
{
	nfc_run_ctx_t ctx;
    nfc_run_ctx_init(&ctx);
	
	platformLog("nfc_saved()\r\n");
	m1_gui_submenu_update(NULL, 0, 0, X_MENU_UPDATE_INIT);
	nfc_uiview_gui_latest_param = 0xFF; // Initialize with an invalid parameter
	// initial
	m1_uiView_functions_init(VIEW_MODE_NFC_END, view_nfc_read_table);
	m1_uiView_display_switch(VIEW_MODE_NFC_SAVED_BROWSE, 0);

	// loop
	while( m1_uiView_q_message_process() )
	{
		;
	}
} 


/*============================================================================*/
/**
 * @brief nfc_read_more_options_save - Save NFC card data to SD card file
 * 
 * Saves the current NFC card context to a file on the SD card in
 * the NFC directory. The function:
 * 1. Checks SD card free space (minimum 4KB required)
 * 2. Creates /NFC directory if it doesn't exist
 * 3. Prompts user for filename using virtual keyboard
 * 4. Validates filename doesn't already exist
 * 5. Writes card data in M1 NFC device format (Version 4)
 * 6. Includes device type, UID, ATQA, SAK, and page dumps (for Ultralight/NTAG)
 * 
 * @retval 0 Success
 * @retval 1 Insufficient SD card space
 * @retval 2 Directory creation failed
 * @retval 3 User cancelled (escaped)
 * @retval 4 File creation failed
 */
/*============================================================================*/
static uint8_t nfc_read_more_options_save(void)
{
	char filepath[128];
	uint8_t error;
	nfc_run_ctx_t* c;

	// Get filename from user and create full path
	error = nfc_save_file_keyboard(filepath);
	if (error != 0) {
		// Error or user escaped
		if (error != 3) { // Not user escape - show error
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_FirstPage(&m1_u8g2);
			u8g2_DrawXBMP(&m1_u8g2, 32, 0, 63, 63, micro_sd_card_error);
			u8g2_NextPage(&m1_u8g2);
		}
		return error;
	}

	// Save NFC profile to file
	c = nfc_ctx_get();
	if (!c) {
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_FirstPage(&m1_u8g2);
		u8g2_DrawXBMP(&m1_u8g2, 32, 0, 63, 63, micro_sd_card_error);
		u8g2_NextPage(&m1_u8g2);
		return 4; // Error
	}

	if (nfc_profile_save(filepath, c)) {
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_FirstPage(&m1_u8g2);
		u8g2_DrawXBMP(&m1_u8g2, 32, 0, 63, 63, nfc_saved_63_63);
		u8g2_NextPage(&m1_u8g2);
		return 0; // Success
	} else {
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_FirstPage(&m1_u8g2);
		u8g2_DrawXBMP(&m1_u8g2, 32, 0, 63, 63, micro_sd_card_error);
		u8g2_NextPage(&m1_u8g2);
		return 4; // Error
	}
} 

/*============================================================================*/
/**
 * @brief nfc_edit_uid_kp_handler - Handle keypad input for NFC edit UID view
 * 
 * Processes button events in the NFC edit UID view:
 * - BACK: Return to submenu
 * 
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_edit_uid_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		}
	}
	return 1;
}

/*============================================================================*/
/**
 * @brief nfc_edit_uid_gui_create - Create and initialize NFC edit UID view
 * 
 * Initializes the edit UID view and triggers an update.
 * 
 * @param[in] param View parameter
 * @retval None
 */
/*============================================================================*/
static void nfc_edit_uid_gui_create(uint8_t param)
{
	s_edit_uid_started = false;  // 초기화
	m1_uiView_display_update(param);
}

/*============================================================================*/
/**
 * @brief nfc_edit_uid_gui_destroy - Destroy NFC edit UID view
 * 
 * Cleanup function for edit UID view (currently empty).
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_edit_uid_gui_destroy(uint8_t param)
{
	s_edit_uid_started = false;  // 플래그 초기화
}

/*============================================================================*/
/**
 * @brief nfc_edit_uid_gui_update - Update NFC edit UID view display
 * 
 * Handles UID editing using virtual keyboard. This function is called
 * from the view system's update cycle, allowing m1_vkbs_get_data() to
 * work properly within the message loop.
 * 
 * @param[in] param View parameter (0 = start edit)
 * @retval None
 */
/*============================================================================*/
static void nfc_edit_uid_gui_update(uint8_t param)
{
	char data_buffer[64];
	uint8_t data_size;
	uint8_t val;
	const uint8_t *pBitmap;

	// param==0일 때만 실행 (한 번만 실행되도록)
	if (param != 0 || s_edit_uid_started) {
		return;
	}
	
	s_edit_uid_started = true;  // 시작 플래그 설정
	
	nfc_run_ctx_t* c = nfc_ctx_get();
	if (!c || c->file.source_kind != LOAD_FILE) {
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		return;
	}

	// Get current UID length
	data_size = c->head.uid_len;
	if (data_size==0 || data_size > 10) {
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		return;
	}

	memset(data_buffer, 0, sizeof(data_buffer));

	// Set initial text from current UID
	m1_byte_to_hextext(c->head.uid, data_size, data_buffer);

	// Debug: Print data_buffer contents before m1_vkbs_get_data
	platformLog("[NFC Edit UID] data_size=%d, uid_len=%d\r\n", data_size, c->head.uid_len);
	platformLog("[NFC Edit UID] data_buffer='%s' (len=%d)\r\n", data_buffer, (int)strlen(data_buffer));
	platformLog("[NFC Edit UID] UID bytes: %s\r\n", hex2Str(c->head.uid, data_size));

	// Get new UID from user (hex input) - this works within view message loop
	val = m1_vkbs_get_data("Enter hex UID:", data_buffer);
	

	if (val) {
		// Validate and convert hex string to bytes
		uint8_t new_uid[10];
		memset(new_uid, 0, sizeof(new_uid));
		
		int converted = m1_strtob_with_base(data_buffer, new_uid, sizeof(new_uid), 16);
		
		// Validate length matches original
		if (converted != data_size) {
			// Show error
			pBitmap = micro_sd_card_error;
			m1_draw_icon(M1_DISP_DRAW_COLOR_TXT, 32, 0, 63, 63, pBitmap);
			uiScreen_timeout_start(UI_SCREEN_TIMEOUT, NULL);
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
			return;
		}

		// Update context
		memcpy(c->head.uid, new_uid, data_size);
		c->head.uid_len = data_size;
		nfc_ctx_refresh_ui();

		// Save to existing file
		if (c->file.path[0] != '\0') {
			// Use nfc_profile_save to save updated data
			if (nfc_profile_save(c->file.path, c)) {
				pBitmap = nfc_saved_63_63;
			} else {
				pBitmap = micro_sd_card_error;
			}
		} else {
			pBitmap = micro_sd_card_error;
		}

		m1_draw_icon(M1_DISP_DRAW_COLOR_TXT, 32, 0, 63, 63, pBitmap);
		uiScreen_timeout_start(UI_SCREEN_TIMEOUT, NULL);
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
	} else {
		// User cancelled
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
	}
}

/*============================================================================*/
/**
 * @brief nfc_edit_uid_gui_message - Process messages for NFC edit UID view
 * 
 * Handles messages from the main queue, primarily keypad events.
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_edit_uid_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_edit_uid_kp_handler();
		} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		else if ( q_item.q_evt_type==Q_EVENT_MENU_TIMEOUT )
		{
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		}
	} 

	return ret_val;
}

/*============================================================================*/
/**
 * @brief nfc_edit_uid_gui_init - Initialize and register NFC edit UID view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC edit UID view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_edit_uid_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_EDIT_UID, nfc_edit_uid_gui_create, nfc_edit_uid_gui_update, nfc_edit_uid_gui_destroy, nfc_edit_uid_gui_message);
}

/*============================================================================*/
/**
 * @brief nfc_read_more_options_delete - Delete loaded NFC card file
 * 
 * Shows confirmation dialog and deletes the loaded NFC card file.
 * 
 * @retval 0 File deleted successfully (exit signal)
 * @retval 1 User cancelled or error
 */
/*============================================================================*/
static uint8_t nfc_read_more_options_delete(void)
{
	// Wait for user input
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	// Show confirmation dialog
	char szString[128];
	char filename[64];

	nfc_run_ctx_t* c = nfc_ctx_get();
	if (!c || c->file.source_kind != LOAD_FILE) {
		return 1; // Only available for loaded files
	}

	if (c->file.path[0]=='\0') {
		return 1; // No file path
	}
	
	// Extract filename from path
	const char *pbuff = fu_get_filename(c->file.path);
	if (pbuff) {
		strncpy(filename, pbuff, sizeof(filename) - 1);
		filename[sizeof(filename) - 1] = '\0';
		fu_get_filename_without_ext(filename, filename, sizeof(filename));
	} else {
		strcpy(filename, "file");
	}

	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	u8g2_DrawStr(&m1_u8g2, 2, 12, "Delete file?");

	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	snprintf(szString, sizeof(szString), "Name: %s", filename);
	u8g2_DrawStr(&m1_u8g2, 2, 22, szString);

	u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
	// Use title_text from NFC context
	if (c->ui.title_text[0] != '\0') {
		strncpy(szString, c->ui.title_text, sizeof(szString) - 1);
		szString[sizeof(szString) - 1] = '\0';
	} else {
		strcpy(szString, "NFC");
	}
	u8g2_DrawStr(&m1_u8g2, 2, 32, szString);

	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	snprintf(szString, sizeof(szString), "UID: %s", hex2Str(c->head.uid, c->head.uid_len));
	u8g2_DrawStr(&m1_u8g2, 2, 42, szString);

	// Bottom bar: LEFT=Cancel, RIGHT=Delete
	m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Cancel", "Delete", arrowright_8x8);
	m1_u8g2_nextpage();

	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if (q_item.q_evt_type==Q_EVENT_KEYPAD)
			{
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if (ret==pdTRUE)
				{
					if (this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK)
					{
						// Confirm delete
						uint8_t delete_ret = m1_fb_delete_file(c->file.path);
						if (delete_ret==0) {
							// Show success message
							u8g2_FirstPage(&m1_u8g2);
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
							u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
							u8g2_DrawStr(&m1_u8g2, 2, 12, "Delete");
							u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
							u8g2_DrawStr(&m1_u8g2, 10, 22, "Delete success");
							m1_u8g2_nextpage();
							uiScreen_timeout_start(UI_SCREEN_TIMEOUT, NULL);

							return 0; // Exit signal
						} else {
							// Show error
							u8g2_FirstPage(&m1_u8g2);
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
							u8g2_DrawXBMP(&m1_u8g2, 32, 0, 63, 63, micro_sd_card_error);
							m1_u8g2_nextpage();
							uiScreen_timeout_start(UI_SCREEN_TIMEOUT, NULL);

							return 1;
						}
					} // else if (this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK)
					else if ( this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK ||
							this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
					{
						// Cancel
						return 1;
					}
				} // if (ret==pdTRUE)
			} // if (q_item.q_evt_type==Q_EVENT_KEYPAD)
		} // if (ret==pdTRUE)
	} // while (1)

	return 1;
} 


/*============================================================================*/
/**
 * @brief nfc_rename_kp_handler - Handle keypad input for NFC rename view
 * 
 * Processes button events in the NFC rename view:
 * - BACK: Return to submenu
 * 
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_rename_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		{
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		}
	}
	return 1;
}

/*============================================================================*/
/**
 * @brief nfc_rename_gui_create - Create and initialize NFC rename view
 * 
 * Initializes the rename view and triggers an update.
 * 
 * @param[in] param View parameter
 * @retval None
 */
/*============================================================================*/
static void nfc_rename_gui_create(uint8_t param)
{
	m1_uiView_display_update(param);
}

/*============================================================================*/
/**
 * @brief nfc_rename_gui_destroy - Destroy NFC rename view
 * 
 * Cleanup function for rename view (currently empty).
 * 
 * @param[in] param View parameter (unused)
 * @retval None
 */
/*============================================================================*/
static void nfc_rename_gui_destroy(uint8_t param)
{
	;
}

/*============================================================================*/
/**
 * @brief nfc_rename_gui_update - Update NFC rename view display
 * 
 * Handles file renaming using virtual keyboard. This function is called
 * from the view system's update cycle, allowing m1_vkb_get_filename() to
 * work properly within the message loop.
 * 
 * @param[in] param View parameter (0 = start rename)
 * @retval None
 */
/*============================================================================*/
static void nfc_rename_gui_update(uint8_t param)
{
	nfc_run_ctx_t* c = nfc_ctx_get();
	if (!c || c->file.source_kind != LOAD_FILE) {
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		return;
	}

	if (c->file.path[0]=='\0') {
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		return;
	}

	char new_file[256];  // Increased buffer size to avoid truncation warning
	char old_file[128];
	char fname[50];
	char dname[50];
	uint8_t ret;
	const uint8_t *pBitmap;

	// Extract current filename without extension
	// fu_get_filename_without_ext takes full path and extracts filename without extension
	if (c->file.path[0] != '\0') {
		fu_get_filename_without_ext(c->file.path, dname, sizeof(dname));
	} else {
	    srand(HAL_GetTick());
	    	sprintf((char*)dname, "nfc_%05u", rand() % 0xFFFFF);
	}

	// Get new filename from user
			ret = m1_vkb_get_filename("Enter filename:", (char*)dname, (char*)fname);
	if (!ret) {
		// User escaped
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		return;
	}

	// Build new file path
	strcpy(old_file, c->file.path);
	
	// Get directory path from old file
	char dir_path[128];
	fu_get_directory_path(old_file, dir_path, sizeof(dir_path));
	
	// Combine directory + new filename + extension
	// Check return value to ensure no truncation
	int snprintf_ret = snprintf(new_file, sizeof(new_file), "%s/%s%s", dir_path, fname, NFC_FILE_EXTENSION);
	if (snprintf_ret < 0 || snprintf_ret >= (int)sizeof(new_file)) {
		// Path truncated or error occurred
		pBitmap = micro_sd_card_error;
		m1_draw_icon(M1_DISP_DRAW_COLOR_TXT, 32, 0, 63, 63, pBitmap);
		uiScreen_timeout_start(UI_SCREEN_TIMEOUT, NULL);
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		return;
	}

	// Check if new file already exists
	if (m1_fb_check_existence(new_file)) {
		// Show error - file exists
		pBitmap = micro_sd_card_error;
		m1_draw_icon(M1_DISP_DRAW_COLOR_TXT, 32, 0, 63, 63, pBitmap);
		uiScreen_timeout_start(UI_SCREEN_TIMEOUT, NULL);
		m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		return;
	}

	// Rename file
	pBitmap = micro_sd_card_error;
	FRESULT res = f_rename(old_file, new_file);
	if (res==FR_OK) {
		pBitmap = nfc_saved_63_63;
		// Update context with new path
		strncpy(c->file.path, new_file, sizeof(c->file.path) - 1);
		c->file.path[sizeof(c->file.path) - 1] = '\0';
	}

	m1_draw_icon(M1_DISP_DRAW_COLOR_TXT, 32, 0, 63, 63, pBitmap);
	uiScreen_timeout_start(UI_SCREEN_TIMEOUT, NULL);
	m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
}

/*============================================================================*/
/**
 * @brief nfc_rename_gui_message - Process messages for NFC rename view
 * 
 * Handles messages from the main queue, primarily keypad events.
 * 
 * @retval 0 Exit requested
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_rename_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_rename_kp_handler();
		} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		else if ( q_item.q_evt_type==Q_EVENT_MENU_TIMEOUT )
		{
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_REFRESH);
		}
	} 

	return ret_val;
}

/*============================================================================*/
/**
 * @brief nfc_rename_gui_init - Initialize and register NFC rename view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC rename view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_rename_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_RENAME, nfc_rename_gui_create, nfc_rename_gui_update, nfc_rename_gui_destroy, nfc_rename_gui_message);
}

/*============================================================================*/
/**
 * @brief nfc_saved_browse_kp_handler - Handle keypad input for NFC saved browse view
 * 
 * Processes button events in the NFC saved browse view:
 * - BACK: Exit to IDLE
 * 
 * @retval 0 Exit
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_saved_browse_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;

	if(xQueueReceive(button_events_q_hdl, &this_button_status, 0) != pdTRUE)
		return 1;

	if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // user wants to exit?
	{
		; // Do extra tasks here if needed
		m1_uiView_display_switch(VIEW_MODE_IDLE, 0);
		xQueueReset(main_q_hdl); // Reset main q before return
		return 0;
	} // if ( m1_buttons_status[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )

	return 1;
}

/*============================================================================*/
/**
 * @brief nfc_saved_browse_gui_create - Create and initialize NFC saved browse view
 * 
 * Initializes the saved browse view and triggers an update.
 * 
 * @param[in] param View parameter
 * @retval None
 */
/*============================================================================*/
static void nfc_saved_browse_gui_create(uint8_t param)
{
	m1_uiView_display_update(0);
}

/*============================================================================*/
/**
 * @brief nfc_saved_browse_gui_destroy - Destroy NFC saved browse view
 * 
 * Cleanup function for the saved browse view.
 * 
 * @param[in] param View parameter
 * @retval None
 */
/*============================================================================*/
static void nfc_saved_browse_gui_destroy(uint8_t param)
{

}

/*============================================================================*/
/**
 * @brief nfc_saved_browse_gui_update - Update NFC saved browse view
 * 
 * Handles file browsing and loading. When a file is selected:
 * - Validates file extension (.nfc)
 * - Loads file using nfc_storage_load_file()
 * - On success: switches to submenu view
 * - On failure: shows error message and returns to browse
 * 
 * @param[in] param View parameter
 * @retval None
 */
/*============================================================================*/
static void nfc_saved_browse_gui_update(uint8_t param)
{
	f_info = storage_browse("0:/NFC");
	if ( f_info->file_is_selected )
	{
		if(nfc_profile_load(f_info, NFC_FILE_EXTENSION_TMP))
		{
			m1_uiView_display_switch(VIEW_MODE_NFC_READ_MORE, X_MENU_UPDATE_RESET);
		}
		else
		{
			m1_message_box(&m1_u8g2, res_string(IDS_UNSUPPORTED_FILE_), " ", NULL, res_string(IDS_BACK));
			m1_uiView_display_switch(VIEW_MODE_NFC_SAVED_BROWSE, 0);
		}
	} // if ( f_info->file_is_selected )
	else	// user escaped?
	{
	    m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);
	}
}

/*============================================================================*/
/**
 * @brief nfc_saved_browse_gui_message - Handle messages for NFC saved browse view
 * 
 * Processes messages from the main queue:
 * - Q_EVENT_KEYPAD: Handles button events
 * - Q_EVENT_MENU_EXIT: Exits to IDLE
 * 
 * @retval 0 Exit
 * @retval 1 Continue processing
 */
/*============================================================================*/
static int nfc_saved_browse_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = nfc_saved_browse_kp_handler();
		} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		else if(q_item.q_evt_type==Q_EVENT_MENU_EXIT)
		{
			m1_uiView_display_switch(VIEW_MODE_IDLE, 0);
			xQueueReset(main_q_hdl); // Reset main q before return
			ret_val = 0;
		}
	} // if (ret==pdTRUE)

	return ret_val;
}

/*============================================================================*/
/**
 * @brief nfc_saved_browse_gui_init - Initialize and register NFC saved browse view functions
 * 
 * Registers the view functions (create, update, destroy, message) for
 * the NFC saved browse view mode.
 * 
 * @retval None
 */
/*============================================================================*/
void nfc_saved_browse_gui_init(void)
{
   m1_uiView_functions_register(VIEW_MODE_NFC_SAVED_BROWSE, nfc_saved_browse_gui_create, nfc_saved_browse_gui_update, nfc_saved_browse_gui_destroy, nfc_saved_browse_gui_message);
}

