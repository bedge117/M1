/* See COPYING.txt for license details. */

/*
*
*  m1_sub_ghz.c
*
*  M1 sub-ghz functions
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_tasks.h"
#include "m1_sub_ghz_api.h"
//#include "m1_sub_ghz.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_ring_buffer.h"
#include "m1_storage.h"
#include "m1_sdcard_man.h"
#include "flipper_subghz.h"
#include "uiView.h"

/*************************** D E F I N E S ************************************/

#define SUBGHZ_RAW_DATA_SAMPLES_MAX			64000 // data type of sample: uint16_t

#define SUBGHZ_TX_RAW_REPLAY_REPEAT_DEFAULT		0 // 2 plus the first transmit before repeating

#define SI4463_nIRQ_EXTI_IRQn         	EXTI12_IRQn

#define NOISE_FLOOR_RSSI_THRESHOLD		-110 //dBm
#define SIGNAL_TO_NOISE_RATIO			20 //dB

#define CHANNEL_STEPS_MAX				256
#define CHANNEL_STEP					(float)0.25 // MHz

#define SUB_GHZ_433_92_NEW_PDTC			0x6C // Attack and decay times of the OOK Peak Detector - Default: 0x28

#define M1_LOGDB_TAG				"Sub-GHz"

#define SUB_GHZ_FILEPATH			"/SUBGHZ"
#define SUB_GHZ_FILE_EXTENSION		".sgh"
#define SUB_GHZ_FILE_PREFIX			"sghz_"

#define SUB_GHZ_DATAFILE_KEY_FORMAT_N		9
#define SUB_GHZ_DATAFILE_RAW_FORMAT_N		4
#define SUB_GHZ_DATAFILE_DATA_KEYWORD		"Data:"
#define SUB_GHZ_DATAFILE_FILETYPE_NOISE		"NOISE"
#define SUB_GHZ_DATAFILE_FILETYPE_PACKET	"PACKET"
#define SUB_GHZ_DATAFILE_FILETYPE_KEYWORD	SUB_GHZ_DATAFILE_FILETYPE_NOISE

#define SUB_GHZ_RAW_DATA_PARSER_ERROR_MASK	0x80
#define SUB_GHZ_RAW_DATA_PARSER_ERROR_L1	0x81
#define SUB_GHZ_RAW_DATA_PARSER_ERROR_L2	0x82
#define SUB_GHZ_RAW_DATA_PARSER_ERROR_L3	0x83
#define SUB_GHZ_RAW_DATA_PARSER_ERROR_L4	0x84
#define SUB_GHZ_RAW_DATA_PARSER_ERROR_L5	0x85

#define SUB_GHZ_RAW_DATA_PARSER_STOPPED		0x40
#define SUB_GHZ_RAW_DATA_PARSER_IDLE		0x0A
#define SUB_GHZ_RAW_DATA_PARSER_READY		0x0B
#define SUB_GHZ_RAW_DATA_PARSER_COMPLETE	0x0C


#define SUBGHZ_ISM_BANDS_LIST_NA			3
#define SUBGHZ_ISM_BANDS_LIST_EU			3
#define SUBGHZ_ISM_BANDS_LIST_ASIA			2

#define	SUBGHZ_FCC_BASE_FREQ_300_000			(float)300.00001
#define	SUBGHZ_FCC_BASE_FREQ_310_000			(float)310.00001
#define	SUBGHZ_FCC_BASE_FREQ_315_000			(float)315.00001
#define	SUBGHZ_FCC_BASE_FREQ_345_000			(float)345.00001
#define	SUBGHZ_FCC_BASE_FREQ_372_000			(float)372.00001
#define	SUBGHZ_FCC_BASE_FREQ_390_000			(float)390.00001
#define	SUBGHZ_FCC_BASE_FREQ_433_000			(float)433.00001
#define	SUBGHZ_FCC_BASE_FREQ_433_920			(float)433.92001
#define SUBGHZ_FCC_BASE_FREQ_915_000			(float)915.00001
#define SUBGHZ_FCC_BASE_FREQ_150_000			(float)150.00001
#define SUBGHZ_FCC_BASE_FREQ_200_000			(float)200.00001
#define SUBGHZ_FCC_BASE_FREQ_250_000			(float)250.00001

// Reference: FCC 15.205 Restricted bands of operation
// 322MHz-335.4MHz, 399.9MHz-410MHz
//#define SUBGHZ_FCC_ISM_BAND_304_100				(float)304.10001 // FZ
#define SUBGHZ_FCC_ISM_BAND_300_000				(float)300.00001
#define SUBGHZ_FCC_ISM_BAND_310_000				(float)310.00001
#define SUBGHZ_FCC_ISM_BAND_321_950				(float)321.95001

#define SUBGHZ_FCC_ISM_BAND_344_000				(float)344.00001
#define SUBGHZ_FCC_ISM_BAND_392_000				(float)392.00001

#define SUBGHZ_FCC_ISM_BAND_433_050				(float)433.05001
#define SUBGHZ_FCC_ISM_BAND_434_790				(float)434.79001

#define SUBGHZ_FCC_ISM_BAND_915_000				(float)915.00001
#define SUBGHZ_FCC_ISM_BAND_928_000				(float)928.00001

//************************** C O N S T A N T **********************************/

const char *subghz_ism_regions_text[SUBGHZ_ISM_BAND_REGIONS_LIST] =
{
	"North America",
	"Europe",
	"Asia"
};

static const char *subghz_modulation_text[SUBGHZ_MODULATION_LIST] =
{
	"OOK",
	"ASK",
	"FSK"
};

static const char *subghz_band_text[] =
{
	"300.000",
	"310.000",
	"315.000",
	"345.000",
	"372.000",
	"390.000",
	"433.000",
	"433.920",
	"915.000",
	"150.000",
	"200.000",
	"250.000"
};

static const char *subghz_datfile_keywords[SUB_GHZ_DATAFILE_KEY_FORMAT_N] =
{
	"Filetype:",
	"Version:",
	"Frequency:",
	"Modulation:",
	"Protocol:",
	"Bits:",
	"Payload:",
	"BT:",
	"IT:"
};

static const float subghz_band_steps[SUB_GHZ_BAND_EOL][2] =
{
	{SUBGHZ_FCC_BASE_FREQ_300_000, 4}, 	// 300.000 - 301.000, step = 250KHz //4
	{SUBGHZ_FCC_BASE_FREQ_310_000, 40}, // 310.000 - 320.000, step = 250KHz //40
	{SUBGHZ_FCC_BASE_FREQ_315_000, 0}, 	// 315.000 - not used // 0
	{SUBGHZ_FCC_BASE_FREQ_345_000, 4},	// 345.000 - 346.000 // 4
	{SUBGHZ_FCC_BASE_FREQ_372_000, 4},	// 372.000 - 373.000 // 4
	{SUBGHZ_FCC_BASE_FREQ_390_000, 4},	// 390.000 - 391.000 // 4
	{SUBGHZ_FCC_BASE_FREQ_433_000, 3},	// 433.000 - 435.000 // 8
	{SUBGHZ_FCC_BASE_FREQ_433_920, 2}, 	// 433.920 - not used // 0
	{SUBGHZ_FCC_BASE_FREQ_915_000, 4}, 	// 915.000 - 916.000 // 4
	{SUBGHZ_FCC_BASE_FREQ_150_000, 4},	// 150.000 - 151.000
	{SUBGHZ_FCC_BASE_FREQ_200_000, 4},	// 200.000 - 201.000
	{SUBGHZ_FCC_BASE_FREQ_250_000, 4}	// 250.000 - 251.000
};

static const float subghz_fcc_ism_bands_NA[SUBGHZ_ISM_BANDS_LIST_NA][2] =
{
	{SUBGHZ_FCC_ISM_BAND_310_000, SUBGHZ_FCC_ISM_BAND_321_950}, 	// 304.10MHz - 321.95MHz
	/*{SUBGHZ_FCC_ISM_BAND_344_000, SUBGHZ_FCC_ISM_BAND_392_000},		// 344.00MHz - 392.00MHz*/
	{SUBGHZ_FCC_ISM_BAND_433_050, SUBGHZ_FCC_ISM_BAND_434_790}, 	// 433.05MHz - 434.79MHz
	{SUBGHZ_FCC_ISM_BAND_915_000, SUBGHZ_FCC_ISM_BAND_928_000}		// 915.00MHz - 928.00MHz
};

static const float subghz_fcc_ism_bands_EU[SUBGHZ_ISM_BANDS_LIST_EU][2] =
{
	{SUBGHZ_FCC_ISM_BAND_310_000, SUBGHZ_FCC_ISM_BAND_321_950}, 	// 304.10MHz - 321.95MHz
	/*{SUBGHZ_FCC_ISM_BAND_344_000, SUBGHZ_FCC_ISM_BAND_392_000},		// 344.00MHz - 392.00MHz*/
	{SUBGHZ_FCC_ISM_BAND_433_050, SUBGHZ_FCC_ISM_BAND_434_790}, 	// 433.05MHz - 434.79MHz
	{SUBGHZ_FCC_ISM_BAND_915_000, SUBGHZ_FCC_ISM_BAND_928_000}		// 915.00MHz - 928.00MHz
};

static const float subghz_fcc_ism_bands_ASIA[SUBGHZ_ISM_BANDS_LIST_ASIA][2] =
{
	{SUBGHZ_FCC_ISM_BAND_310_000, SUBGHZ_FCC_ISM_BAND_321_950}, 	// 304.10MHz - 321.95MHz
	/*{SUBGHZ_FCC_ISM_BAND_344_000, SUBGHZ_FCC_ISM_BAND_392_000},		// 344.00MHz - 392.00MHz*/
	/*{SUBGHZ_FCC_ISM_BAND_433_050, SUBGHZ_FCC_ISM_BAND_434_790}, 	// 433.05MHz - 434.79MHz*/
	{SUBGHZ_FCC_ISM_BAND_915_000, SUBGHZ_FCC_ISM_BAND_928_000}		// 915.00MHz - 928.00MHz
};

//typedef struct S_M1_SUBGHZ_ISM_REGIONS_t;
typedef struct
{
	float (*this_region)[2];
	uint8_t bands_list;
} S_M1_SUBGHZ_ISM_REGIONS_t;

const S_M1_SUBGHZ_ISM_REGIONS_t subghz_regions_list[SUBGHZ_ISM_BAND_REGIONS_LIST] =
{
	{	.this_region = subghz_fcc_ism_bands_NA,
		.bands_list = SUBGHZ_ISM_BANDS_LIST_NA
	},
	{	.this_region = subghz_fcc_ism_bands_EU,
		.bands_list = SUBGHZ_ISM_BANDS_LIST_EU
	},
	{	.this_region = subghz_fcc_ism_bands_ASIA,
		.bands_list = SUBGHZ_ISM_BANDS_LIST_ASIA
	},
};

/* Frequency-sorted band order for UI navigation (L/R buttons) */
#define SUBGHZ_BAND_ORDER_COUNT		12
static const S_M1_SubGHz_Band subghz_band_order[SUBGHZ_BAND_ORDER_COUNT] = {
	SUB_GHZ_BAND_150, SUB_GHZ_BAND_200, SUB_GHZ_BAND_250,
	SUB_GHZ_BAND_300, SUB_GHZ_BAND_310, SUB_GHZ_BAND_315,
	SUB_GHZ_BAND_345, SUB_GHZ_BAND_372, SUB_GHZ_BAND_390,
	SUB_GHZ_BAND_433, SUB_GHZ_BAND_433_92, SUB_GHZ_BAND_915
};

static uint8_t subghz_band_order_find(S_M1_SubGHz_Band band)
{
	for (uint8_t i = 0; i < SUBGHZ_BAND_ORDER_COUNT; i++)
		if (subghz_band_order[i] == band) return i;
	return 3; /* default to 300 MHz position */
}

/* TX power setting (shared between Radio Settings UI and Record/Replay TX calls) */
#define TX_POWER_LEVELS          4
static const uint8_t tx_power_values[TX_POWER_LEVELS] = { 10, 40, 80, 127 };
static const char *tx_power_labels[TX_POWER_LEVELS] = { "Low", "Med", "High", "Max" };
static uint8_t subghz_tx_power_idx = 3;  /* Default: Max */

/* Custom frequency for SUB_GHZ_BAND_CUSTOM mode */
static uint32_t subghz_custom_freq_hz = 433920000UL;

/* Last decoded protocol info during Record (for overlay + .sub save) */
static SubGHz_Dec_Info_t subghz_record_last_decoded;
static bool subghz_record_has_decoded = false;

//************************** S T R U C T U R E S *******************************

typedef enum {
	SUB_GHZ_RECORD_IDLE = 0,
	SUB_GHZ_RECORD_ACTIVE,
	SUB_GHZ_RECORD_STANDBY,
	SUB_GHZ_RECORD_REPLAY,
	SUB_GHZ_RECORD_UNKNOWN
} S_M1_SubGHz_Record_t;

typedef enum {
    VIEW_MODE_SUBGHZ_RECORD_NONE = 0,
    VIEW_MODE_SUBGHZ_RECORD,
    VIEW_MODE_SUBGHZ_RECORD_EOL
} S_M1_SubGHz_View_Mode_Record_t;

typedef enum {
    VIEW_MODE_SUBGHZ_REPLAY_NONE = 0,
    VIEW_MODE_SUBGHZ_REPLAY_BROWSE,
	VIEW_MODE_SUBGHZ_REPLAY_PLAY,
    VIEW_MODE_SUBGHZ_REPLAY_EOL
} S_M1_SubGHz_View_Mode_Replay_t;

typedef enum {
	SUBGHZ_RECORD_DISPLAY_PARAM_READY = 0,
	SUBGHZ_RECORD_DISPLAY_PARAM_ACTIVE,
	SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE,
	SUBGHZ_RECORD_DISPLAY_PARAM_PLAY,
	SUBGHZ_RECORD_DISPLAY_PARAM_SAVE,
	SUBGHZ_RECORD_DISPLAY_PARAM_RESET,
	SUBGHZ_RECORD_DISPLAY_PARAM_MEM_ERROR,
	SUBGHZ_RECORD_DISPLAY_PARAM_SDCARD_ERROR,
	SUBGHZ_RECORD_DISPLAY_PARAM_SYS_ERROR
} S_M1_SubGHz_Record_Display_Param_t;

typedef enum {
	SUBGHZ_REPLAY_DISPLAY_PARAM_ACTIVE = 0,
	SUBGHZ_REPLAY_DISPLAY_PARAM_PLAY,
	SUBGHZ_REPLAY_DISPLAY_PARAM_SYS_ERROR
} S_M1_SubGHz_Replay_Display_Param_t;

/***************************** V A R I A B L E S ******************************/

TIM_HandleTypeDef   timerhdl_subghz_tx;
TIM_HandleTypeDef   timerhdl_subghz_rx;
EXTI_HandleTypeDef 	si4463_exti_hdl;
DMA_HandleTypeDef hdma_subghz_tx;

S_M1_RingBuffer subghz_rx_rawdata_rb;
static uint16_t *subghz_front_buffer = NULL;
static uint16_t subghz_front_buffer_size = 0;
static uint8_t *subghz_ring_read_buffer = NULL;
static uint8_t *subghz_sdcard_write_buffer = NULL;
static uint8_t *sdcard_dat_buffer = NULL;
static uint8_t *sdcard_dat_buffer_base = NULL; // Original malloc pointer (before offset)
static uint8_t *sdcard_buffer_run_ptr = NULL;
static uint8_t double_buffer_ptr_id = 0;
static uint16_t raw_samples_count;
static uint16_t sdcard_dat_read_size;
static uint16_t raw_samples_buffer_size;
static uint16_t subghz_back_buffer_size;
static uint16_t *subghz_back_buffer = NULL;
static uint16_t *double_buffer_ptr[2];
static uint32_t sdcard_dat_file_size, sdcard_dat_buffer_end_pos;
uint8_t subghz_tx_tc_flag;
uint8_t subghz_record_mode_flag = 0;
static uint8_t subghz_uiview_gui_latest_param;
static uint8_t subghz_replay_ret_code;
static uint8_t subghz_replay_mod;
static uint8_t subghz_replay_band, subghz_replay_channel;
static float subghz_replay_freq;
static S_M1_file_info *f_info = NULL;
static S_M1_SDM_DatFileInfo_t datfile_info;
S_M1_Q_Union_t *subghz_rx_q = NULL;
S_M1_SubGHz_Scan_Config subghz_scan_config =
{
	.band = SUB_GHZ_BAND_300,
	.modulation = MODULATION_OOK
};

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

void menu_sub_ghz_init(void);
void menu_sub_ghz_exit(void);

void sub_ghz_init(void);
void sub_ghz_record(void);
void sub_ghz_replay(void);
void sub_ghz_frequency_reader(void);
void sub_ghz_regional_information(void);
void sub_ghz_radio_settings(void);

static void sub_ghz_rx_init(void);
static void sub_ghz_rx_start(void);
static void sub_ghz_rx_pause(void);
static void sub_ghz_rx_deinit(void);
static uint8_t sub_ghz_ring_buffers_init(void);
static void sub_ghz_ring_buffers_deinit(void);
static uint8_t sub_ghz_rx_raw_save(bool header_init, bool last_data);
static void sub_ghz_tx_raw_init(void);
static void sub_ghz_tx_raw_deinit(void);

static void sub_ghz_set_opmode(uint8_t opmode, uint8_t band, uint8_t channel, uint8_t tx_power);
static void sub_ghz_display(SubGHz_Dec_Info_t decoded_data);
static uint8_t sub_ghz_raw_samples_init(void);
static void sub_ghz_raw_samples_deinit(bool discard_samples);
static void sub_ghz_transmit_raw(uint32_t source, uint32_t dest, uint32_t len, uint8_t repeat);
static void sub_ghz_transmit_raw_restart(uint32_t source, uint32_t len);
static uint8_t sub_ghz_raw_replay_init(void);
static void sub_ghz_raw_tx_stop(void);
static uint8_t sub_ghz_replay_start(bool record_mode, S_M1_SubGHz_Band band, uint8_t channel, uint8_t power);
static uint8_t sub_ghz_replay_continue(uint8_t ret_code_in);
static uint8_t sub_ghz_fcc_ism_band_check(uint8_t band, uint8_t channel);
static void sub_ghz_buffer_rotate(S_M1_RingBuffer *prb_handle);
static uint8_t sub_ghz_parse_raw_data(uint8_t buffer_ptr_id);
static uint8_t sub_ghz_file_load(void);

static bool sub_ghz_custom_freq_entry(void);

static void subghz_record_gui_init(void);
static void subghz_record_gui_create(uint8_t param);
static void subghz_record_gui_destroy(uint8_t param);
static void subghz_record_gui_update(uint8_t param);
static int subghz_record_gui_message(void);
static int subghz_record_kp_handler(void);

static void subghz_replay_browse_gui_init(void);
static void subghz_replay_browse_gui_create(uint8_t param);
static void subghz_replay_browse_gui_destroy(uint8_t param);
static void subghz_replay_browse_gui_update(uint8_t param);
static int  subghz_replay_browse_gui_message(void);
static int subghz_replay_browse_kp_handler(void);

static void subghz_replay_play_gui_init(void);
static void subghz_replay_play_gui_create(uint8_t param);
static void subghz_replay_play_gui_destroy(uint8_t param);
static void subghz_replay_play_gui_update(uint8_t param);
static int  subghz_replay_play_gui_message(void);
static int subghz_replay_play_kp_handler(void);

//************************** C O N S T A N T **********************************/

static const view_func_t view_subghz_record_table[] = {
    NULL,               			// Empty
    subghz_record_gui_init,      	// VIEW_MODE_SUBGHZ_RECORD
};

static const view_func_t view_subghz_replay_table[] = {
    NULL,               			// Empty
    subghz_replay_browse_gui_init,  // VIEW_MODE_SUBGHZ_REPLAY_BROWSE
	subghz_replay_play_gui_init,  	// VIEW_MODE_SUBGHZ_REPLAY_PLAY
};

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/
/*
uint16_t *q = 0;
arrpush(q, 1);
arrpush(q, 2);
for(int i = 0; i < arrlen(q); i++)
    printf("%g\n", q[i]);
arrfree(q);
*/
/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void menu_sub_ghz_init(void)
{
	// First radio init with reset (do_reset is true) will load the radio patch SI446X_PATCH_CMDS!
	// Other calls to radio_init_rx_tx() will load new configuration data only!
	radio_init_rx_tx(SUB_GHZ_BAND_300, MODEM_MOD_TYPE_OOK, true);
} // void menu_sub_ghz_init(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void menu_sub_ghz_exit(void)
{
	// Default IO setting for the SI4463
	HAL_GPIO_WritePin(SI4463_CS_GPIO_Port, SI4463_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SI4463_ENA_GPIO_Port, SI4463_ENA_Pin, GPIO_PIN_RESET);
} // void menu_sub_ghz_exit(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void sub_ghz_init(void)
{
	GPIO_InitTypeDef gpio_init_structure;

	si4463_exti_hdl.Line = EXTI_LINE_12;
	si4463_exti_hdl.RisingCallback = NULL;
	si4463_exti_hdl.FallingCallback = NULL;

	/* Configure Interrupt mode for SI4463_nINT pin */
	gpio_init_structure.Pin = SI4463_nINT_Pin;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
	gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
	HAL_GPIO_Init(SI4463_nINT_GPIO_Port, &gpio_init_structure);

	/* Enable and set SI4463_nINT Interrupt to the low priority */
	HAL_NVIC_SetPriority((IRQn_Type)(SI4463_nIRQ_EXTI_IRQn), configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
	HAL_NVIC_EnableIRQ((IRQn_Type)(SI4463_nIRQ_EXTI_IRQn));
} // void sub_ghz_init(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void sub_ghz_set_opmode(uint8_t opmode, uint8_t band, uint8_t channel, uint8_t tx_power)
{
	uint8_t mod_type;
	S_M1_SubGHz_Band init_freq;   /* Band used to load radio config */
	uint32_t retune_freq_hz = 0;  /* Non-zero → retune after init */

	switch(band)
	{
		case SUB_GHZ_BAND_300:
		case SUB_GHZ_BAND_310:
		case SUB_GHZ_BAND_315:
		case SUB_GHZ_BAND_345:
		case SUB_GHZ_BAND_372:
		case SUB_GHZ_BAND_390:
		case SUB_GHZ_BAND_433:
		case SUB_GHZ_BAND_433_92:
			init_freq = band;
			mod_type = MODEM_MOD_TYPE_OOK;
			break;

		case SUB_GHZ_BAND_915:
			init_freq = band;
			mod_type = MODEM_MOD_TYPE_FSK;
			break;

		case SUB_GHZ_BAND_150:
			init_freq = SUB_GHZ_BAND_300;
			retune_freq_hz = 150000000UL;
			mod_type = MODEM_MOD_TYPE_OOK;
			break;

		case SUB_GHZ_BAND_200:
			init_freq = SUB_GHZ_BAND_300;
			retune_freq_hz = 200000000UL;
			mod_type = MODEM_MOD_TYPE_OOK;
			break;

		case SUB_GHZ_BAND_250:
			init_freq = SUB_GHZ_BAND_300;
			retune_freq_hz = 250000000UL;
			mod_type = MODEM_MOD_TYPE_OOK;
			break;

		case SUB_GHZ_BAND_CUSTOM:
			if (subghz_custom_freq_hz >= 850000000UL)
			{
				init_freq = SUB_GHZ_BAND_915;
				mod_type = MODEM_MOD_TYPE_FSK;
			}
			else if (subghz_custom_freq_hz >= 420000000UL)
			{
				init_freq = SUB_GHZ_BAND_433;
				mod_type = MODEM_MOD_TYPE_OOK;
			}
			else
			{
				init_freq = SUB_GHZ_BAND_300;
				mod_type = MODEM_MOD_TYPE_OOK;
			}
			retune_freq_hz = subghz_custom_freq_hz;
			break;

		default:
			init_freq = SUB_GHZ_BAND_300;
			mod_type = MODEM_MOD_TYPE_OOK;
			break;
	} // switch(band)

	radio_init_rx_tx(init_freq, mod_type, SI446x_Get_Reset_Stat());
	SI446x_Select_Frontend((band == SUB_GHZ_BAND_CUSTOM) ? init_freq : band);

	if (retune_freq_hz)
		SI446x_Set_Frequency(retune_freq_hz);

	M1_LOG_D(M1_LOGDB_TAG, "Rx_Tx mode %d band %d channel %d\r\n", opmode, band, channel);

	switch (opmode)
	{
		case SUB_GHZ_OPMODE_RX:
			radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);
			// Put the radio in Rx mode
			SI446x_Start_Rx(channel);
			break;

		case SUB_GHZ_OPMODE_TX:
			radio_set_antenna_mode(RADIO_ANTENNA_MODE_TX);
			// Read INTs, clear pending ones
			SI446x_Get_IntStatus(0, 0, 0);
			// Direct mode asynchronous mode, TX direct mode on GPIO2,  modulation is sourced in real-time, OOK
			// Mode: TX_DIRECT_MODE_TYPE[7]	TX_DIRECT_MODE_GPIO[6:5]	MOD_SOURCE[4:3]	MOD_TYPE[2:0]
			//					1					10						01				000
			SI446x_Change_ModType(0xC8 | mod_type);
			// Read INTs, clear pending ones
			SI446x_Get_IntStatus(0, 0, 0);
			/* Start sending packet, channel 0, START immediately */
			Radio_Start_Tx(channel, START_TX_COMPLETE_STATE_NOCHANGE, 0); // Do not change state after completion of the packet transmission
			//SI446x_Start_Tx_CW(uint8_t channel);
			SI446x_Set_Tx_Power(tx_power);
			break;

		default: // SUB_GHZ_OPMODE_ISOLATED
			// Put the radio in sleep mode
			SI446x_Change_State(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
			radio_set_antenna_mode(RADIO_ANTENNA_MODE_ISOLATED);
			break;
	} // switch (opmode)

} // static void sub_ghz_set_opmode(uint8_t opmode, uint8_t band, uint8_t channel, uint8_t tx_power)



/*============================================================================*/
/**
  * @brief  Custom frequency digit-by-digit entry
  *         UP/DOWN change digit, L/R move cursor, OK confirm, BACK cancel
  * @return true if frequency was entered, false if cancelled
  */
/*============================================================================*/
static bool sub_ghz_custom_freq_entry(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	char freq_str[12]; /* "XXXX.XXX\0" */
	uint8_t digits[7]; /* 4 integer + 3 fractional digits */
	uint8_t cursor = 0;
	bool done = false;
	bool accepted = false;

	/* Initialize from current custom freq */
	uint32_t mhz = subghz_custom_freq_hz / 1000000UL;
	uint32_t khz = (subghz_custom_freq_hz % 1000000UL) / 1000UL;
	digits[0] = (mhz / 1000) % 10;
	digits[1] = (mhz / 100) % 10;
	digits[2] = (mhz / 10) % 10;
	digits[3] = mhz % 10;
	digits[4] = (khz / 100) % 10;
	digits[5] = (khz / 10) % 10;
	digits[6] = khz % 10;

	while (!done)
	{
		/* Build display string */
		snprintf(freq_str, sizeof(freq_str), "%d%d%d%d.%d%d%d",
		         digits[0], digits[1], digits[2], digits[3],
		         digits[4], digits[5], digits[6]);

		u8g2_FirstPage(&m1_u8g2);
		do {
			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
			u8g2_DrawStr(&m1_u8g2, 10, 12, "Enter Frequency (MHz)");

			/* Draw frequency string in large font */
			u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
			u8g2_DrawStr(&m1_u8g2, 8, 38, freq_str);

			/* Draw cursor underline under active digit */
			/* Each large font char is roughly 10px wide, dot is ~5px */
			uint8_t cursor_x = 8;
			for (uint8_t c = 0; c < cursor; c++)
			{
				if (c == 4) cursor_x += 5; /* skip past the dot before fractional */
				cursor_x += 10;
			}
			if (cursor >= 4) cursor_x += 5; /* dot offset for fractional digits */
			u8g2_DrawHLine(&m1_u8g2, cursor_x, 40, 8);

			/* Controls hint */
			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
			u8g2_DrawStr(&m1_u8g2, 0, 56, "\x18\x19:Digit L/R:Move OK:Set");
		} while (u8g2_NextPage(&m1_u8g2));

		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
		{
			xQueueReceive(button_events_q_hdl, &this_button_status, 0);

			if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
			{
				digits[cursor] = (digits[cursor] + 1) % 10;
			}
			else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
			{
				digits[cursor] = (digits[cursor] + 9) % 10; /* wrap down */
			}
			else if (this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (cursor < 6) cursor++;
			}
			else if (this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
			{
				if (cursor > 0) cursor--;
			}
			else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				/* Compute and validate frequency */
				uint32_t new_mhz = digits[0]*1000 + digits[1]*100 + digits[2]*10 + digits[3];
				uint32_t new_khz = digits[4]*100 + digits[5]*10 + digits[6];
				uint32_t new_freq = new_mhz * 1000000UL + new_khz * 1000UL;

				if (new_freq >= 142000000UL && new_freq <= 1050000000UL)
				{
					subghz_custom_freq_hz = new_freq;
					accepted = true;
					done = true;
				}
				else
				{
					/* Out of range — flash error */
					m1_message_box(&m1_u8g2, "Out of range!", "142.000 - 1050.000 MHz", "", "BACK to retry");
				}
			}
			else if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				done = true;
			}
		}
	}

	xQueueReset(main_q_hdl);
	return accepted;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void sub_ghz_record(void)
{
	bool sys_ready;
	uint8_t param = SUBGHZ_RECORD_DISPLAY_PARAM_READY;

	sys_ready = !sub_ghz_ring_buffers_init();
	if ( !sys_ready )
		param = SUBGHZ_RECORD_DISPLAY_PARAM_MEM_ERROR;
	datfile_info.dir_name = SUB_GHZ_FILEPATH;
	datfile_info.file_ext = SUB_GHZ_FILE_EXTENSION;
	datfile_info.file_prefix = SUB_GHZ_FILE_PREFIX;
	datfile_info.file_infix = NULL;
	datfile_info.file_suffix = NULL;

	menu_sub_ghz_init();
    xQueueReset(main_q_hdl); // Reset main q before start

	m1_gui_submenu_update(NULL, 0, 0, X_MENU_UPDATE_INIT);
	subghz_uiview_gui_latest_param = 0xFF; // Initialize with an invalid parameter

	// GUI init
	m1_uiView_functions_init(VIEW_MODE_SUBGHZ_RECORD_EOL, view_subghz_record_table);
	m1_uiView_display_switch(VIEW_MODE_SUBGHZ_RECORD, param);

	// Run
	while( m1_uiView_q_message_process() )
	{
		;
	}
} // void sub_ghz_record(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_record_gui_init(void)
{
	   m1_uiView_functions_register(VIEW_MODE_SUBGHZ_RECORD, subghz_record_gui_create, subghz_record_gui_update, subghz_record_gui_destroy, subghz_record_gui_message);
} // static void subghz_record_gui_init(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_record_gui_create(uint8_t param)
{
	m1_uiView_display_update(param);
} // static void subghz_record_gui_create(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_record_gui_destroy(uint8_t param)
{
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
} // static void subghz_record_gui_destroy(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_record_gui_update(uint8_t param)
{
	switch (param)
	{
		case SUBGHZ_RECORD_DISPLAY_PARAM_READY:
			/* Graphic work starts here */
		    u8g2_FirstPage(&m1_u8g2); // This call required for page drawing in mode 1
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 70, 0, 58, 10);
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
			u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
			u8g2_DrawXBMP(&m1_u8g2, 70, 1, 8, 8, arrowleft_8x8);
			u8g2_DrawXBMP(&m1_u8g2, 120, 1, 8, 8, arrowright_8x8);
			u8g2_DrawStr(&m1_u8g2, 82, 8, "Change");

			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawStr(&m1_u8g2, 96, 61, "Start");
			u8g2_DrawXBMP(&m1_u8g2, 84, 52, 10, 10, target_10x10); // draw TARGET icon

	//		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	//		u8g2_DrawStr(&m1_u8g2, 56, 30, "Receiving...");

			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			if (subghz_scan_config.band == SUB_GHZ_BAND_CUSTOM)
			{
				char cfreq_str[16];
				snprintf(cfreq_str, sizeof(cfreq_str), "%lu.%03lu",
				         subghz_custom_freq_hz / 1000000UL,
				         (subghz_custom_freq_hz % 1000000UL) / 1000UL);
				u8g2_DrawStr(&m1_u8g2, 55, 18, cfreq_str);
			}
			else
			{
				u8g2_DrawStr(&m1_u8g2, 60, 18, subghz_band_text[subghz_scan_config.band]);
			}
			u8g2_DrawStr(&m1_u8g2, 108, 18, subghz_modulation_text[subghz_scan_config.modulation]);

			u8g2_DrawXBMP(&m1_u8g2, 0, 5, 50, 27, subghz_antenna_50x27);
			break;

		case SUBGHZ_RECORD_DISPLAY_PARAM_ACTIVE:
			// Clear the CHANGE option at the top right
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
			u8g2_DrawBox(&m1_u8g2, 65, 0, 63, 10); // Clear existing content
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

			/* Show decoded protocol info if available */
			if (subghz_record_has_decoded)
			{
				char dec_str[40];
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
				u8g2_DrawBox(&m1_u8g2, 0, 18, 128, 32); // Clear middle area
				u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
				u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
				u8g2_DrawStr(&m1_u8g2, 2, 28, protocol_text[subghz_record_last_decoded.protocol]);
				snprintf(dec_str, sizeof(dec_str), "0x%lX %dbit",
				         (uint32_t)subghz_record_last_decoded.key,
				         subghz_record_last_decoded.bit_len);
				u8g2_DrawStr(&m1_u8g2, 2, 38, dec_str);
				snprintf(dec_str, sizeof(dec_str), "%ddBm TE:%d",
				         subghz_record_last_decoded.rssi,
				         subghz_record_last_decoded.te);
				u8g2_DrawStr(&m1_u8g2, 2, 48, dec_str);
			}

			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options

			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawXBMP(&m1_u8g2, 84, 52, 10, 10, target_10x10); // draw TARGET icon
			u8g2_DrawStr(&m1_u8g2, 96, 61, "Stop ");
			break;

		case SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE:
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawXBMP(&m1_u8g2, 84, 52, 10, 10, target_10x10); // draw TARGET icon
			u8g2_DrawStr(&m1_u8g2, 96, 61, "Play ");

			u8g2_DrawXBMP(&m1_u8g2, 1, 53, 8, 8, arrowleft_8x8); // draw arrowleft icon
			u8g2_DrawStr(&m1_u8g2, 11, 61, "Reset");

			u8g2_DrawXBMP(&m1_u8g2, 48, 53, 8, 8, arrowdown_8x8); // draw arrowdown icon
			u8g2_DrawStr(&m1_u8g2, 58, 61, "Save");
			break;

		case SUBGHZ_RECORD_DISPLAY_PARAM_PLAY:
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options

			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawXBMP(&m1_u8g2, 2, 52, 10, 10, target_10x10); // draw TARGET icon
			u8g2_DrawStr(&m1_u8g2, 14, 61, "Press OK to replay");
			break;

		case SUBGHZ_RECORD_DISPLAY_PARAM_SAVE:
			break;

		case SUBGHZ_RECORD_DISPLAY_PARAM_RESET:
			break;

		case SUBGHZ_RECORD_DISPLAY_PARAM_MEM_ERROR:
			/* Graphic work starts here */
		    u8g2_FirstPage(&m1_u8g2); // This call required for page drawing in mode 1
			// Display error message on screen
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawXBMP(&m1_u8g2, 2, 52, 10, 10, error_10x10); // draw ERROR icon
			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
			u8g2_DrawStr(&m1_u8g2, 14, 61, "Memory error! BACK to exit");
			break;

		case SUBGHZ_RECORD_DISPLAY_PARAM_SDCARD_ERROR:
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawStr(&m1_u8g2, 2, 61, "SD card access error!");
			param = subghz_uiview_gui_latest_param; // Do not update this parameter
			break;

		case SUBGHZ_RECORD_DISPLAY_PARAM_SYS_ERROR:
			// Display error message on screen
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawXBMP(&m1_u8g2, 2, 54, 10, 10, error_10x10); // draw ERROR icon
			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
			u8g2_DrawStr(&m1_u8g2, 14, 61, "Error! BACK to exit");
			break;

		default:
			break;
	} // switch (param)

	m1_u8g2_nextpage(); // Update display RAM

	subghz_uiview_gui_latest_param = param; // Update new param
} // static void subghz_record_gui_update(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static int subghz_record_gui_message(void)
{
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t ret_val = 1;
	uint32_t rcv_samples;

	ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
	if (ret==pdTRUE)
	{
		if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		{
			// Notification is only sent to this task when there's any button activity,
			// so it doesn't need to wait when reading the event from the queue
			ret_val = subghz_record_kp_handler();
		}
		else if ( q_item.q_evt_type==Q_EVENT_SUBGHZ_RX )
		{
			//arrpush(subghz_rx_q, q_item);
			//m1_buzzer_notification();
			rcv_samples = ringbuffer_get_data_slots(&subghz_rx_rawdata_rb);
			if ( rcv_samples >= SUBGHZ_RAW_DATA_SAMPLES_TO_RW )
			{
				M1_LOG_N(M1_LOGDB_TAG, "Raw samples %d\r\n", rcv_samples);
				sub_ghz_rx_raw_save(false, false);
				vTaskDelay(10); // Give the system some time in case RF noise is flooding the receiver
			} // if ( rcv_samples >= SUBGHZ_RAW_DATA_SAMPLES_TO_RW )

		} // if ( q_item.q_evt_type==Q_EVENT_SUBGHZ_RX )

		else if ( q_item.q_evt_type==Q_EVENT_SUBGHZ_TX )
		{
			subghz_replay_ret_code = sub_ghz_replay_continue(subghz_replay_ret_code);
			if ( subghz_replay_ret_code==SUB_GHZ_RAW_DATA_PARSER_IDLE )
			{
				m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
			} // if ( subghz_replay_ret_code==SUB_GHZ_RAW_DATA_PARSER_IDLE )
		} // else if ( q_item.q_evt_type==Q_EVENT_SUBGHZ_TX )
	} // if (ret==pdTRUE)

	return ret_val;
} // static int  subghz_record_gui_message(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static int subghz_record_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;
	char infix[5], *str;
	static uint8_t last_data_saved;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) 		// Exit or Stop
		{
			if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_ACTIVE )
			{
				// This case is handled the same way
				// with the case the [BUTTON_OK_KP_ID] is pressed
				// This code is copied from that case below. It can be placed in a sub-function if needed.
				sub_ghz_rx_pause(); // Stop receiving
				m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
				xQueueReset(main_q_hdl); // Reset old samples in the queue, if any

				if ( !last_data_saved )
				{
					sub_ghz_rx_raw_save(false, true);
					last_data_saved = true;
				} // if ( !last_data_saved )
				m1_sdm_task_stop(); // Stop sampling raw data and flush data to SD card
				m1_sdm_task_deinit();
				sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, subghz_scan_config.band, 0, 0);

				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE);
			} // if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_ACTIVE )
			else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE )
			{
				sub_ghz_raw_samples_deinit(true); // Discard samples
				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_READY);
			} // else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE )
			else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_PLAY )
			{
				sub_ghz_raw_tx_stop();
				sub_ghz_raw_samples_deinit(false);
				subghz_decenc_ctl.ntx_raw_repeat = 0;
				m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
				sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, subghz_scan_config.band, 0, 0);
				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE);
			} // else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_PLAY )
			else
			{
				m1_uiView_display_switch(VIEW_MODE_IDLE, 0);
				; // Do extra tasks here if needed
				sub_ghz_rx_deinit();
				sub_ghz_ring_buffers_deinit();
				sub_ghz_tx_raw_deinit();
				menu_sub_ghz_exit();
				xQueueReset(main_q_hdl); // Reset main q before return

				return 0;
				//break; // Exit and return to the calling task (subfunc_handler_task)
			} // else
		} // if ( m1_buttons_status[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )	// Start/Stop
		{
			if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_READY )
			{
				if (subghz_scan_config.band == SUB_GHZ_BAND_CUSTOM)
					strncpy(infix, "CUS", 4);
				else
					strncpy(infix, subghz_band_text[subghz_scan_config.band], 3);
				infix[3] = '\0';
				datfile_info.file_infix = infix;
				datfile_info.file_suffix = subghz_modulation_text[subghz_scan_config.modulation];
				ret = m1_sdm_file_init(&datfile_info);
				if ( !ret )
				{
					last_data_saved = false;
					subghz_record_has_decoded = false;
					m1_sdm_task_init();
					m1_sdm_task_start();
					sub_ghz_rx_raw_save(true, false);
					xQueueReset(main_q_hdl); // Reset old samples in the queue, if any
					m1_ringbuffer_reset(&subghz_rx_rawdata_rb); // Reset sample ring buffer
					m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M); // Turn on
					sub_ghz_tx_raw_deinit();
					subghz_decenc_ctl.pulse_det_stat = PULSE_DET_ACTIVE;
					sub_ghz_set_opmode(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
					// Adjust the attack and decay times
					SI446x_Change_Modem_OOK_PDTC(SUB_GHZ_433_92_NEW_PDTC);
					sub_ghz_rx_init();
					sub_ghz_rx_start();
					subghz_record_mode_flag = true;
					m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_ACTIVE);
				} // if ( !ret )
				else
				{
					m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_SDCARD_ERROR);
				} // else
			} // if ( nfc_uiview_gui_latest_param==NFC_READ_DISPLAY_PARAM_READING_COMPLETE )
			else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_ACTIVE )
			{
				sub_ghz_rx_pause(); // Stop receiving
				m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
				xQueueReset(main_q_hdl); // Reset old samples in the queue, if any

				if ( !last_data_saved )
				{
					sub_ghz_rx_raw_save(false, true);
					last_data_saved = true;
				} // if ( !last_data_saved )
				m1_sdm_task_stop(); // Stop sampling raw data and flush data to SD card
				m1_sdm_task_deinit();
				sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, subghz_scan_config.band, 0, 0);

				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE);
			} // else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_ACTIVE )
			else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE )
			{
				subghz_replay_ret_code = sub_ghz_replay_start(true, subghz_scan_config.band, 0, 255);
				if ( subghz_replay_ret_code )
				{
					double_buffer_ptr_id = 1; // Update raw samples buffer
					m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_PLAY);
					m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M); // Turn on
				} // if ( subghz_replay_ret_code )
				else
				{
					m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_SYS_ERROR);
				} // else
			} // else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE )
			else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_PLAY )
			{
				if ( subghz_replay_ret_code != SUB_GHZ_RAW_DATA_PARSER_IDLE ) // Do nothing if it's replaying
					return 1;
				sub_ghz_set_opmode(SUB_GHZ_OPMODE_TX, subghz_scan_config.band, 0, tx_power_values[subghz_tx_power_idx]);
				subghz_replay_ret_code = sub_ghz_raw_replay_init();
				if ( subghz_replay_ret_code!=1 )
				{
					double_buffer_ptr_id = 1;
					subghz_decenc_ctl.ntx_raw_repeat = SUBGHZ_TX_RAW_REPLAY_REPEAT_DEFAULT;
					m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M); // Turn on
				} // if ( subghz_replay_ret_code!=1 )
				else
				{
					sub_ghz_raw_tx_stop();
					sub_ghz_raw_samples_deinit(false);
					sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, SUB_GHZ_BAND_EOL, 0, 0);
					// Display error message at the bottom line if the function x_reinit() failed.
					m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_SYS_ERROR);
				} // else
			} // else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_PLAY )
		} // else if(this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )	// Left
		{
			if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_READY )
			{
				uint8_t idx = subghz_band_order_find(subghz_scan_config.band);
				if (idx > 0) idx--; else idx = SUBGHZ_BAND_ORDER_COUNT - 1;
				subghz_scan_config.band = subghz_band_order[idx];
				subghz_scan_config.modulation = (subghz_scan_config.band == SUB_GHZ_BAND_915) ? MODULATION_FSK : MODULATION_OOK;
				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_READY);
			} // if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_READY )
			else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE )
			{
				sub_ghz_raw_samples_deinit(true); // Discard samples
				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_READY);
			} // else if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE )
		} // else if(this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK )	// Right
		{
			if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_READY )
			{
				uint8_t idx = subghz_band_order_find(subghz_scan_config.band);
				idx++;
				if (idx >= SUBGHZ_BAND_ORDER_COUNT) idx = 0;
				subghz_scan_config.band = subghz_band_order[idx];
				subghz_scan_config.modulation = (subghz_scan_config.band == SUB_GHZ_BAND_915) ? MODULATION_FSK : MODULATION_OOK;
				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_READY);
			} // if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_READY )
		} // else if(this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )	// Up = Custom Freq
		{
			if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_READY )
			{
				if (sub_ghz_custom_freq_entry())
				{
					subghz_scan_config.band = SUB_GHZ_BAND_CUSTOM;
					subghz_scan_config.modulation = (subghz_custom_freq_hz >= 850000000UL) ? MODULATION_FSK : MODULATION_OOK;
				}
				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_READY);
			}
		} // else if(this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )	// Down
		{
			if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE )
			{
				// Save .sgh raw file
				sub_ghz_raw_samples_deinit(false);
				str = strstr(&datfile_info.dat_filename[1], "/");
				if ( str!=NULL )
					str += 1;
				else
					str = datfile_info.dat_filename;

				/* Also save a Flipper-compatible .sub file if protocol was decoded */
				if (subghz_record_has_decoded && subghz_record_last_decoded.key != 0)
				{
					flipper_subghz_signal_t sub_sig;
					char sub_path[64];
					uint32_t freq_hz;

					memset(&sub_sig, 0, sizeof(sub_sig));
					sub_sig.type = FLIPPER_SUBGHZ_TYPE_PARSED;

					if (subghz_scan_config.band == SUB_GHZ_BAND_CUSTOM)
						freq_hz = subghz_custom_freq_hz;
					else if (subghz_scan_config.band < SUB_GHZ_BAND_EOL)
						freq_hz = (uint32_t)(subghz_band_steps[subghz_scan_config.band][0] * 1000000.0f);
					else
						freq_hz = 433920000UL;

					sub_sig.frequency = freq_hz;
					strncpy(sub_sig.preset, "FuriHalSubGhzPresetOok650Async", FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1);
					strncpy(sub_sig.protocol, protocol_text[subghz_record_last_decoded.protocol], FLIPPER_SUBGHZ_PROTO_MAX_LEN - 1);
					sub_sig.bit_count = subghz_record_last_decoded.bit_len;
					sub_sig.key = subghz_record_last_decoded.key;
					sub_sig.te = subghz_record_last_decoded.te;

					uint32_t next_num = m1_sdm_getlastfilenumber("/SUBGHZ", "sig_") + 1;
					snprintf(sub_path, sizeof(sub_path), "/SUBGHZ/sig_%04lu.sub", next_num);

					if (flipper_subghz_save(sub_path, &sub_sig))
						m1_message_box(&m1_u8g2, "Saved .sgh + .sub:", str, sub_path + 8, "BACK to exit");
					else
						m1_message_box(&m1_u8g2, "Saved .sgh:", str, ".sub save failed", "BACK to exit");
				}
				else
				{
					m1_message_box(&m1_u8g2, "Saved to:", str, "", "BACK to exit");
				}

				subghz_record_has_decoded = false;
				m1_uiView_display_update(SUBGHZ_RECORD_DISPLAY_PARAM_READY);
			} // if ( subghz_uiview_gui_latest_param==SUBGHZ_RECORD_DISPLAY_PARAM_COMPLETE )
		} // else if(this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )
	} // if (ret==pdTRUE)

	return 1;
} // static int subghz_record_kp_handler(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_replay_browse_gui_init(void)
{
	m1_uiView_functions_register(VIEW_MODE_SUBGHZ_REPLAY_BROWSE, subghz_replay_browse_gui_create, subghz_replay_browse_gui_update, subghz_replay_browse_gui_destroy, subghz_replay_browse_gui_message);
} // static void subghz_replay_browse_gui_init(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_replay_browse_gui_create(uint8_t param)
{
	m1_uiView_display_update(param);
} // static void subghz_replay_browse_gui_create(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_replay_browse_gui_destroy(uint8_t param)
{

} // static void subghz_replay_browse_gui_destroy(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_replay_browse_gui_update(uint8_t param)
{
	while (true)
	{
		f_info = storage_browse();
		if ( !f_info->file_is_selected ) // User exits?
		{
			m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);
			break;
		} // if ( f_info->file_is_selected )

		if ( sub_ghz_file_load() ) // Error?
		{
			m1_message_box(&m1_u8g2, "File error!", "", "", "BACK to return");
			continue;
		}

		m1_uiView_display_switch(VIEW_MODE_SUBGHZ_REPLAY_PLAY, SUBGHZ_REPLAY_DISPLAY_PARAM_ACTIVE);
		menu_sub_ghz_init();
		subghz_replay_ret_code = sub_ghz_replay_start(false, subghz_replay_band, subghz_replay_channel, 255);
		if ( subghz_replay_ret_code )
		{
			double_buffer_ptr_id = 1; // Update raw samples buffer
			m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M); // Turn on
			m1_uiView_display_update(SUBGHZ_REPLAY_DISPLAY_PARAM_PLAY);
		} // if ( ret_code )
		else
		{
			m1_uiView_display_update(SUBGHZ_REPLAY_DISPLAY_PARAM_SYS_ERROR);
		} // else
		break;
	} // while (true)

} // static void subghz_replay_browse_gui_update(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static int subghz_replay_browse_gui_message(void)
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
			ret_val = subghz_replay_browse_kp_handler();
		} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
		else if(q_item.q_evt_type==Q_EVENT_MENU_EXIT)
		{
			m1_uiView_display_switch(VIEW_MODE_IDLE, 0);
			xQueueReset(main_q_hdl); // Reset main q before return
			menu_sub_ghz_exit();
			ret_val = 0;
		}
	} // if (ret==pdTRUE)

	return ret_val;
} // static int  subghz_replay_browse_gui_message(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static int subghz_replay_browse_kp_handler(void)
{
	return 1;
} // static int subghz_replay_browse_kp_handler(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_replay_play_gui_init(void)
{
	m1_uiView_functions_register(VIEW_MODE_SUBGHZ_REPLAY_PLAY, subghz_replay_play_gui_create, subghz_replay_play_gui_update, subghz_replay_play_gui_destroy, subghz_replay_play_gui_message);
} // static void subghz_replay_play_gui_init(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_replay_play_gui_create(uint8_t param)
{
	m1_uiView_display_update(param);
} // static void subghz_replay_play_gui_create(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_replay_play_gui_destroy(uint8_t param)
{

} // static void subghz_replay_play_gui_destroy(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void subghz_replay_play_gui_update(uint8_t param)
{
	uint8_t freq_text[20];

	switch (param)
	{
		case SUBGHZ_REPLAY_DISPLAY_PARAM_ACTIVE:
			// This call required for page drawing in mode 1
			m1_u8g2_firstpage();
			u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			m1_float_to_string(freq_text, subghz_replay_freq, 3);
			strcat(freq_text, "MHz ");
			strcat(freq_text, subghz_modulation_text[subghz_replay_mod]);
			u8g2_DrawStr(&m1_u8g2, 40, 10, freq_text);

			u8g2_DrawXBMP(&m1_u8g2, 0, 5, 50, 27, subghz_antenna_50x27);
			break;

		case SUBGHZ_REPLAY_DISPLAY_PARAM_PLAY:
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawXBMP(&m1_u8g2, 2, 52, 10, 10, target_10x10); // draw TARGET icon
			u8g2_DrawStr(&m1_u8g2, 14, 61, "Press OK to replay");
			break;

		case SUBGHZ_REPLAY_DISPLAY_PARAM_SYS_ERROR:
			// Display error message on screen
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12); // Draw an inverted bar at the bottom to display options
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // Write text in inverted color
			u8g2_DrawXBMP(&m1_u8g2, 2, 52, 10, 10, error_10x10); // draw ERROR icon
			u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
			u8g2_DrawStr(&m1_u8g2, 14, 61, "File error! BACK to return");
			break;

		default:
			break;
	} // switch (param)

	m1_u8g2_nextpage(); // Update display RAM
    subghz_uiview_gui_latest_param = param; // Update new param
} // static void subghz_replay_play_gui_update(uint8_t param)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static int subghz_replay_play_gui_message(void)
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
			ret_val = subghz_replay_play_kp_handler();
		}
		else if ( q_item.q_evt_type==Q_EVENT_SUBGHZ_TX )
		{
			subghz_replay_ret_code = sub_ghz_replay_continue(subghz_replay_ret_code);
			if ( subghz_replay_ret_code==SUB_GHZ_RAW_DATA_PARSER_IDLE )
			{
				m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
			} // if ( subghz_replay_ret_code==SUB_GHZ_RAW_DATA_PARSER_IDLE )
		} // else if ( q_item.q_evt_type==Q_EVENT_SUBGHZ_TX )
	} // if (ret==pdTRUE)

	return ret_val;
} // static int  subghz_replay_play_gui_message(void)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static int subghz_replay_play_kp_handler(void)
{
	S_M1_Buttons_Status this_button_status;
	BaseType_t ret;

	ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
	if (ret==pdTRUE)
	{
		if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) 		// Exit or Stop
		{
			m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF, LED_FASTBLINK_ONTIME_OFF); // Turn off
			; // Do extra tasks here if needed
			sub_ghz_raw_samples_deinit(false);
			sub_ghz_ring_buffers_deinit();
			sub_ghz_tx_raw_deinit();
			//menu_sub_ghz_exit();
			m1_uiView_display_switch(VIEW_MODE_SUBGHZ_REPLAY_BROWSE, 0);
		} // if ( m1_buttons_status[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
		else if(this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )	// Start/Stop
		{
			if ( subghz_uiview_gui_latest_param==SUBGHZ_REPLAY_DISPLAY_PARAM_PLAY )
			{
				if ( subghz_replay_ret_code != SUB_GHZ_RAW_DATA_PARSER_IDLE ) // Do nothing if it's replaying
					return 1;
				sub_ghz_set_opmode(SUB_GHZ_OPMODE_TX, subghz_replay_band, subghz_replay_channel, tx_power_values[subghz_tx_power_idx]);
				subghz_replay_ret_code = sub_ghz_raw_replay_init();
				if ( subghz_replay_ret_code!=1 )
				{
					double_buffer_ptr_id = 1;
					subghz_decenc_ctl.ntx_raw_repeat = SUBGHZ_TX_RAW_REPLAY_REPEAT_DEFAULT;
					m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M, LED_FASTBLINK_ONTIME_M); // Turn on
				} // if ( ret_code!=1 )
				else
				{
					sub_ghz_raw_tx_stop();
					sub_ghz_raw_samples_deinit(false);
					sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, SUB_GHZ_BAND_EOL, 0, 0);
					// Display error message at the bottom line if the function x_reinit() failed.
					m1_uiView_display_update(SUBGHZ_REPLAY_DISPLAY_PARAM_SYS_ERROR);
				} // else
			} // if ( subghz_uiview_gui_latest_param==SUBGHZ_REPLAY_DISPLAY_PARAM_PLAY )
		} // else if(this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK )
	} // if (ret==pdTRUE)

	return 1;
} // static int subghz_replay_play_kp_handler(void)



/*============================================================================*/
/**
  * @brief  Load Sub-GHz data file from SD card
  * @param
  * @retval 0 if success
  */
/*============================================================================*/
static uint8_t sub_ghz_file_load(void)
{
	uint8_t uret, sys_error, key_len;
	char *token, *str, *end_ptr;
	float freq_min, freq_max;

	sys_error = 1;
	do
	{
		uret = strlen(f_info->file_name);
		if ( !uret )
			break;
		key_len = 0;
		while ( uret )
		{
			key_len++;
			if ( f_info->file_name[--uret]=='.' ) // Find the dot starting from the last character
				break;
		} // while ( uret )
		if ( !uret || (key_len!=4) ) // key_len==length of SUB_GHZ_FILE_EXTENSION
			break;
		if ( strcmp(&f_info->file_name[uret], SUB_GHZ_FILE_EXTENSION) )
			break;
		sprintf(datfile_info.dat_filename, "%s/%s", f_info->dir_name, f_info->file_name);

		sys_error = sub_ghz_ring_buffers_init();
		if ( sys_error )
			break;
		sys_error = sub_ghz_raw_samples_init();
		if ( sys_error )
			break;

		token = strtok(sdcard_dat_buffer, "\r\n"); // Filetype
		key_len = SUB_GHZ_DATAFILE_RAW_FORMAT_N;
		if ( strstr(token, SUB_GHZ_DATAFILE_FILETYPE_PACKET) )
		{
			key_len = SUB_GHZ_DATAFILE_RAW_FORMAT_N;
			break; // Not support for now.
		} // if ( strstr(token, SUB_GHZ_DATAFILE_FILETYPE_PACKET) )
		token = strtok(NULL, "\r\n"); // Version
		token = strtok(NULL, "\r\n"); // Frequency
		str = strstr(token, ":");
		str += 1; // Move to the frequency value
		subghz_replay_freq = strtol(str, &end_ptr, 10);
		if ( subghz_replay_freq==0 )
			break;
		subghz_replay_freq /= 1000000; // Convert frequency from Hz to MHz
		for (subghz_replay_band=0; subghz_replay_band<SUB_GHZ_BAND_EOL; subghz_replay_band++)
		{
			freq_min = subghz_band_steps[subghz_replay_band][0];
			freq_max = freq_min;
			freq_max += 0.25*subghz_band_steps[subghz_replay_band][1]; // Channel spacing = 0.25MHz
			if ( (subghz_replay_freq >= freq_min) && (subghz_replay_freq <= freq_max) )
				break;
		} // for (subghz_replay_band=0; subghz_replay_band<SUB_GHZ_BAND_EOL; subghz_replay_band++)
		if ( subghz_replay_band >= SUB_GHZ_BAND_EOL ) // Not found?
			break;
		subghz_replay_channel = 0;
		while ( subghz_replay_freq > freq_min )
		{
			freq_min += 0.25; // Increase channel until the selected frequency matches closely with the desired frequency
			subghz_replay_channel++;
		} // while ( subghz_replay_freq > freq_min )

		token = strtok(NULL, "\r\n"); // Modulation
		m1_strtoupper(token);
		for (subghz_replay_mod=0; subghz_replay_mod<SUBGHZ_MODULATION_LIST; subghz_replay_mod++)
		{
			if ( strstr(token, subghz_modulation_text[subghz_replay_mod]) )
				break;
		}
		if ( subghz_replay_mod >= SUBGHZ_MODULATION_LIST ) // Not found?
			break;
		sys_error = 0; // Reset, no error
	} while (0);

	return sys_error;
} // static uint8_t sub_ghz_file_load(void)


/*============================================================================*/
/**
  * @brief  Load data file from SD card and replay it
  * @param  None
  * @retval None
  */
/*============================================================================*/
void sub_ghz_replay(void)
{
	m1_gui_submenu_update(NULL, 0, 0, X_MENU_UPDATE_INIT);
	subghz_uiview_gui_latest_param = 0xFF; // Initialize with an invalid parameter

	// GUI init
	m1_uiView_functions_init(VIEW_MODE_SUBGHZ_REPLAY_EOL, view_subghz_replay_table);
	m1_uiView_display_switch(VIEW_MODE_SUBGHZ_REPLAY_BROWSE, 0);

	// Run
	while( m1_uiView_q_message_process() )
	{
		;
	}
} // void sub_ghz_replay(void)


/*============================================================================*/
/**
  * @brief  Convert a Flipper .sub file to M1's .sgh format and replay it.
  *         Handles RAW type .sub files.  Streams via a temp file on SD card
  *         so there is no sample-count limit.
  * @param  sub_path  Path to the .sub file on the SD card
  * @retval 0 = success, non-zero = error
  */
/*============================================================================*/
uint8_t sub_ghz_replay_flipper_file(const char *sub_path)
{
#define FLIPPER_SUB_TMP_SGH   "/SUBGHZ/_flipper_tmp.sgh"
#define FLIPPER_SUB_LINE_MAX  512

	FIL f_sub, f_sgh;
	FRESULT fr;
	char *line_buf;
	char *out_buf;
	uint32_t frequency = 0;
	uint8_t modulation = MODULATION_OOK;
	bool is_raw = false;
	float freq_mhz, freq_min;

	line_buf = malloc(FLIPPER_SUB_LINE_MAX);
	if (!line_buf) return 1;
	out_buf = malloc(FLIPPER_SUB_LINE_MAX);
	if (!out_buf) { free(line_buf); return 1; }

	/* ── 1. Open .sub source ── */
	fr = f_open(&f_sub, sub_path, FA_READ);
	if (fr != FR_OK)
	{
		free(line_buf); free(out_buf);
		return 1;
	}

	/* ── 2. Create temp .sgh file ── */
	f_mkdir("/SUBGHZ");
	fr = f_open(&f_sgh, FLIPPER_SUB_TMP_SGH, FA_WRITE | FA_CREATE_ALWAYS);
	if (fr != FR_OK)
	{
		f_close(&f_sub);
		free(line_buf); free(out_buf);
		return 1;
	}

	/* ── 3. Convert header + data ── */
	while (f_gets(line_buf, FLIPPER_SUB_LINE_MAX, &f_sub))
	{
		/* Strip trailing CR/LF */
		size_t len = strlen(line_buf);
		while (len > 0 && (line_buf[len - 1] == '\r' || line_buf[len - 1] == '\n'))
			line_buf[--len] = '\0';

		if (strncmp(line_buf, "Filetype:", 9) == 0)
		{
			if (strstr(line_buf, "RAW"))
				is_raw = true;
			f_puts("Filetype: M1 SubGHz NOISE\r\n", &f_sgh);
		}
		else if (strncmp(line_buf, "Version:", 8) == 0)
		{
			f_puts("Version: 0.8\r\n", &f_sgh);
		}
		else if (strncmp(line_buf, "Frequency:", 10) == 0)
		{
			frequency = (uint32_t)strtoul(line_buf + 10, NULL, 10);
			snprintf(out_buf, FLIPPER_SUB_LINE_MAX, "Frequency: %lu\r\n",
			         (unsigned long)frequency);
			f_puts(out_buf, &f_sgh);
		}
		else if (strncmp(line_buf, "Preset:", 7) == 0)
		{
			if (strstr(line_buf, "Ook") || strstr(line_buf, "OOK"))
				modulation = MODULATION_OOK;
			else if (strstr(line_buf, "2FSK") || strstr(line_buf, "FSK"))
				modulation = MODULATION_FSK;
			snprintf(out_buf, FLIPPER_SUB_LINE_MAX, "Modulation: %s\r\n",
			         subghz_modulation_text[modulation]);
			f_puts(out_buf, &f_sgh);
		}
		else if (strncmp(line_buf, "RAW_Data:", 9) == 0)
		{
			/* Parse signed values, write absolute values as Data: line */
			const char *p = line_buf + 9;
			int pos = snprintf(out_buf, FLIPPER_SUB_LINE_MAX, "%s",
			                   SUB_GHZ_DATAFILE_DATA_KEYWORD);
			while (*p)
			{
				while (*p == ' ') p++;
				if (*p == '\0') break;
				char *endp;
				long val = strtol(p, &endp, 10);
				if (endp == p) break;   /* no more numbers */
				p = endp;
				if (val < 0) val = -val;
				pos += snprintf(&out_buf[pos],
				                (size_t)(FLIPPER_SUB_LINE_MAX - pos),
				                " %lu", (unsigned long)val);
				if (pos >= FLIPPER_SUB_LINE_MAX - 16) break;
			}
			strcat(out_buf, "\r\n");
			f_puts(out_buf, &f_sgh);
		}
		/* Protocol: line from .sub is ignored — .sgh doesn't use it */
	}

	f_close(&f_sub);
	f_close(&f_sgh);

	free(line_buf);
	free(out_buf);

	if (!is_raw || frequency == 0)
	{
		f_unlink(FLIPPER_SUB_TMP_SGH);
		return 2; /* unsupported format or missing data */
	}

	/* ── 4. Map frequency to band/channel ── */
	freq_mhz = (float)frequency / 1000000.0f;
	subghz_replay_freq = freq_mhz;
	subghz_replay_mod  = modulation;

	for (subghz_replay_band = 0;
	     subghz_replay_band < SUB_GHZ_BAND_EOL;
	     subghz_replay_band++)
	{
		freq_min = subghz_band_steps[subghz_replay_band][0];
		float freq_max = freq_min +
		                 0.25f * subghz_band_steps[subghz_replay_band][1];
		if (freq_mhz >= freq_min && freq_mhz <= freq_max)
			break;
	}
	if (subghz_replay_band >= SUB_GHZ_BAND_EOL)
	{
		f_unlink(FLIPPER_SUB_TMP_SGH);
		return 3; /* unsupported frequency */
	}

	subghz_replay_channel = 0;
	freq_min = subghz_band_steps[subghz_replay_band][0];
	while (freq_mhz > freq_min)
	{
		freq_min += 0.25f;
		subghz_replay_channel++;
	}

	/* ── 5. Set up datfile_info → temp .sgh ── */
	strncpy((char *)datfile_info.dat_filename, FLIPPER_SUB_TMP_SGH,
	        sizeof(datfile_info.dat_filename) - 1);
	datfile_info.dat_filename[sizeof(datfile_info.dat_filename) - 1] = '\0';

	/* ── 6. Init buffers and open the file for streaming ── */
	if (sub_ghz_ring_buffers_init())
	{
		f_unlink(FLIPPER_SUB_TMP_SGH);
		return 4;
	}
	if (sub_ghz_raw_samples_init())
	{
		sub_ghz_ring_buffers_deinit();
		f_unlink(FLIPPER_SUB_TMP_SGH);
		return 5;
	}

	/* ── 7. Draw replay screen and start first TX ── */
	menu_sub_ghz_init();
	subghz_replay_play_gui_update(SUBGHZ_REPLAY_DISPLAY_PARAM_ACTIVE);

	subghz_replay_ret_code = sub_ghz_replay_start(false, subghz_replay_band,
	                                              subghz_replay_channel, 255);
	if (subghz_replay_ret_code)
	{
		double_buffer_ptr_id = 1;
		m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M,
		                  LED_FASTBLINK_ONTIME_M);
		subghz_replay_play_gui_update(SUBGHZ_REPLAY_DISPLAY_PARAM_PLAY);
	}
	else
	{
		subghz_replay_play_gui_update(SUBGHZ_REPLAY_DISPLAY_PARAM_SYS_ERROR);
	}

	/* ── 8. Self-contained event loop (blocks until BACK) ── */
	{
		S_M1_Main_Q_t q_item;
		S_M1_Buttons_Status btn;
		BaseType_t qret;
		bool running = true;

		while (running)
		{
			qret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
			if (qret != pdTRUE)
				continue;

			if (q_item.q_evt_type == Q_EVENT_KEYPAD)
			{
				qret = xQueueReceive(button_events_q_hdl, &btn, 0);
				if (qret != pdTRUE)
					continue;

				if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
				{
					/* Stop TX, cleanup, exit */
					m1_led_fast_blink(LED_BLINK_ON_RGB,
					                  LED_FASTBLINK_PWM_OFF,
					                  LED_FASTBLINK_ONTIME_OFF);
					sub_ghz_raw_samples_deinit(false);
					sub_ghz_ring_buffers_deinit();
					sub_ghz_tx_raw_deinit();
					running = false;
				}
				else if (btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
				{
					/* Replay again */
					if (subghz_replay_ret_code == SUB_GHZ_RAW_DATA_PARSER_IDLE)
					{
						sub_ghz_set_opmode(SUB_GHZ_OPMODE_TX,
						                   subghz_replay_band,
						                   subghz_replay_channel,
						                   tx_power_values[subghz_tx_power_idx]);
						subghz_replay_ret_code = sub_ghz_raw_replay_init();
						if (subghz_replay_ret_code != 1)
						{
							double_buffer_ptr_id = 1;
							subghz_decenc_ctl.ntx_raw_repeat =
							    SUBGHZ_TX_RAW_REPLAY_REPEAT_DEFAULT;
							m1_led_fast_blink(LED_BLINK_ON_RGB,
							                  LED_FASTBLINK_PWM_M,
							                  LED_FASTBLINK_ONTIME_M);
						}
						else
						{
							sub_ghz_raw_tx_stop();
							sub_ghz_raw_samples_deinit(false);
							sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED,
							                   SUB_GHZ_BAND_EOL, 0, 0);
							subghz_replay_play_gui_update(
							    SUBGHZ_REPLAY_DISPLAY_PARAM_SYS_ERROR);
						}
					}
				}
			}
			else if (q_item.q_evt_type == Q_EVENT_SUBGHZ_TX)
			{
				/* Continue double-buffered TX streaming */
				subghz_replay_ret_code =
				    sub_ghz_replay_continue(subghz_replay_ret_code);
				if (subghz_replay_ret_code == SUB_GHZ_RAW_DATA_PARSER_IDLE)
				{
					m1_led_fast_blink(LED_BLINK_ON_RGB,
					                  LED_FASTBLINK_PWM_OFF,
					                  LED_FASTBLINK_ONTIME_OFF);
				}
			}
		} /* while (running) */
	}

	xQueueReset(main_q_hdl);
	menu_sub_ghz_exit();

	/* ── 9. Cleanup temp file ── */
	f_unlink(FLIPPER_SUB_TMP_SGH);
	return 0;

#undef FLIPPER_SUB_TMP_SGH
#undef FLIPPER_SUB_LINE_MAX
} // uint8_t sub_ghz_replay_flipper_file(const char *sub_path)


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void sub_ghz_frequency_reader(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	int16_t rssi, rssi_max, rssi_avg, freq_step;
	int16_t avg_noisefloor[SUB_GHZ_BAND_EOL][3];
	uint8_t active_band_id, i, j, asf_sample_count, detection_count;
	float freq_found;
	uint8_t prn_buffer[30], float_buffer[10];
	struct si446x_reply_GET_MODEM_STATUS_map *pmodemstat;

	m1_u8g2_firstpage();
	 // This call required for page drawing in mode 1
    do
    {
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 1, 10, "Frequency Reader");
		u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
		u8g2_DrawStr(&m1_u8g2, 10, 30, "000.000");
		u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_1B);
		u8g2_DrawStr(&m1_u8g2, 100, 28, "MHz");
		u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
		u8g2_DrawStr(&m1_u8g2, 1, INFO_BOX_Y_POS_ROW_1, "000.000MHz");
		u8g2_DrawStr(&m1_u8g2, 1, INFO_BOX_Y_POS_ROW_2, "000.000MHz");
		u8g2_DrawStr(&m1_u8g2, 1, INFO_BOX_Y_POS_ROW_3, "000.000MHz");
    } while (m1_u8g2_nextpage());

    for (i=0; i<SUB_GHZ_BAND_EOL; i++)
    {
    	for (j=0; j<3; j++)
    		avg_noisefloor[i][j] = NOISE_FLOOR_RSSI_THRESHOLD;
    }

    active_band_id = SUB_GHZ_BAND_300;
    freq_step = CHANNEL_STEPS_MAX + 1;
    detection_count = 0;
    asf_sample_count = 0;

    menu_sub_ghz_init();

    while (1 ) // Main loop of this task
	{
		;
		; // Do other parts of this task here
		;
	    if ( freq_step <= subghz_band_steps[active_band_id][1] )
	    {
	    	SI446x_Start_Rx(freq_step); // Change channel
	    } // if ( freq_step <= subghz_band_steps[active_band_id] )
	    else
	    {
	    	sub_ghz_set_opmode(SUB_GHZ_OPMODE_RX, active_band_id, 0, 0); // Process time: ~27.57ms (with reset) - ~4.5ms (optimized)
	    	if ( subghz_band_steps[active_band_id][1] > 1 ) // There're other channels to retry
	    		freq_step = 0; // Let start with channel 1 in next round
	    } // else
		// Read INTs, clear pending ones
		SI446x_Get_IntStatus(0, 0, 0);
		rssi_max = -255;
		for ( i=0; i<2; i++)
		{
			//vTaskDelay(3); // Give the radio chip some time to do its task
			pmodemstat = SI446x_Get_ModemStatus(0x00); // Process time: ~99.7us
			// RF_Input_Level_dBm = (RSSI_value / 2) – MODEM_RSSI_COMP – 70
			// MODEM_RSSI_COMP = 0x40 = 64d is appropriate for most applications.
			rssi = pmodemstat->CURR_RSSI/2 - MODEM_RSSI_COMP - 70;
			if ( rssi > rssi_max )
				rssi_max = rssi;
			vTaskDelay(1);
		} // for ( i=0; i<2; i++)

		rssi_avg = 0;
		for (i=0; i<3; i++)
			rssi_avg += avg_noisefloor[active_band_id][i];
		rssi_avg /= 3; // Get average noise floor of the current frequency
		if ( rssi_max >= (rssi_avg + SIGNAL_TO_NOISE_RATIO) ) // SNR matches the condition?
		{
			m1_buzzer_notification();
			freq_found = subghz_band_steps[active_band_id][0];
			if ( freq_step <= CHANNEL_STEPS_MAX )
				freq_found += freq_step*CHANNEL_STEP;
			m1_float_to_string(float_buffer, freq_found, 3);

			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
			u8g2_DrawBox(&m1_u8g2, 10, 14, 90, 17); // Clear old content
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
			u8g2_DrawStr(&m1_u8g2, 10, 30, float_buffer);
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
			u8g2_DrawBox(&m1_u8g2, 1, INFO_BOX_Y_POS_ROW_1 + detection_count*10 - 9, M1_LCD_DISPLAY_WIDTH, 10); // Clear old content
			u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
			u8g2_SetFont(&m1_u8g2, M1_DISP_MAIN_MENU_FONT_N);
			sprintf(prn_buffer, "%sMHz RSSI%ddBm", float_buffer, rssi);
			u8g2_DrawStr(&m1_u8g2, 1, INFO_BOX_Y_POS_ROW_1 + detection_count*10, prn_buffer);
			m1_u8g2_nextpage(); // Update display RAM

			M1_LOG_N(M1_LOGDB_TAG, float_buffer);
			M1_LOG_N(M1_LOGDB_TAG, " RSSI: %ddBm\r\n", rssi);
			detection_count++;
			if ( detection_count >= 3 )
				detection_count = 0;
		} // if ( rssi_max >= (rssi_avg + SIGNAL_TO_NOISE_RATIO) )
		else
		{
			avg_noisefloor[active_band_id][asf_sample_count] = rssi_max;
		}

		if ( ++freq_step > subghz_band_steps[active_band_id][1] ) // All channels have been completed?
		{
			freq_step = CHANNEL_STEPS_MAX + 1; // Reset
			while (true)
			{
				active_band_id++;
				if ( active_band_id >= SUB_GHZ_BAND_EOL)
				{
					active_band_id = SUB_GHZ_BAND_300;
					asf_sample_count++;
					if ( asf_sample_count >= 3 )
						asf_sample_count = 0;
					vTaskDelay(20); // Return time to system to do its job
				} // if ( active_band_id >= SUB_GHZ_BAND_EOL)
				if ( subghz_band_steps[active_band_id][1] != 0 ) // Change to this band if it is not disabled
					break;
			} // while (true)
		} // if ( ++freq_step > subghz_band_steps[active_band_id][1] )
		// Wait for the notification from button_event_handler_task to subfunc_handler_task.
		// This task is the sub-task of subfunc_handler_task.
		// The notification is given in the form of an item in the main queue.
		// So let read the main queue.
		ret = xQueueReceive(main_q_hdl, &q_item, 0/*portMAX_DELAY*/);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				// Notification is only sent to this task when there's any button activity,
				// so it doesn't need to wait when reading the event from the queue
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // user wants to exit?
				{
					; // Do extra tasks here if needed
					sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, SUB_GHZ_BAND_300, 0, 0);
					menu_sub_ghz_exit();

					xQueueReset(main_q_hdl); // Reset main q before return
					break; // Exit and return to the calling task (subfunc_handler_task)
				} // if ( m1_buttons_status[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				else
				{
					; // Do other things for this task, if needed
				}
			} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			else
			{
				; // Do other things for this task
			}
		} // if (ret==pdTRUE)
	} // while (1 ) // Main loop of this task

} // void sub_ghz_frequency_reader(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void sub_ghz_regional_information(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t i, row, freq_l[10], freq_h[10], freq_range[25];

	/* Graphic work starts here */
	u8g2_FirstPage(&m1_u8g2);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B); // Set bold font
	sprintf(freq_range, "Region: %s", subghz_ism_regions_text[m1_device_stat.config.ism_band_region]);
	u8g2_DrawStr(&m1_u8g2, 0, 10, freq_range);
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N); // Set normal font
	u8g2_DrawStr(&m1_u8g2, 0, 20, "Bands:");
	row = 30;
	for (i=0; i<subghz_regions_list[m1_device_stat.config.ism_band_region].bands_list; i++)
	{
		m1_float_to_string(freq_l, subghz_regions_list[m1_device_stat.config.ism_band_region].this_region[i][0], 3);
		m1_float_to_string(freq_h, subghz_regions_list[m1_device_stat.config.ism_band_region].this_region[i][1], 3);
		sprintf(freq_range, "%s - %s MHz", freq_l, freq_h);
		u8g2_DrawStr(&m1_u8g2, 0, row, freq_range);
		row += 10;
	}
	m1_u8g2_nextpage(); // Update display RAM

	while (1 ) // Main loop of this task
	{
		;
		; // Do other parts of this task here
		;

		// Wait for the notification from button_event_handler_task to subfunc_handler_task.
		// This task is the sub-task of subfunc_handler_task.
		// The notification is given in the form of an item in the main queue.
		// So let read the main queue.
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				// Notification is only sent to this task when there's any button activity,
				// so it doesn't need to wait when reading the event from the queue
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // user wants to exit?
				{
					; // Do extra tasks here if needed

					xQueueReset(main_q_hdl); // Reset main q before return
					break; // Exit and return to the calling task (subfunc_handler_task)
				} // if ( m1_buttons_status[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				else
				{
					; // Do other things for this task, if needed
				}
			} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			else
			{
				; // Do other things for this task
			}
		} // if (ret==pdTRUE)
	} // while (1 ) // Main loop of this task

} // void sub_ghz_regional_information(void)



/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
/* Radio Settings — TX Power, Default Modulation, ISM Region                  */
/*============================================================================*/

#define RADIO_SETTINGS_ITEMS     3
#define RADIO_SETTINGS_TX_POWER  0
#define RADIO_SETTINGS_MODULATION 1
#define RADIO_SETTINGS_REGION    2

static void radio_settings_draw(uint8_t sel)
{
	m1_u8g2_firstpage();
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
	m1_draw_text(&m1_u8g2, 2, 10, 124, "Radio Settings", TEXT_ALIGN_CENTER);

	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

	/* TX Power row */
	if (sel == RADIO_SETTINGS_TX_POWER)
	{
		u8g2_DrawBox(&m1_u8g2, 0, 14, 128, 12);
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	}
	m1_draw_text(&m1_u8g2, 4, 24, 72, "TX Power:", TEXT_ALIGN_LEFT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
	m1_draw_text(&m1_u8g2, 78, 24, 46, tx_power_labels[subghz_tx_power_idx], TEXT_ALIGN_LEFT);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

	/* Modulation row */
	if (sel == RADIO_SETTINGS_MODULATION)
	{
		u8g2_DrawBox(&m1_u8g2, 0, 27, 128, 12);
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	}
	m1_draw_text(&m1_u8g2, 4, 37, 72, "Modulation:", TEXT_ALIGN_LEFT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
	m1_draw_text(&m1_u8g2, 78, 37, 46, subghz_modulation_text[subghz_scan_config.modulation], TEXT_ALIGN_LEFT);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

	/* ISM Region row */
	if (sel == RADIO_SETTINGS_REGION)
	{
		u8g2_DrawBox(&m1_u8g2, 0, 40, 128, 12);
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	}
	m1_draw_text(&m1_u8g2, 4, 50, 72, "Region:", TEXT_ALIGN_LEFT);
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_B);
	m1_draw_text(&m1_u8g2, 78, 50, 46, subghz_ism_regions_text[m1_device_stat.config.ism_band_region], TEXT_ALIGN_LEFT);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

	m1_draw_bottom_bar(&m1_u8g2, arrowleft_8x8, "Back", "Change", arrowright_8x8);
	m1_u8g2_nextpage();
}

void sub_ghz_radio_settings(void)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t selected = RADIO_SETTINGS_TX_POWER;

	radio_settings_draw(selected);

	while (1)
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret != pdTRUE)
			continue;

		if (q_item.q_evt_type != Q_EVENT_KEYPAD)
			continue;

		ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
		if (ret != pdTRUE)
			continue;

		if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
		{
			xQueueReset(main_q_hdl);
			break;
		}
		else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
		{
			if (selected > 0) selected--;
			else selected = RADIO_SETTINGS_ITEMS - 1;
			radio_settings_draw(selected);
		}
		else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
		{
			selected++;
			if (selected >= RADIO_SETTINGS_ITEMS) selected = 0;
			radio_settings_draw(selected);
		}
		else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK ||
				 this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
		{
			switch (selected)
			{
				case RADIO_SETTINGS_TX_POWER:
					subghz_tx_power_idx = (subghz_tx_power_idx + 1) % TX_POWER_LEVELS;
					break;
				case RADIO_SETTINGS_MODULATION:
					if (subghz_scan_config.modulation == MODULATION_OOK)
						subghz_scan_config.modulation = MODULATION_FSK;
					else
						subghz_scan_config.modulation = MODULATION_OOK;
					break;
				case RADIO_SETTINGS_REGION:
					m1_device_stat.config.ism_band_region++;
					if (m1_device_stat.config.ism_band_region >= SUBGHZ_ISM_BAND_REGIONS_LIST)
						m1_device_stat.config.ism_band_region = 0;
					break;
			}
			radio_settings_draw(selected);
		}
		else if (this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
		{
			switch (selected)
			{
				case RADIO_SETTINGS_TX_POWER:
					if (subghz_tx_power_idx > 0) subghz_tx_power_idx--;
					else subghz_tx_power_idx = TX_POWER_LEVELS - 1;
					break;
				case RADIO_SETTINGS_MODULATION:
					if (subghz_scan_config.modulation == MODULATION_OOK)
						subghz_scan_config.modulation = MODULATION_FSK;
					else
						subghz_scan_config.modulation = MODULATION_OOK;
					break;
				case RADIO_SETTINGS_REGION:
					if (m1_device_stat.config.ism_band_region > 0)
						m1_device_stat.config.ism_band_region--;
					else
						m1_device_stat.config.ism_band_region = SUBGHZ_ISM_BAND_REGIONS_LIST - 1;
					break;
			}
			radio_settings_draw(selected);
		}
	}
} // void sub_ghz_radio_settings(void)



/*============================================================================*/
/*
  * @brief  Initialize the decoder module
  * Input capture mode for both rising and falling edges, given in TIMx_CCR1 for each edge capture
  * @param  None
  * @retval None
 */
/*============================================================================*/
static void sub_ghz_rx_init(void)
{
	GPIO_InitTypeDef gpio_init_struct;
	TIM_IC_InitTypeDef tim_ic_init = {0};
	TIM_MasterConfigTypeDef tim_master_conf = {0};
	uint32_t tim_prescaler_val;

	/* Pin configuration: input floating */
	gpio_init_struct.Pin = SUBGHZ_RX_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_AF_OD;
	gpio_init_struct.Pull = GPIO_NOPULL; // GPIO_PULLDOWN;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_init_struct.Alternate = SUBGHZ_GPIO_AF_RX;
	HAL_GPIO_Init(SUBGHZ_RX_GPIO_PORT, &gpio_init_struct);

	/*  Clock Configuration for TIMER */
	SUBGHZ_RX_TIMER_CLK();

	/* Timer Clock */
	tim_prescaler_val = (uint32_t) (HAL_RCC_GetPCLK2Freq() / 1000000) - 1; // 1MHz

	timerhdl_subghz_rx.Instance = SUBGHZ_RX_TIMER;

	timerhdl_subghz_rx.Init.ClockDivision = 0;
	timerhdl_subghz_rx.Init.CounterMode = TIM_COUNTERMODE_UP;
	timerhdl_subghz_rx.Init.Period = SUBGHZ_RX_TIMEOUT_TIME;
	timerhdl_subghz_rx.Init.Prescaler = tim_prescaler_val;
	timerhdl_subghz_rx.Init.RepetitionCounter = 0;
	timerhdl_subghz_rx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&timerhdl_subghz_rx) != HAL_OK)
	{
		//Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}

  /* Enable the Master/Slave Mode */
  /* SMS = 1000:Combined reset + trigger mode - Rising edge of the selected trigger input (tim_trgi)
   * reinitializes the counter, generates an update of the registers and starts the counter.
   * SMS = 0100: Reset mode - Rising edge of the selected trigger input (tim_trgi)
   * reinitializes the counter and generates an update of the registers. -Recommended by Reference Manual
   */
	tim_master_conf.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;//TIM_SMCR_MSM;
	tim_master_conf.MasterOutputTrigger = TIM_TRGO_RESET;
	if (HAL_TIMEx_MasterConfigSynchronization(&timerhdl_subghz_rx, &tim_master_conf) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}

	tim_ic_init.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	tim_ic_init.ICSelection = TIM_ICSELECTION_DIRECTTI;
	tim_ic_init.ICPrescaler = TIM_ICPSC_DIV1;
	tim_ic_init.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&timerhdl_subghz_rx, &tim_ic_init, SUBGHZ_RX_TIMER_RX_CHANNEL) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}

	/* Enable the TIMx global Interrupt */
	HAL_NVIC_SetPriority(SUBGHZ_RX_TIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(SUBGHZ_RX_TIMER_IRQn);

	/* Configures the TIM Update Request Interrupt source: counter overflow */
	__HAL_TIM_URS_ENABLE(&timerhdl_subghz_rx);

	/* Clear update flag */
	__HAL_TIM_CLEAR_FLAG( &timerhdl_subghz_rx, TIM_FLAG_UPDATE);

	/* Enable TIM Update Event Interrupt Request */
	/* Enable the CCx/CCy Interrupt Request */
	__HAL_TIM_ENABLE_IT( &timerhdl_subghz_rx, TIM_FLAG_UPDATE);

	/* Enable the timer */
	//__HAL_TIM_ENABLE(&timerhdl_subghz_rx);

} // static void sub_ghz_rx_init(void)



/*============================================================================*/
/*
  * @brief  Start receiving radio data
  * @param  None
  * @retval None
 */
/*============================================================================*/
static void sub_ghz_rx_start(void)
{
	if (HAL_TIM_IC_Start_IT(&timerhdl_subghz_rx, SUBGHZ_RX_TIMER_RX_CHANNEL) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}
} // static void sub_ghz_rx_start(void)



/*============================================================================*/
/*
  * @brief  Temporarily stop receiving radio data
  * @param  None
  * @retval None
 */
/*============================================================================*/
static void sub_ghz_rx_pause(void)
{
	if (HAL_TIM_IC_Stop_IT(&timerhdl_subghz_rx, SUBGHZ_RX_TIMER_RX_CHANNEL) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}
} // static void sub_ghz_rx_pause(void)




/*============================================================================*/
/**
  * @brief  De-initializes the peripherals (RCC,GPIO, TIM)
  * @param  None
  * @retval None
  */
/*============================================================================*/
static void sub_ghz_rx_deinit(void)
{
	if ( timerhdl_subghz_rx.State==HAL_TIM_STATE_READY ) // Make sure that the timer has been initialized!
	{
		HAL_TIM_IC_DeInit(&timerhdl_subghz_rx);
		__HAL_TIM_URS_DISABLE(&timerhdl_subghz_rx);
		/* Reset the CC1E Bit */
		timerhdl_subghz_rx.Instance->CCER &= ~TIM_CCER_CC1E;
		timerhdl_subghz_rx.Instance->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
	} // if ( timerhdl_subghz_rx.State==HAL_TIM_STATE_READY )

	SUBGHZ_RX_TIMER_CLK_DIS();
	HAL_NVIC_DisableIRQ(SUBGHZ_RX_TIMER_IRQn);

	HAL_GPIO_DeInit(SUBGHZ_RX_GPIO_PORT, SUBGHZ_RX_GPIO_PIN);

	if ( main_q_hdl != NULL)
		xQueueReset(main_q_hdl);
	//arrfree(subghz_rx_q);
} // static void sub_ghz_rx_deinit(void)




/*============================================================================*/
/*
  * @brief  Initialize ring buffers to receive RF data
  * @param  None
  * @retval 0 if success
 */
/*============================================================================*/
static uint8_t sub_ghz_ring_buffers_init(void)
{
	subghz_front_buffer_size = SUBGHZ_RAW_DATA_SAMPLES_MAX;

	while ( true )
	{
		subghz_front_buffer = malloc(subghz_front_buffer_size*sizeof(uint16_t));
		if ( subghz_front_buffer )
			break;
		subghz_front_buffer_size /= 2;
		if ( subghz_front_buffer_size < 256 )
			break;
	} // while ( true )

	while ( subghz_front_buffer )
	{
		subghz_ring_read_buffer = malloc(SUBGHZ_RAW_DATA_SAMPLES_TO_RW*2); // Each sample has a 2-byte value
		if ( !subghz_ring_read_buffer )
			break;
		subghz_sdcard_write_buffer = malloc(SUBGHZ_FORTMATTED_DATA_SAMPLES_TO_RW);
		if ( !subghz_sdcard_write_buffer )
			break;
		m1_ringbuffer_init(&subghz_rx_rawdata_rb, (uint8_t *)subghz_front_buffer, subghz_front_buffer_size, sizeof(uint16_t));

		M1_LOG_I(M1_LOGDB_TAG, "sub_ghz_ring_buffers_init %d\r\n", subghz_front_buffer_size);

		return 0;
	} // while ( subghz_front_buffer )

	return 1;
} // static uint8_t sub_ghz_ring_buffers_init(void)


/*============================================================================*/
/*
  * @brief  Initialize the radio to replay the recorded raw data
  * @param  None
  * @retval None
 */
/*============================================================================*/
static void sub_ghz_tx_raw_init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	GPIO_InitTypeDef gpio_init_struct = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
	uint32_t tim_prescaler_val;

	/* Pin configuration: output push-pull */
	gpio_init_struct.Pin = SUBGHZ_TX_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;
	gpio_init_struct.Pull = GPIO_PULLDOWN;//GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_init_struct.Alternate = SUBGHZ_GPIO_AF_TX;
	HAL_GPIO_Init(SUBGHZ_TX_GPIO_PORT, &gpio_init_struct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SUBGHZ_TX_GPIO_PORT, SUBGHZ_TX_GPIO_PIN, GPIO_PIN_RESET);

	/*  Clock Configuration for TIMER */
	SUBGHZ_TX_TIMER_CLK();

	/* Timer Clock */
	tim_prescaler_val = (uint32_t) (HAL_RCC_GetPCLK2Freq() / 1000000) - 1; // 1MHz

	timerhdl_subghz_tx.Instance = SUBGHZ_TX_CARRIER_TIMER;
	timerhdl_subghz_tx.Init.Prescaler = tim_prescaler_val;
	timerhdl_subghz_tx.Init.CounterMode = TIM_COUNTERMODE_UP;
	timerhdl_subghz_tx.Init.Period = 0; // temporary value
	timerhdl_subghz_tx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timerhdl_subghz_tx.Init.RepetitionCounter = 0;
	timerhdl_subghz_tx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&timerhdl_subghz_tx) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&timerhdl_subghz_tx, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&timerhdl_subghz_tx, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCNPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfigOC.Pulse = 0; // temporary value
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if ( HAL_TIM_PWM_ConfigChannel(&timerhdl_subghz_tx, &sConfigOC, SUBGHZ_TX_TIMER_TX_CHANNEL) != HAL_OK)
	{
	    Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&timerhdl_subghz_tx, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}

	// Disable Output Compare 4 preload enable so that the CCR4 register will be updated immediately when its value changes
	//__HAL_TIM_DISABLE_OCxPRELOAD(&timerhdl_subghz_tx, SUBGHZ_TX_TIMER_TX_CHANNEL);

    /* Peripheral clock enable */
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    /* TIM1 DMA Init */
    /* GPDMA1_REQUEST_TIM1_UP Init */
    hdma_subghz_tx.Instance = GPDMA1_Channel0;
    hdma_subghz_tx.Init.Request = GPDMA1_REQUEST_TIM1_UP;
    hdma_subghz_tx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    hdma_subghz_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_subghz_tx.Init.SrcInc = DMA_SINC_INCREMENTED;
    hdma_subghz_tx.Init.DestInc = DMA_DINC_FIXED;
    hdma_subghz_tx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
    hdma_subghz_tx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
    hdma_subghz_tx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    hdma_subghz_tx.Init.SrcBurstLength = 1;
    hdma_subghz_tx.Init.DestBurstLength = 1;
    hdma_subghz_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    hdma_subghz_tx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    hdma_subghz_tx.Init.Mode = DMA_NORMAL;

    if (HAL_DMA_Init(&hdma_subghz_tx) != HAL_OK)
    {
    	Error_Handler();
    }
    __HAL_LINKDMA(&timerhdl_subghz_tx, hdma[TIM_DMA_ID_UPDATE], hdma_subghz_tx);

    if (HAL_DMA_ConfigChannelAttributes(&hdma_subghz_tx, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
    	Error_Handler();
    }

    /* GPDMA1 interrupt init */
	/* Clear all interrupt flags */
	__HAL_DMA_CLEAR_FLAG(&hdma_subghz_tx, DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE
						| DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_SUSP | DMA_FLAG_TO);
	// IRQ priority should be lower than that of the Timer
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 5);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

	__HAL_TIM_ENABLE_DMA(&timerhdl_subghz_tx, TIM_DMA_UPDATE);
	/* Enable TIM Update Event Interrupt Request */
	__HAL_TIM_ENABLE_IT(&timerhdl_subghz_tx, TIM_FLAG_UPDATE);

	/* Peripheral interrupt init */
	HAL_NVIC_SetPriority(SUBGHZ_TX_TIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(SUBGHZ_TX_TIMER_IRQn);
} // static void sub_ghz_tx_raw_init(void)



/*============================================================================*/
/**
  * @brief  De-initialize buffers
  * @param  None
  * @retval None
  */
/*============================================================================*/
static void sub_ghz_ring_buffers_deinit(void)
{
	if ( subghz_front_buffer )
	{
		free(subghz_front_buffer);
		subghz_front_buffer = NULL;
		subghz_front_buffer_size = 0;
	} // if ( subghz_front_buffer )

	if ( subghz_ring_read_buffer )
	{
		free(subghz_ring_read_buffer);
		subghz_ring_read_buffer = NULL;
	} // if ( subghz_ring_read_buffer )

	if ( subghz_sdcard_write_buffer )
	{
		free(subghz_sdcard_write_buffer);
		subghz_sdcard_write_buffer = NULL;
	}
	subghz_record_mode_flag = false;
	M1_LOG_I(M1_LOGDB_TAG, "sub_ghz_ring_buffers_deinit %d\r\n", subghz_back_buffer_size);
} // static void sub_ghz_ring_buffers_deinit(void)


/*============================================================================*/
/**
  * @brief  De-initializes the peripherals (RCC,GPIO, TIM)
  * @param  None
  * @retval None
  */
/*============================================================================*/
static void sub_ghz_tx_raw_deinit(void)
{
	GPIO_InitTypeDef gpio_init_struct = {0};

	/* Pin configuration: output push-pull */
	gpio_init_struct.Pin = SUBGHZ_TX_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_ANALOG;
	gpio_init_struct.Pull = GPIO_PULLUP;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SUBGHZ_TX_GPIO_PORT, &gpio_init_struct);

	//HAL_DMA_Abort_IT();
	HAL_NVIC_DisableIRQ(GPDMA1_Channel0_IRQn);
	if ( hdma_subghz_tx.Instance != NULL )
	{
		HAL_DMA_DeInit(&hdma_subghz_tx);
		__HAL_DMA_DISABLE_IT(&hdma_subghz_tx, (DMA_IT_TC | DMA_IT_DTE | DMA_IT_ULE | DMA_IT_USE | DMA_IT_TO));
	} // if ( hdma_subghz_tx.Instance!=NULL )

	if ( timerhdl_subghz_tx.Instance != NULL )
	{
		HAL_TIMEx_PWMN_Stop(&timerhdl_subghz_tx, SUBGHZ_TX_TIMER_TX_CHANNEL);
		__HAL_TIM_DISABLE_DMA(&timerhdl_subghz_tx, TIM_DMA_UPDATE);
		__HAL_TIM_DISABLE_IT(&timerhdl_subghz_tx, TIM_FLAG_UPDATE);
	} // if ( timerhdl_subghz_tx.Instance != NULL )

	SUBGHZ_TX_TIMER_CLK_DIS();
	HAL_NVIC_DisableIRQ(SUBGHZ_TX_TIMER_IRQn);

	//SI446x_Set_Tx_Power(12);
	sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, SUB_GHZ_BAND_EOL, 0, 0);

	if ( main_q_hdl != NULL )
		xQueueReset(main_q_hdl);
} // static void sub_ghz_tx_raw_deinit(void)



/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
static void sub_ghz_raw_tx_stop(void)
{
	uint32_t timeout;

	// Stop DMA
	__HAL_DMA_DISABLE(&hdma_subghz_tx);

	// Stop timer
	//__HAL_TIM_DISABLE(&timerhdl_subghz_tx);
	timerhdl_subghz_tx.Instance->CR1 &= ~(TIM_CR1_CEN);
	//hdma_subghz_tx.Instance->CCR &= ~DMA_CCR_EN;
	/* Check if the DMA channel is effectively disabled (timeout ~10ms at 250MHz) */
	timeout = 2500000UL;
	while ( (hdma_subghz_tx.Instance->CCR & DMA_CCR_EN) != 0U )
	{
		if ( --timeout == 0 )
		{
			hdma_subghz_tx.Instance->CCR &= ~DMA_CCR_EN; // Force disable
			break;
		}
	}
	__HAL_TIM_DISABLE_DMA(&timerhdl_subghz_tx, TIM_DMA_UPDATE);
	// Clear the update interrupt flag
	__HAL_TIM_CLEAR_FLAG(&timerhdl_subghz_tx, TIM_FLAG_UPDATE);
} // static void sub_ghz_raw_tx_stop(void)


/*============================================================================*/
/**
  * @brief This function allocates memory for data buffers, reads data file header and checks its format
  * @param  None
  * @retval error code
  */
/*============================================================================*/
static uint8_t sub_ghz_raw_samples_init(void)
{
	uint16_t sdcard_read_result;
	uint8_t i, error, key_len, *token;
	uint8_t *psdcard_dat_buffer = NULL;

	do
	{
		error = m1_sdm_get_logging_error();
		if ( error )
			break;
		sdcard_dat_buffer_base = m1_malloc(M1_SDM_MIN_BUFFER_SIZE);
		if (sdcard_dat_buffer_base==NULL)
		{
			error = 1;
			break;
		}
		sdcard_dat_buffer = sdcard_dat_buffer_base + M1_SDM_MIN_BUFFER_SIZE/2; // Start at the middle of the buffer
		sdcard_dat_read_size = M1_SDM_MIN_BUFFER_SIZE/4; // Limit the reading size from SD card to avoid data error
		subghz_back_buffer_size = SUBGHZ_RAW_DATA_SAMPLES_MAX;
		while ( true )
		{
			subghz_back_buffer = malloc(subghz_back_buffer_size*sizeof(uint16_t));
			if ( subghz_back_buffer )
				break;
			subghz_back_buffer_size /= 2;
			if ( subghz_back_buffer_size < 256 )
				break;
		} // while ( true )
		if ( !subghz_back_buffer )
		{
			error = 1;
			break;
		}
		M1_LOG_D(M1_LOGDB_TAG, "sub_ghz_raw_samples_init %d\r\n", subghz_back_buffer_size);
		raw_samples_buffer_size = (subghz_back_buffer_size < subghz_front_buffer_size)?subghz_back_buffer_size:subghz_front_buffer_size;
		double_buffer_ptr[0] = subghz_front_buffer;
		double_buffer_ptr[1] = subghz_back_buffer;

		error = m1_fb_open_file(&datfile_info.dat_file_hdl, datfile_info.dat_filename);
		if (error)
			break;

		sdcard_dat_file_size = f_size(&datfile_info.dat_file_hdl);
		if ( sdcard_dat_read_size > sdcard_dat_file_size )
			sdcard_dat_read_size = sdcard_dat_file_size;
		sdcard_read_result = m1_fb_read_from_file(&datfile_info.dat_file_hdl, sdcard_dat_buffer, sdcard_dat_read_size);
		if ( sdcard_read_result!=sdcard_dat_read_size )
		{
			error = 1;
			break;
		}
		sdcard_dat_buffer[sdcard_dat_read_size] = '\0'; // Add end of string to the buffer
		sdcard_buffer_run_ptr = sdcard_dat_buffer;

		psdcard_dat_buffer = malloc(sdcard_dat_read_size + 1);
		if ( psdcard_dat_buffer==NULL )
		{
			error = 1;
			break;
		}
		memcpy(psdcard_dat_buffer, sdcard_dat_buffer, sdcard_dat_read_size + 1); // Duplicate this file header

		key_len = SUB_GHZ_DATAFILE_KEY_FORMAT_N;
		i = 0;
		token = strtok(psdcard_dat_buffer, "\r\n"); // Tokenize this duplicated buffer
		if ( strstr(token, subghz_datfile_keywords[i]) )
		{
			if ( strstr(token, SUB_GHZ_DATAFILE_FILETYPE_NOISE) )
				key_len = SUB_GHZ_DATAFILE_RAW_FORMAT_N;
			else if ( strstr(token, SUB_GHZ_DATAFILE_FILETYPE_PACKET) )
				key_len = SUB_GHZ_DATAFILE_KEY_FORMAT_N;
			else
				token = NULL; // Unknown format
		} // if ( strstr(token, subghz_datfile_keywords[i]) )
		else
		{
			token = NULL; // Terminate
		}

		while ( token!=NULL )
		{
			if ( strstr(token, subghz_datfile_keywords[i])==NULL )
				break;
			sdcard_buffer_run_ptr += strlen(token) + 2; // 2 for "\r\n"
			if ( ++i >= key_len )
				break;
			token = strtok(NULL, "\r\n");
		} // while ( token!=NULL )
		if ( i < key_len )
		{
			error = 1;
			break;
		} // if ( i < SUB_GHZ_DATAFILE_KEY_FORMAT_N )

		sdcard_dat_buffer_end_pos = (uint32_t)sdcard_dat_buffer + sdcard_dat_read_size;

	} while(0); // while (0)

	if ( psdcard_dat_buffer!=NULL )
		free(psdcard_dat_buffer);

	return error;

} // static uint8_t sub_ghz_raw_samples_init(void)


/*============================================================================*/
/**
 * @brief
 * @param  None
 * @retval None
 */
/*============================================================================*/
static void sub_ghz_raw_samples_deinit(bool discard_samples)
{
	if ( subghz_back_buffer )
	{
		free(subghz_back_buffer);
		subghz_back_buffer = NULL;
	}
	if ( sdcard_dat_buffer_base )
	{
		free(sdcard_dat_buffer_base);
		sdcard_dat_buffer_base = NULL;
		sdcard_dat_buffer = NULL;
	}
	m1_fb_close_file(&datfile_info.dat_file_hdl);
	if (discard_samples)
		m1_fb_delete_file(datfile_info.dat_filename);

	M1_LOG_D(M1_LOGDB_TAG, "sub_ghz_raw_samples_deinit %d\r\n", subghz_back_buffer_size);
} // static void sub_ghz_raw_samples_deinit(bool discard_samples)



/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
static uint8_t sub_ghz_parse_raw_data(uint8_t buffer_ptr_id)
{
	char *token, *endptr;
	uint8_t error_code, crlf;
	uint16_t rd_samples_count;
	uint16_t sdcard_read_result, number;

	rd_samples_count = 0;
	raw_samples_count = 0;
	error_code = 0;
	crlf = 0;

	while ( true )
	{
		token = strstr(sdcard_buffer_run_ptr, " "); // Find next numbers
		if ( token==NULL )
		{
			token = strstr(sdcard_buffer_run_ptr, "\r\n"); // Find end of current data block
			do
			{
				if (token==NULL) // Possibly last few characters of this buffer
				{
					if ( (sdcard_buffer_run_ptr + 10) < sdcard_dat_buffer_end_pos ) // Remaining data is too long. Not valid!
					{
						error_code = SUB_GHZ_RAW_DATA_PARSER_ERROR_L1;
						break;
					} // if ( (sdcard_buffer_run_ptr + 10) < sdcard_dat_buffer_end_pos )
					sdcard_dat_buffer_end_pos -= (uint32_t)sdcard_buffer_run_ptr; // Remaining characters
					if ( sdcard_dat_buffer_end_pos ) // Are there remaining characters?
					{
						// If the write buffer address is not aligned, the data read from SDcard may be corrupted randomly when writing.
						// So if there're remaining characters, let copy the last few characters to before the beginning of the aligned buffer
						sdcard_dat_buffer -= sdcard_dat_buffer_end_pos; // Move backward a few positions
						memcpy(sdcard_dat_buffer, sdcard_buffer_run_ptr, sdcard_dat_buffer_end_pos);
						sdcard_dat_buffer += sdcard_dat_buffer_end_pos; // Restore
					}
					sdcard_dat_file_size -= sdcard_dat_read_size; // Update the remainder
					if ( sdcard_dat_file_size==0 ) // End of file?
					{
						error_code = SUB_GHZ_RAW_DATA_PARSER_COMPLETE;
						if ( sdcard_dat_buffer_end_pos ) // Last number?
						{
							number = strtol((char *)sdcard_buffer_run_ptr, &endptr, 10);
							if ( number!=0 ) // Valid number?
								double_buffer_ptr[buffer_ptr_id][raw_samples_count++] = number;
						} // if ( sdcard_dat_buffer_end_pos )
					} // if ( sdcard_dat_file_size==0 )
					else
					{
						if ( sdcard_dat_read_size > sdcard_dat_file_size ) // Last block to read from file?
							sdcard_dat_read_size = sdcard_dat_file_size; // Adjust the read size
						sdcard_read_result = m1_fb_read_from_file(&datfile_info.dat_file_hdl, sdcard_dat_buffer, sdcard_dat_read_size);
						if ( sdcard_read_result!=sdcard_dat_read_size )
						{
							error_code = SUB_GHZ_RAW_DATA_PARSER_ERROR_L2;
							break;
						}
						sdcard_buffer_run_ptr = sdcard_dat_buffer; // Update moving buffer pointer
						sdcard_buffer_run_ptr -= sdcard_dat_buffer_end_pos; // Update address for remaining characters, if any
						sdcard_dat_buffer_end_pos = (uint32_t)sdcard_dat_buffer; // Update moving buffer pointer
						sdcard_dat_buffer_end_pos += sdcard_dat_read_size; // Update buffer end index for this data block
						sdcard_dat_buffer[sdcard_dat_read_size] = '\0'; // Add end of string to the buffer
						token = strstr(sdcard_buffer_run_ptr, " "); // Find next numbers
						if ( token==NULL ) // Possibly error.
						{
							token = strstr(sdcard_buffer_run_ptr, "\r\n"); // Find end of current data block
							if ( token==NULL ) // Real error
								error_code = SUB_GHZ_RAW_DATA_PARSER_ERROR_L3;
							else
								crlf = 1; // CRLF found, likely reaching end of file
							break;
						} // if ( token==NULL )
					} // else
				} // if (token==NULL)
				else
				{
					crlf = 1; // Carriage return and Line feed is found
				} // else
			} while (0);

			if ( error_code )
				break;
		} // if ( token==NULL )

		*token = '\0'; // Add end of string to replace the " " at the end
		token = sdcard_buffer_run_ptr;
		sdcard_buffer_run_ptr += strlen(token) + 1; // Move read pointer to the next item, +1 for " " at the end
		if ( crlf ) // "\r\n" was found
		{
			crlf = 0; // reset
			sdcard_buffer_run_ptr += 1; // Move pointer forward one more position
		}
		if (*token=='+' || *token=='-')
			*token = '0'; // Replace the +/- sign before the number with number 0, if any
		number = strtol(token, &endptr, 10);
		if ( number==0 && endptr==token ) // Not a valid number?
		{
			if ( strstr(token, SUB_GHZ_DATAFILE_DATA_KEYWORD)==NULL ) // Not the main key word?
			{
				if ( strlen(token)!=0 ) // In case the data file has more than one CR/LF at the end of file
				{
					error_code = SUB_GHZ_RAW_DATA_PARSER_ERROR_L4;
					break;
				} // if ( strlen(token)!=0 )
			} // if ( strstr(token, SUB_GHZ_DATAFILE_DATA_KEYWORD)==NULL )
			continue;
		} // if ( number==0 && endptr==token )

		rd_samples_count++;
		if ( rd_samples_count >= SUBGHZ_RAW_DATA_SAMPLES_TO_RW )
		{
			rd_samples_count = 0; // reset
		}
		double_buffer_ptr[buffer_ptr_id][raw_samples_count++] = number;
		if ( raw_samples_count >= raw_samples_buffer_size )
		{
			error_code = SUB_GHZ_RAW_DATA_PARSER_READY;
			break;
			// Start transmitting here,
			// and continue to fill the other tx buffer
		} // if ( raw_samples_count >= raw_samples_buffer_size )
	} // while (true)

	return error_code;
} // static uint8_t sub_ghz_parse_raw_data(uint8_t buffer_ptr_id)



/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
static void sub_ghz_transmit_raw(uint32_t source, uint32_t dest, uint32_t len, uint8_t repeat)
{
	if ( source==0 )
		return;

	if ( dest==0 )
		return;

	if ( len==0 )
		return;

	//len &= ~(uint16_t)1; // Make length an even number. DMA doesn't work with odd length for unknown reason!
	// Save these data for repeat
	subghz_decenc_ctl.ntx_raw_len = (len<<1); // Convert length to byte
	subghz_decenc_ctl.ntx_raw_src = source;
	subghz_decenc_ctl.ntx_raw_dest = dest;
	subghz_decenc_ctl.ntx_raw_repeat = repeat;
	subghz_tx_tc_flag = 0;

	/* Enable the DMA channel */
	/**
	  * @brief  Start the DMA data transfer.
	  * @param  hdma DMA handle
	  * @param  src      : The source memory Buffer address.
	  * @param  dst      : The destination memory Buffer address.
	  * @param  length   : The size of a source block transfer in byte.
	  * @retval HAL status
	  */
	HAL_StatusTypeDef ret = TIM_DMA_Start_IT(timerhdl_subghz_tx.hdma[TIM_DMA_ID_UPDATE], source, dest, subghz_decenc_ctl.ntx_raw_len);
	if ( ret != HAL_OK)
		return;

	timerhdl_subghz_tx.Instance->CCR4 = 0; // initial value

	//	__HAL_TIM_URS_ENABLE(&timerhdl_subghz_tx); // Enable URS to temporarily disable the UIF when the UG bit is set
	// Generate Update Event (set UG bit) to reload the DMA source data[0] to the ARR register
	HAL_TIM_GenerateEvent(&timerhdl_subghz_tx, TIM_EVENTSOURCE_UPDATE);
	// Do it again to reload the DMA source data[1] to the ARR register, and reload the DMA source data[0] to the ARR shadow register
	HAL_TIM_GenerateEvent(&timerhdl_subghz_tx, TIM_EVENTSOURCE_UPDATE);
	//	__HAL_TIM_URS_DISABLE(&timerhdl_subghz_tx); // Disable URS to enable the UIF again

    // Start the timer
	HAL_TIMEx_PWMN_Start(&timerhdl_subghz_tx, SUBGHZ_TX_TIMER_TX_CHANNEL);
} // static void sub_ghz_transmit_raw(uint32_t source, uint32_t dest, uint32_t len, uint8_t repeat)



/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
static void sub_ghz_transmit_raw_restart(uint32_t source, uint32_t len)
{
	sub_ghz_raw_tx_stop();

	if ( (source!=0) && (len!=0) ) // New data source and length?
	{
		//len &= ~(uint16_t)1; // Make length an even number. DMA doesn't work with odd length for unknown reason!
		subghz_decenc_ctl.ntx_raw_len = (len<<1); // Convert length to byte
		subghz_decenc_ctl.ntx_raw_src = source;
	} // if ( (source!=0) && (len!=0) )
	/* Enable the DMA channel */
	/**
	  * @brief  Start the DMA data transfer.
	  * @param  hdma DMA handle
	  * @param  src      : The source memory Buffer address.
	  * @param  dst      : The destination memory Buffer address.
	  * @param  length   : The size of a source block transfer in byte.
	  * @retval HAL status
	  */

	__HAL_TIM_ENABLE_DMA(&timerhdl_subghz_tx, TIM_DMA_UPDATE);

	MODIFY_REG(hdma_subghz_tx.Instance->CBR1, DMA_CBR1_BNDT, (subghz_decenc_ctl.ntx_raw_len & DMA_CBR1_BNDT));
	/* Clear all interrupt flags */
	__HAL_DMA_CLEAR_FLAG(&hdma_subghz_tx, DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE
						| DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_SUSP | DMA_FLAG_TO);
	// Configure DMA channel source address
	hdma_subghz_tx.Instance->CSAR = subghz_decenc_ctl.ntx_raw_src;
	// Configure DMA channel destination address
	hdma_subghz_tx.Instance->CDAR = subghz_decenc_ctl.ntx_raw_dest;

    /* Enable common interrupts: Transfer Complete and Transfer Errors ITs */
	__HAL_DMA_ENABLE(&hdma_subghz_tx);

	/* Temporarily disable the complementary PWM output  */
	//__HAL_TIM_MOE_DISABLE(&timerhdl_subghz_tx);

	timerhdl_subghz_tx.Instance->CCR4 = 0; // initial value

	// Generate Update Event (set UG bit) to reload the DMA source data[0] to the ARR register
	HAL_TIM_GenerateEvent(&timerhdl_subghz_tx, TIM_EVENTSOURCE_UPDATE);
	// Do it again to reload the DMA source data[1] to the ARR register, and reload the DMA source data[0] to the ARR shadow register
	HAL_TIM_GenerateEvent(&timerhdl_subghz_tx, TIM_EVENTSOURCE_UPDATE);

	/* Enable the complementary PWM output  */
	//__HAL_TIM_MOE_ENABLE(&timerhdl_subghz_tx);

	// Start the timer
	__HAL_TIM_ENABLE(&timerhdl_subghz_tx);
} // static void sub_ghz_transmit_raw_restart(uint32_t source, uint32_t len)




/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
static uint8_t sub_ghz_rx_raw_save(bool header_init, bool last_data)
{
	char *prn_buffer;
	uint32_t freq32;
	uint16_t count, n_samples_to_rw, *pdata;
	char *sign_text[2] = {"+", ""};
	uint8_t *pfillbuffer;
	uint8_t sign;

	prn_buffer = malloc(64);
	if (prn_buffer == NULL)
		return 1;
	pfillbuffer = subghz_sdcard_write_buffer;
	if ( header_init )
	{
		sprintf(pfillbuffer, "%s M1 SubGHz %s\r\n", subghz_datfile_keywords[0], SUB_GHZ_DATAFILE_FILETYPE_KEYWORD);
		sprintf(prn_buffer, "%s %d.%d\r\n", subghz_datfile_keywords[1], m1_device_stat.config.fw_version_major, m1_device_stat.config.fw_version_minor);
		strcat(pfillbuffer, prn_buffer);
		freq32 = subghz_band_steps[subghz_scan_config.band][0]*1000000; // Convert frequency from MHz to Hz
		sprintf(prn_buffer, "%s %lu\r\n", subghz_datfile_keywords[2], freq32);
		strcat(pfillbuffer, prn_buffer);
		sprintf(prn_buffer, "%s %s\r\n", subghz_datfile_keywords[3], subghz_modulation_text[subghz_scan_config.modulation]);
		strcat(pfillbuffer, prn_buffer);
		m1_sdm_fill_buffer(pfillbuffer, strlen(pfillbuffer));
		free(prn_buffer);
		return 0;
	} // if ( header_init )

	sprintf(pfillbuffer, "%s", SUB_GHZ_DATAFILE_DATA_KEYWORD);
	//m1_test_gpio_pull_high();
	//sub_ghz_rx_pause();
	n_samples_to_rw = SUBGHZ_RAW_DATA_SAMPLES_TO_RW;
	if ( last_data )
	{
		n_samples_to_rw = ringbuffer_get_data_slots(&subghz_rx_rawdata_rb);
		if ( n_samples_to_rw > SUBGHZ_RAW_DATA_SAMPLES_TO_RW ) // This should never happen!
			n_samples_to_rw = SUBGHZ_RAW_DATA_SAMPLES_TO_RW;
	} // if ( last_data )
	m1_ringbuffer_read(&subghz_rx_rawdata_rb, subghz_ring_read_buffer, n_samples_to_rw);
	//sub_ghz_rx_start();
	//m1_test_gpio_pull_low();
	pdata = (uint16_t *)subghz_ring_read_buffer;
	if ( n_samples_to_rw & 0x01 ) // Odd number of samples?
	{
		pdata[n_samples_to_rw] = INTERPACKET_GAP_MIN; // Extra dummy data
		n_samples_to_rw++; // Make even number
	}
	sign = 0;
	for (count=0; count<n_samples_to_rw; count++)
	{
		sprintf(prn_buffer, " %s%u", sign_text[sign], *pdata);
		strcat(pfillbuffer, prn_buffer);
		pdata++;
		sign ^= 1;
	}
	strcat(pfillbuffer, "\r\n");

	m1_sdm_fill_buffer(pfillbuffer, strlen(pfillbuffer));

	if ( prn_buffer!=NULL )
		free(prn_buffer);
} // static uint8_t sub_ghz_rx_raw_save(bool header_init, bool last_data)



/*============================================================================*/
/**
  * @brief Init the raw samples for replay
  * @param  None
  * @retval Error code
  */
/*============================================================================*/
static uint8_t sub_ghz_raw_replay_init(void)
{
	uint8_t ret_code;

	sub_ghz_raw_samples_deinit(false);
	ret_code = sub_ghz_raw_samples_init();
	while ( !ret_code )
	{
		ret_code = sub_ghz_parse_raw_data(0);
		if ( ret_code & SUB_GHZ_RAW_DATA_PARSER_ERROR_MASK )
		{
			ret_code = 1; // Change to common error code
			break;
		}
		sub_ghz_transmit_raw_restart((uint32_t)double_buffer_ptr[0], raw_samples_count);
		if ( ret_code==SUB_GHZ_RAW_DATA_PARSER_READY ) // There're more samples to read?
			ret_code = sub_ghz_parse_raw_data(1);
		else // COMPLETE
			ret_code = SUB_GHZ_RAW_DATA_PARSER_STOPPED; // No more sample
		break;
	} // while ( !ret_code )

	return ret_code;
} // static uint8_t sub_ghz_raw_replay_init(void)



/*============================================================================*/
/**
  * @brief Play recorded samples or samples from SD card
  * @param record_mode true if the record mode is active
  * @retval Error code
  */
/*============================================================================*/
static uint8_t sub_ghz_replay_start(bool record_mode, S_M1_SubGHz_Band band, uint8_t channel, uint8_t power)
{
	uint8_t ret_code = 0;

	if ( sub_ghz_fcc_ism_band_check(band, channel) )
	{
		ret_code = 1;
		record_mode = 0;
		m1_buzzer_notification();
	} // if ( sub_ghz_fcc_ism_band_check(band, channel) )

	if ( record_mode )
	{
		sub_ghz_rx_deinit();
		m1_sdm_task_stop(); // Stop sampling raw data and flush data to SD card and then close file
		ret_code = sub_ghz_raw_samples_init();
	} // if ( record_mode )
	while ( !ret_code )
	{
		ret_code = sub_ghz_parse_raw_data(0);
		if ( ret_code & SUB_GHZ_RAW_DATA_PARSER_ERROR_MASK )
		{
			ret_code = 1; // Change to common error code
			break;
		}
		if ( raw_samples_count==0 ) // No samples found in Record mode or Replay mode?
		{
			if ( record_mode ) // Try to replay in Record mode?
			{
				raw_samples_count = 10; // Just play dummy data
			}
			else
			{
				ret_code = 1; // Change to common error code
				break;
			}
		} // if ( raw_samples_count==0 )
		M1_LOG_I(M1_LOGDB_TAG, "sub_ghz_replay_start: %d samples\r\n", raw_samples_count);
		sub_ghz_tx_raw_init();
		sub_ghz_set_opmode(SUB_GHZ_OPMODE_TX, band, channel, power);
		sub_ghz_transmit_raw((uint32_t)double_buffer_ptr[0], (uint32_t)&timerhdl_subghz_tx.Instance->ARR, raw_samples_count, SUBGHZ_TX_RAW_REPLAY_REPEAT_DEFAULT);
		if ( ret_code==SUB_GHZ_RAW_DATA_PARSER_READY ) // There're more samples to read?
			ret_code = sub_ghz_parse_raw_data(1);
		else // COMPLETE
			ret_code = SUB_GHZ_RAW_DATA_PARSER_STOPPED; // No more sample
		break;
	} // while ( !ret_code )
	if ( ret_code==1 )
	{
		sub_ghz_raw_samples_deinit(record_mode);
		ret_code = 0; // Reset
	} // if ( ret_code==1 )

	return ret_code;
} // static uint8_t sub_ghz_replay_start(bool record_mode, S_M1_SubGHz_Band band, uint8_t channel, uint8_t power)



/*============================================================================*/
/**
  * @brief Continue to replay the samples, and update its status
  * @param
  * @retval Error code
  */
/*============================================================================*/
static uint8_t sub_ghz_replay_continue(uint8_t ret_code_in)
{
	uint8_t ret_code;

	ret_code = ret_code_in;
	switch (ret_code)
	{
		case SUB_GHZ_RAW_DATA_PARSER_READY:
		case SUB_GHZ_RAW_DATA_PARSER_COMPLETE:
			sub_ghz_transmit_raw_restart((uint32_t)double_buffer_ptr[double_buffer_ptr_id], raw_samples_count);
			double_buffer_ptr_id ^= 1; // Update raw samples buffer
			if ( ret_code==SUB_GHZ_RAW_DATA_PARSER_READY )
				ret_code = sub_ghz_parse_raw_data(double_buffer_ptr_id);
			else if ( ret_code==SUB_GHZ_RAW_DATA_PARSER_COMPLETE )
				ret_code = SUB_GHZ_RAW_DATA_PARSER_STOPPED;
			break;

		case SUB_GHZ_RAW_DATA_PARSER_ERROR_MASK:
			ret_code = SUB_GHZ_RAW_DATA_PARSER_IDLE;
			subghz_decenc_ctl.ntx_raw_repeat = 0; // Force to stop
			break;

		case SUB_GHZ_RAW_DATA_PARSER_STOPPED:
			if ( subghz_decenc_ctl.ntx_raw_repeat-- )
			{
				ret_code = sub_ghz_raw_replay_init();
				double_buffer_ptr_id = 1;
			} // if ( subghz_decenc_ctl.ntx_raw_repeat-- )
			else
			{
				ret_code = SUB_GHZ_RAW_DATA_PARSER_IDLE;
			}

			if ( ret_code==SUB_GHZ_RAW_DATA_PARSER_IDLE )
			{
				sub_ghz_raw_tx_stop();
				sub_ghz_raw_samples_deinit(false);
				sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, SUB_GHZ_BAND_EOL, 0, 0);
				subghz_decenc_ctl.ntx_raw_repeat = 0;
			} // if ( ret_code==SUB_GHZ_RAW_DATA_PARSER_IDLE )
			break;

		default:
			break;
	} // switch (ret_code)

	return ret_code;

} // static uint8_t sub_ghz_replay_continue(uint8_t ret_code_in)



/*============================================================================*/
/**
  * @brief Check a frequency if it falls within the ranges of the FCC ISM list
  * @param
  * @retval 0 if no violation
  */
/*============================================================================*/
static uint8_t sub_ghz_fcc_ism_band_check(uint8_t band, uint8_t channel)
{
	float freq;
	uint8_t i, ret;

	ret = 1;

	freq = subghz_band_steps[band][0];
	freq += channel*CHANNEL_STEP;
	for (i=0; i<subghz_regions_list[m1_device_stat.config.ism_band_region].bands_list; i++)
	{
		if ( (freq >= subghz_regions_list[m1_device_stat.config.ism_band_region].this_region[i][0]) &&
				(freq <= subghz_regions_list[m1_device_stat.config.ism_band_region].this_region[i][1]) )
		{
			ret = 0;
			break;
		}
	} // for (i=0; i<subghz_regions_list[m1_device_stat.config.ism_band_region].bands_list; i++)

	return ret;
} // static uint8_t sub_ghz_fcc_ism_band_check(uint8_t band, uint8_t channel)



/*============================================================================*/
/**
  * @brief Rotate the ring buffer, move [tail] or read pointer to the first position of the buffer
  * @param  None
  * @retval None
  */
/*============================================================================*/
static void sub_ghz_buffer_rotate(S_M1_RingBuffer *prb_handle)
{
	uint16_t first, middle, last, next;
	uint16_t swap_val;
	uint16_t data_size;

	middle = prb_handle->tail;
	first = 0;
	last = ringbuffer_get_data_slots(prb_handle);
	data_size = prb_handle->data_size;

	// Even positions: high pulse, odd positions: low pulse
	// Transmitter always starts with a high pulse.
	if ( (middle % 2)!=0 ) // Odd position -> low pulse
	{
		middle++; // Move to the high pulse position
		if ( middle==prb_handle->len ) // overflow?
			middle = 0;
	} // if ( (middle % 2)!=0 )

	next = middle;
	while ( next != first )
	{
		// swap [first] and [next]
		memcpy(&swap_val, &prb_handle->pdata[next*data_size], data_size);
		// copy [first] to [next]
		memcpy(&prb_handle->pdata[next*data_size], &prb_handle->pdata[first*data_size], data_size);
		// copy [next] to [first]
		memcpy(&prb_handle->pdata[first*data_size], &swap_val, data_size);
		first++;
		next++;
		if ( next==last )
			next = middle;
		else if ( first==middle )
			middle = next;
	} // while ( next != first )

} // static void sub_ghz_buffer_rotate(S_M1_RingBuffer *prb_handle)



/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
void sub_ghz_display(SubGHz_Dec_Info_t decoded_data)
{
    char hexString[64];
    uint32_t value;

    value = decoded_data.key;
    if (value)
    {
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        u8g2_DrawBox(&m1_u8g2, 2, 42, 126, 22); // Clear existing content
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
        u8g2_DrawStr(&m1_u8g2, 22, 42, protocol_text[decoded_data.protocol]);
    	if ( decoded_data.protocol==SECURITY_PLUS_20 )
    	{
    		value = subghz_decenc_ctl.n32_rollingcode;
    		sprintf(hexString, "SN 0x%lX Id 0x%X\r\n", subghz_decenc_ctl.n32_serialnumber, subghz_decenc_ctl.n8_buttonid);
    		u8g2_DrawStr(&m1_u8g2, 2, 62, hexString);
    	}
        sprintf(hexString, "0x%lX %ddBm\r\n", value, decoded_data.rssi);
        u8g2_DrawStr(&m1_u8g2, 22, 52, hexString);
		m1_u8g2_nextpage(); // Update display
		M1_LOG_I(M1_LOGDB_TAG, hexString);
        //display_info(decoded_data, 1, raw);
    } // if (value)
    subghz_decenc_ctl.subghz_reset_data();

} // void sub_ghz_display(SubGHz_Dec_Info_t decoded_data)


/*============================================================================*/
/*                                                                            */
/*  SPECTRUM ANALYZER                                                         */
/*  Sweep RSSI across a frequency range and display as bar graph on LCD       */
/*                                                                            */
/*============================================================================*/

#define SPECTRUM_BAR_COUNT     128  /* one bar per LCD pixel column */
#define SPECTRUM_RSSI_MIN     -120  /* dBm floor */
#define SPECTRUM_RSSI_MAX      -30  /* dBm ceiling */
#define SPECTRUM_BAR_HEIGHT     34  /* pixels for bar area (rows 11..44) */
#define SPECTRUM_MIN_SPAN   500000UL   /* 0.5 MHz minimum zoom */
#define SPECTRUM_MAX_SPAN   200000000UL /* 200 MHz maximum zoom */

void sub_ghz_spectrum_analyzer(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    int8_t rssi_values[SPECTRUM_BAR_COUNT];
    struct si446x_reply_GET_MODEM_STATUS_map *pmodemstat;
    char info_str[40];

    uint32_t center_freq;
    uint32_t span;
    uint32_t step;
    uint32_t freq;
    uint8_t  i;
    int16_t  rssi;
    uint8_t  bar_h;
    bool     running = true;
    uint8_t  band_idx = 0;
    bool     custom_view = false;  /* true when user has zoomed/recentered */

    /* Peak tracking */
    uint32_t peak_freq_hz = 0;
    int8_t   peak_rssi = -127;
    uint8_t  peak_bar = 0;

    /* Predefined sweep ranges */
    static const uint32_t sweep_centers[] = {
        307000000UL,  /* 300-315 MHz */
        370000000UL,  /* 345-395 MHz */
        435000000UL,  /* 430-440 MHz */
        915000000UL,  /* 910-920 MHz */
        200000000UL,  /* 142-258 MHz (extended low) */
    };
    static const uint32_t sweep_spans[] = {
        15000000UL,
        50000000UL,
        10000000UL,
        10000000UL,
        116000000UL,
    };
    #define NUM_SWEEP_RANGES  5

    /* Start with first preset */
    center_freq = sweep_centers[band_idx];
    span = sweep_spans[band_idx];

    menu_sub_ghz_init();

    /* Initialize radio */
    if (center_freq < 525000000UL && center_freq >= 284000000UL)
        radio_init_rx_tx(SUB_GHZ_BAND_433, MODEM_MOD_TYPE_OOK, true);
    else
        radio_init_rx_tx(SUB_GHZ_BAND_915, MODEM_MOD_TYPE_OOK, true);

    radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);

    while (running)
    {
        /* Load preset unless user is in custom zoom/center */
        if (!custom_view)
        {
            center_freq = sweep_centers[band_idx];
            span = sweep_spans[band_idx];
        }

        step = span / SPECTRUM_BAR_COUNT;
        if (step == 0) step = 1;

        /* Sweep and find peak */
        freq = center_freq - span / 2;
        peak_rssi = -127;
        peak_bar = 0;
        peak_freq_hz = freq;

        for (i = 0; i < SPECTRUM_BAR_COUNT; i++)
        {
            SI446x_Set_Frequency(freq);
            SI446x_Start_Rx(0);
            HAL_Delay(1);  /* Let AGC settle */

            SI446x_Get_IntStatus(0, 0, 0);
            pmodemstat = SI446x_Get_ModemStatus(0x00);
            rssi = pmodemstat->CURR_RSSI / 2 - MODEM_RSSI_COMP - 70;
            rssi_values[i] = (int8_t)rssi;

            if ((int8_t)rssi > peak_rssi)
            {
                peak_rssi = (int8_t)rssi;
                peak_freq_hz = freq;
                peak_bar = i;
            }

            freq += step;
        }

        /* Compute display values for low/high edges */
        uint32_t lo_hz = center_freq - span / 2;
        uint32_t hi_hz = lo_hz + (uint32_t)step * SPECTRUM_BAR_COUNT;

        /* Draw spectrum */
        u8g2_FirstPage(&m1_u8g2);
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

            /* Row 1 (y=9): peak frequency and RSSI */
            snprintf(info_str, sizeof(info_str), "Pk:%lu.%02lu %ddBm",
                     peak_freq_hz / 1000000UL,
                     (peak_freq_hz % 1000000UL) / 10000UL,
                     peak_rssi);
            u8g2_DrawStr(&m1_u8g2, 0, 9, info_str);

            /* Draw bars (y=11 to y=44, height up to SPECTRUM_BAR_HEIGHT) */
            for (i = 0; i < SPECTRUM_BAR_COUNT; i++)
            {
                int16_t clamped = rssi_values[i];
                if (clamped < SPECTRUM_RSSI_MIN) clamped = SPECTRUM_RSSI_MIN;
                if (clamped > SPECTRUM_RSSI_MAX) clamped = SPECTRUM_RSSI_MAX;
                bar_h = (uint8_t)(((clamped - SPECTRUM_RSSI_MIN) * SPECTRUM_BAR_HEIGHT) /
                                  (SPECTRUM_RSSI_MAX - SPECTRUM_RSSI_MIN));
                if (bar_h > 0)
                    u8g2_DrawVLine(&m1_u8g2, i, 44 - bar_h, bar_h);
            }

            /* Peak marker — small triangle above peak bar */
            if (peak_bar > 0 && peak_bar < 127)
            {
                uint8_t marker_y = 44 - SPECTRUM_BAR_HEIGHT - 2;
                u8g2_DrawPixel(&m1_u8g2, peak_bar, marker_y);
                u8g2_DrawPixel(&m1_u8g2, peak_bar - 1, marker_y - 1);
                u8g2_DrawPixel(&m1_u8g2, peak_bar + 1, marker_y - 1);
            }

            /* Row below bars (y=54): range labels */
            snprintf(info_str, sizeof(info_str), "%lu.%01lu",
                     lo_hz / 1000000UL, (lo_hz % 1000000UL) / 100000UL);
            u8g2_DrawStr(&m1_u8g2, 0, 54, info_str);
            snprintf(info_str, sizeof(info_str), "%lu.%01lu",
                     hi_hz / 1000000UL, (hi_hz % 1000000UL) / 100000UL);
            u8g2_DrawStr(&m1_u8g2, 90, 54, info_str);

            /* Bottom row (y=64): controls hint */
            if (custom_view)
                u8g2_DrawStr(&m1_u8g2, 0, 64, "\x18\x19:Zoom OK:Peak L/R:Pan");
            else
                u8g2_DrawStr(&m1_u8g2, 0, 64, "\x18\x19:Zoom OK:Peak L/R:Band");

        } while (u8g2_NextPage(&m1_u8g2));

        /* Check for button input (non-blocking with short timeout) */
        ret = xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(100));
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                running = false;
            }
            else if (this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                if (custom_view)
                {
                    /* Pan right by 25% of span */
                    center_freq += span / 4;
                }
                else
                {
                    band_idx = (band_idx + 1) % NUM_SWEEP_RANGES;
                }
            }
            else if (this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                if (custom_view)
                {
                    /* Pan left by 25% of span */
                    if (center_freq > span / 4)
                        center_freq -= span / 4;
                }
                else
                {
                    band_idx = (band_idx + NUM_SWEEP_RANGES - 1) % NUM_SWEEP_RANGES;
                }
            }
            else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
            {
                /* Zoom in — halve span */
                if (span > SPECTRUM_MIN_SPAN)
                {
                    span /= 2;
                    if (span < SPECTRUM_MIN_SPAN) span = SPECTRUM_MIN_SPAN;
                    custom_view = true;
                }
            }
            else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
            {
                /* Zoom out — double span, reset to preset if back at original span */
                if (span < SPECTRUM_MAX_SPAN)
                {
                    span *= 2;
                    if (span > SPECTRUM_MAX_SPAN) span = SPECTRUM_MAX_SPAN;
                    /* If zoomed back out to or past the preset span, snap to preset */
                    if (span >= sweep_spans[band_idx])
                        custom_view = false;
                }
            }
            else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                /* Re-center on peak and zoom in one step */
                center_freq = peak_freq_hz;
                if (span > SPECTRUM_MIN_SPAN)
                {
                    span /= 2;
                    if (span < SPECTRUM_MIN_SPAN) span = SPECTRUM_MIN_SPAN;
                }
                custom_view = true;
            }
        }
    }

    radio_set_antenna_mode(RADIO_ANTENNA_MODE_ISOLATED);
    SI446x_Change_State(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
    menu_sub_ghz_exit();
    xQueueReset(main_q_hdl);
    m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);
}


/*============================================================================*/
/*                                                                            */
/*  WEATHER STATION MONITOR                                                   */
/*  Listen for weather station transmissions and display decoded data          */
/*                                                                            */
/*============================================================================*/

void sub_ghz_weather_station(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    SubGHz_Dec_Info_t decoded_data;
    const SubGHz_Weather_Data_t *wx;
    char line1[32], line2[32], line3[32];
    bool running = true;
    bool has_data = false;

    menu_sub_ghz_init();

    /* Weather stations typically transmit on 433.92 MHz */
    radio_init_rx_tx(SUB_GHZ_BAND_433_92, MODEM_MOD_TYPE_OOK, true);
    SI446x_Select_Frontend(SUB_GHZ_BAND_433_92);
    radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);
    SI446x_Start_Rx(0);
    subghz_decenc_init();

    /* Initial display */
    u8g2_FirstPage(&m1_u8g2);
    do {
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 2, 12, "Weather Station");
        u8g2_DrawStr(&m1_u8g2, 2, 28, "Listening 433.92MHz...");
        u8g2_DrawStr(&m1_u8g2, 2, 56, "Press BACK to exit");
    } while (u8g2_NextPage(&m1_u8g2));

    while (running)
    {
        /* Check for decoded data */
        if (subghz_decenc_read(&decoded_data, false))
        {
            /* Check if it's a weather protocol */
            if (decoded_data.protocol >= OREGON_V2 &&
                decoded_data.protocol <= LACROSSE_TX)
            {
                wx = subghz_get_weather_data();
                has_data = true;

                int16_t temp_int = wx->temp_raw / 10;
                int16_t temp_frac = abs(wx->temp_raw) % 10;

                snprintf(line1, sizeof(line1), "%s  Ch:%d",
                         protocol_text[decoded_data.protocol], wx->channel);
                snprintf(line2, sizeof(line2), "Temp: %d.%dC  Hum: %d%%",
                         temp_int, temp_frac, wx->humidity);
                snprintf(line3, sizeof(line3), "ID:%04X %s %ddBm",
                         wx->id,
                         wx->battery_low ? "LOW" : "OK",
                         decoded_data.rssi);

                u8g2_FirstPage(&m1_u8g2);
                do {
                    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
                    u8g2_DrawStr(&m1_u8g2, 2, 12, "Weather Station");
                    u8g2_DrawStr(&m1_u8g2, 2, 24, line1);
                    u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
                    u8g2_DrawStr(&m1_u8g2, 2, 38, line2);
                    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
                    u8g2_DrawStr(&m1_u8g2, 2, 50, line3);
                    u8g2_DrawStr(&m1_u8g2, 2, 62, "BACK to exit");
                } while (u8g2_NextPage(&m1_u8g2));

                M1_LOG_I(M1_LOGDB_TAG, "WX: %s ch%d %d.%dC %d%% RSSI=%d\r\n",
                         protocol_text[decoded_data.protocol],
                         wx->channel, temp_int, temp_frac,
                         wx->humidity, decoded_data.rssi);
            }
        }

        ret = xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(200));
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                running = false;
            }
        }
    }

    radio_set_antenna_mode(RADIO_ANTENNA_MODE_ISOLATED);
    SI446x_Change_State(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
    menu_sub_ghz_exit();
    xQueueReset(main_q_hdl);
    m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);
}


/*============================================================================*/
/*                                                                            */
/*  BRUTE FORCE for fixed-code protocols                                      */
/*  Iterates through all possible codes for Princeton, CAME, Nice FLO,        */
/*  Linear, Holtek. Interruptible with BACK button.                           */
/*                                                                            */
/*============================================================================*/

/* Encoder: generates OOK pulse timing from a code value */
static void brute_force_encode_pwm(uint32_t code, uint8_t bits,
                                   uint16_t te_short, uint16_t te_long,
                                   uint16_t *pulse_buf, uint16_t *pulse_count)
{
    uint16_t idx = 0;
    int8_t b;

    for (b = bits - 1; b >= 0; b--)
    {
        if ((code >> b) & 1)
        {
            /* bit 1: long-high, short-low */
            pulse_buf[idx++] = te_long | SUBGHZ_OTA_PULSE_BIT_MASK;   /* mark */
            pulse_buf[idx++] = te_short & SUBGHZ_OTA_SPACE_BIT_MASK;  /* space */
        }
        else
        {
            /* bit 0: short-high, long-low */
            pulse_buf[idx++] = te_short | SUBGHZ_OTA_PULSE_BIT_MASK;
            pulse_buf[idx++] = te_long & SUBGHZ_OTA_SPACE_BIT_MASK;
        }
    }
    /* Inter-packet gap */
    pulse_buf[idx++] = (te_short * 30) & SUBGHZ_OTA_SPACE_BIT_MASK;
    *pulse_count = idx;
}

void sub_ghz_brute_force(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    char line1[32], line2[32], line3[32];
    bool running = true;

    /* Protocol selection — defaults */
    uint8_t proto_idx = 0;
    static const uint8_t brute_protos[] = { PRINCETON, CAME_12BIT, NICE_FLO, LINEAR_10BIT, HOLTEK_HT12E };
    static const char *brute_names[] = { "Princeton", "CAME", "Nice FLO", "Linear", "Holtek" };
    static const uint8_t brute_bits[] = { 24, 12, 12, 10, 12 };
    #define NUM_BRUTE_PROTOS 5

    uint32_t code = 0;
    uint32_t max_code;
    uint16_t pulse_buf[256];
    uint16_t pulse_count;
    uint8_t  state = 0;  /* 0=select protocol, 1=running, 2=done */
    uint16_t te_short, te_long;

    menu_sub_ghz_init();

    /* Select protocol screen */
    while (running && state == 0)
    {
        u8g2_FirstPage(&m1_u8g2);
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 12, "Brute Force");
            snprintf(line1, sizeof(line1), "> %s (%d bit)",
                     brute_names[proto_idx], brute_bits[proto_idx]);
            u8g2_DrawStr(&m1_u8g2, 2, 28, line1);
            max_code = (1UL << brute_bits[proto_idx]) - 1;
            snprintf(line2, sizeof(line2), "Codes: 0-%lu", max_code);
            u8g2_DrawStr(&m1_u8g2, 2, 40, line2);
            u8g2_DrawStr(&m1_u8g2, 2, 52, "UP/DN:Proto OK:Start");
            u8g2_DrawStr(&m1_u8g2, 2, 62, "BACK to exit");
        } while (u8g2_NextPage(&m1_u8g2));

        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                running = false;
            }
            else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
            {
                proto_idx = (proto_idx + NUM_BRUTE_PROTOS - 1) % NUM_BRUTE_PROTOS;
            }
            else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
            {
                proto_idx = (proto_idx + 1) % NUM_BRUTE_PROTOS;
            }
            else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                state = 1;
                code = 0;
                max_code = (1UL << brute_bits[proto_idx]) - 1;
                te_short = subghz_protocols_list[brute_protos[proto_idx]].te_short;
                te_long  = subghz_protocols_list[brute_protos[proto_idx]].te_long;
            }
        }
    }

    if (state == 1)
    {
        /* Init radio for TX on 433.92 MHz */
        radio_init_rx_tx(SUB_GHZ_BAND_433_92, MODEM_MOD_TYPE_OOK, true);
        SI446x_Select_Frontend(SUB_GHZ_BAND_433_92);
        radio_set_antenna_mode(RADIO_ANTENNA_MODE_TX);
    }

    /* Brute force loop */
    while (running && state == 1)
    {
        /* Encode and transmit current code */
        brute_force_encode_pwm(code, brute_bits[proto_idx],
                              te_short, te_long,
                              pulse_buf, &pulse_count);

        /* Load into ring buffer for TX (reuse existing TX path) */
        m1_ringbuffer_reset(&subghz_rx_rawdata_rb);
        for (uint16_t i = 0; i < pulse_count; i++)
        {
            uint32_t val32 = pulse_buf[i];
            m1_ringbuffer_insert(&subghz_rx_rawdata_rb, (uint8_t *)&val32);
        }

        /* Update display every 64 codes */
        if ((code & 0x3F) == 0)
        {
            uint32_t pct = (code * 100) / (max_code + 1);
            u8g2_FirstPage(&m1_u8g2);
            do {
                u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
                u8g2_DrawStr(&m1_u8g2, 2, 12, "Brute Force");
                snprintf(line1, sizeof(line1), "%s %d-bit", brute_names[proto_idx], brute_bits[proto_idx]);
                u8g2_DrawStr(&m1_u8g2, 2, 24, line1);
                snprintf(line2, sizeof(line2), "Code: 0x%lX", code);
                u8g2_DrawStr(&m1_u8g2, 2, 38, line2);
                snprintf(line3, sizeof(line3), "Progress: %lu%%", pct);
                u8g2_DrawStr(&m1_u8g2, 2, 50, line3);
                /* Progress bar */
                u8g2_DrawFrame(&m1_u8g2, 2, 54, 124, 8);
                u8g2_DrawBox(&m1_u8g2, 3, 55, (uint16_t)(pct * 122 / 100), 6);
            } while (u8g2_NextPage(&m1_u8g2));

            /* Check for BACK button (non-blocking) */
            ret = xQueueReceive(main_q_hdl, &q_item, 0);
            if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
            {
                ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
                if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
                {
                    running = false;
                }
            }
        }

        /* Transmit delay — allow signal to be sent */
        HAL_Delay(2);

        code++;
        if (code > max_code)
        {
            state = 2; /* Done */
        }
    }

    if (state == 2)
    {
        u8g2_FirstPage(&m1_u8g2);
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 12, "Brute Force");
            u8g2_DrawStr(&m1_u8g2, 2, 28, "Complete!");
            snprintf(line1, sizeof(line1), "%lu codes sent", max_code + 1);
            u8g2_DrawStr(&m1_u8g2, 2, 44, line1);
            u8g2_DrawStr(&m1_u8g2, 2, 62, "BACK to exit");
        } while (u8g2_NextPage(&m1_u8g2));

        /* Wait for BACK */
        while (1)
        {
            ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
            if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
            {
                ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
                if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
                    break;
            }
        }
    }

    radio_set_antenna_mode(RADIO_ANTENNA_MODE_ISOLATED);
    SI446x_Change_State(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
    menu_sub_ghz_exit();
    xQueueReset(main_q_hdl);
    m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);
}  /* end sub_ghz_brute_force */


/*============================================================================*/
/*                                                                            */
/*  RSSI SIGNAL STRENGTH METER                                                */
/*  Real-time RSSI display on a single frequency with bar graph               */
/*                                                                            */
/*============================================================================*/

void sub_ghz_rssi_meter(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    struct si446x_reply_GET_MODEM_STATUS_map *pmodemstat;
    char info_str[32];
    bool running = true;
    int16_t rssi, peak_rssi = -127;

    menu_sub_ghz_init();

    sub_ghz_set_opmode(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);

    while (running)
    {
        /* Read RSSI */
        SI446x_Get_IntStatus(0, 0, 0);
        pmodemstat = SI446x_Get_ModemStatus(0x00);
        rssi = pmodemstat->CURR_RSSI / 2 - MODEM_RSSI_COMP - 70;

        if (rssi > peak_rssi) peak_rssi = rssi;

        /* Compute bar width (0-120 px, mapped from -120 to -30 dBm) */
        int16_t clamped = rssi;
        if (clamped < -120) clamped = -120;
        if (clamped > -30) clamped = -30;
        uint8_t bar_w = (uint8_t)(((clamped + 120) * 120) / 90);

        int16_t peak_clamped = peak_rssi;
        if (peak_clamped < -120) peak_clamped = -120;
        if (peak_clamped > -30) peak_clamped = -30;
        uint8_t peak_x = (uint8_t)(4 + ((peak_clamped + 120) * 120) / 90);

        /* Draw */
        u8g2_FirstPage(&m1_u8g2);
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

            /* Title with frequency */
            if (subghz_scan_config.band == SUB_GHZ_BAND_CUSTOM)
                snprintf(info_str, sizeof(info_str), "RSSI: %lu.%03lu MHz",
                         subghz_custom_freq_hz / 1000000UL,
                         (subghz_custom_freq_hz % 1000000UL) / 1000UL);
            else if (subghz_scan_config.band < SUB_GHZ_BAND_EOL)
                snprintf(info_str, sizeof(info_str), "RSSI: %s MHz",
                         subghz_band_text[subghz_scan_config.band]);
            else
                snprintf(info_str, sizeof(info_str), "RSSI Meter");
            u8g2_DrawStr(&m1_u8g2, 0, 9, info_str);

            /* Current and peak dBm */
            snprintf(info_str, sizeof(info_str), "%ddBm  Pk:%ddBm", rssi, peak_rssi);
            u8g2_DrawStr(&m1_u8g2, 0, 22, info_str);

            /* Bar graph background */
            u8g2_DrawFrame(&m1_u8g2, 3, 26, 122, 14);

            /* Current RSSI bar */
            if (bar_w > 0)
                u8g2_DrawBox(&m1_u8g2, 4, 27, bar_w, 12);

            /* Peak marker line */
            if (peak_x >= 4 && peak_x <= 124)
                u8g2_DrawVLine(&m1_u8g2, peak_x, 25, 16);

            /* Scale labels */
            u8g2_DrawStr(&m1_u8g2, 0, 52, "-120");
            u8g2_DrawStr(&m1_u8g2, 105, 52, "-30");

            /* Controls */
            u8g2_DrawStr(&m1_u8g2, 0, 64, "L/R:Band OK:Reset \x18:Freq");

        } while (u8g2_NextPage(&m1_u8g2));

        /* Check for button input */
        ret = xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(50));
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &this_button_status, 0);

            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                running = false;
            }
            else if (this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                uint8_t idx = subghz_band_order_find(subghz_scan_config.band);
                idx++;
                if (idx >= SUBGHZ_BAND_ORDER_COUNT) idx = 0;
                subghz_scan_config.band = subghz_band_order[idx];
                peak_rssi = -127;
                sub_ghz_set_opmode(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
            }
            else if (this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                uint8_t idx = subghz_band_order_find(subghz_scan_config.band);
                if (idx > 0) idx--; else idx = SUBGHZ_BAND_ORDER_COUNT - 1;
                subghz_scan_config.band = subghz_band_order[idx];
                peak_rssi = -127;
                sub_ghz_set_opmode(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
            }
            else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                peak_rssi = -127; /* Reset peak */
            }
            else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
            {
                /* Custom frequency entry */
                sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, subghz_scan_config.band, 0, 0);
                if (sub_ghz_custom_freq_entry())
                {
                    subghz_scan_config.band = SUB_GHZ_BAND_CUSTOM;
                }
                peak_rssi = -127;
                sub_ghz_set_opmode(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
            }
        }
    }

    sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, subghz_scan_config.band, 0, 0);
    menu_sub_ghz_exit();
    xQueueReset(main_q_hdl);
    m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);
}


/*============================================================================*/
/*                                                                            */
/*  FREQUENCY SCANNER / ACTIVITY MONITOR                                      */
/*  Sweep a frequency range and show active signals above RSSI threshold      */
/*                                                                            */
/*============================================================================*/

#define FREQ_SCANNER_MAX_HITS     16
#define FREQ_SCANNER_THRESHOLD    -80   /* dBm threshold for "active" */
#define FREQ_SCANNER_DEDUP_KHZ    50    /* Merge hits within 50 kHz */
#define FREQ_SCANNER_VISIBLE_ROWS  5

typedef struct {
    uint32_t freq_hz;
    int8_t   rssi;
    uint8_t  hit_count;
} freq_scanner_hit_t;

void sub_ghz_freq_scanner(void)
{
    S_M1_Buttons_Status this_button_status;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;
    struct si446x_reply_GET_MODEM_STATUS_map *pmodemstat;
    char info_str[40];

    freq_scanner_hit_t hits[FREQ_SCANNER_MAX_HITS];
    uint8_t  hit_count = 0;
    uint8_t  scroll_pos = 0;
    bool     running = true;
    uint8_t  range_idx = 0;
    int8_t   threshold = FREQ_SCANNER_THRESHOLD;

    static const uint32_t scan_centers[] = {
        307000000UL,
        370000000UL,
        435000000UL,
        915000000UL,
    };
    static const uint32_t scan_spans[] = {
        15000000UL,
        50000000UL,
        10000000UL,
        10000000UL,
    };
    static const char *scan_labels[] = {
        "300-315",
        "345-395",
        "430-440",
        "910-920",
    };
    #define NUM_SCAN_RANGES 4

    memset(hits, 0, sizeof(hits));

    menu_sub_ghz_init();

    /* Initialize radio */
    if (scan_centers[range_idx] < 525000000UL && scan_centers[range_idx] >= 284000000UL)
        radio_init_rx_tx(SUB_GHZ_BAND_433, MODEM_MOD_TYPE_OOK, true);
    else
        radio_init_rx_tx(SUB_GHZ_BAND_915, MODEM_MOD_TYPE_OOK, true);
    radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);

    while (running)
    {
        /* Perform one sweep */
        uint32_t center = scan_centers[range_idx];
        uint32_t span = scan_spans[range_idx];
        uint32_t step = span / 128;
        if (step < 50000) step = 50000; /* Min 50 kHz steps */
        uint32_t freq = center - span / 2;
        uint32_t freq_end = center + span / 2;

        while (freq < freq_end)
        {
            SI446x_Set_Frequency(freq);
            SI446x_Start_Rx(0);
            HAL_Delay(2); /* Let AGC settle */

            SI446x_Get_IntStatus(0, 0, 0);
            pmodemstat = SI446x_Get_ModemStatus(0x00);
            int16_t rssi = pmodemstat->CURR_RSSI / 2 - MODEM_RSSI_COMP - 70;

            if (rssi > threshold)
            {
                /* Check for duplicate (within DEDUP range) */
                bool found = false;
                for (uint8_t h = 0; h < hit_count; h++)
                {
                    int32_t diff = (int32_t)freq - (int32_t)hits[h].freq_hz;
                    if (diff < 0) diff = -diff;
                    if (diff < (int32_t)(FREQ_SCANNER_DEDUP_KHZ * 1000))
                    {
                        if ((int8_t)rssi > hits[h].rssi)
                        {
                            hits[h].rssi = (int8_t)rssi;
                            hits[h].freq_hz = freq;
                        }
                        if (hits[h].hit_count < 255) hits[h].hit_count++;
                        found = true;
                        break;
                    }
                }

                if (!found && hit_count < FREQ_SCANNER_MAX_HITS)
                {
                    hits[hit_count].freq_hz = freq;
                    hits[hit_count].rssi = (int8_t)rssi;
                    hits[hit_count].hit_count = 1;
                    hit_count++;
                }
            }

            freq += step;
        }

        /* Draw results */
        u8g2_FirstPage(&m1_u8g2);
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

            /* Title */
            snprintf(info_str, sizeof(info_str), "Scan: %s MHz [%ddBm]",
                     scan_labels[range_idx], threshold);
            u8g2_DrawStr(&m1_u8g2, 0, 9, info_str);

            if (hit_count == 0)
            {
                u8g2_DrawStr(&m1_u8g2, 10, 32, "Scanning...");
            }
            else
            {
                /* Draw hit list */
                for (uint8_t row = 0; row < FREQ_SCANNER_VISIBLE_ROWS; row++)
                {
                    uint8_t idx = scroll_pos + row;
                    if (idx >= hit_count) break;

                    uint8_t y = 20 + row * 9;
                    snprintf(info_str, sizeof(info_str), "%lu.%03lu %ddBm x%d",
                             hits[idx].freq_hz / 1000000UL,
                             (hits[idx].freq_hz % 1000000UL) / 1000UL,
                             hits[idx].rssi,
                             hits[idx].hit_count);
                    u8g2_DrawStr(&m1_u8g2, 2, y, info_str);
                }

                /* Scroll indicator */
                if (hit_count > FREQ_SCANNER_VISIBLE_ROWS)
                {
                    snprintf(info_str, sizeof(info_str), "%d/%d",
                             scroll_pos + 1, hit_count);
                    u8g2_DrawStr(&m1_u8g2, 100, 64, info_str);
                }
            }

            /* Bottom controls */
            u8g2_DrawStr(&m1_u8g2, 0, 64, "L/R:Band OK:Clr \x18\x19:Scrl");

        } while (u8g2_NextPage(&m1_u8g2));

        /* Check for button input */
        ret = xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(100));
        if (ret == pdTRUE && q_item.q_evt_type == Q_EVENT_KEYPAD)
        {
            xQueueReceive(button_events_q_hdl, &this_button_status, 0);

            if (this_button_status.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                running = false;
            }
            else if (this_button_status.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                range_idx = (range_idx + 1) % NUM_SCAN_RANGES;
                hit_count = 0;
                scroll_pos = 0;
                memset(hits, 0, sizeof(hits));
                if (scan_centers[range_idx] >= 850000000UL)
                    radio_init_rx_tx(SUB_GHZ_BAND_915, MODEM_MOD_TYPE_OOK, true);
                else
                    radio_init_rx_tx(SUB_GHZ_BAND_433, MODEM_MOD_TYPE_OOK, true);
                radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);
            }
            else if (this_button_status.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
            {
                range_idx = (range_idx + NUM_SCAN_RANGES - 1) % NUM_SCAN_RANGES;
                hit_count = 0;
                scroll_pos = 0;
                memset(hits, 0, sizeof(hits));
                if (scan_centers[range_idx] >= 850000000UL)
                    radio_init_rx_tx(SUB_GHZ_BAND_915, MODEM_MOD_TYPE_OOK, true);
                else
                    radio_init_rx_tx(SUB_GHZ_BAND_433, MODEM_MOD_TYPE_OOK, true);
                radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);
            }
            else if (this_button_status.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
            {
                if (scroll_pos > 0) scroll_pos--;
            }
            else if (this_button_status.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
            {
                if (scroll_pos + FREQ_SCANNER_VISIBLE_ROWS < hit_count) scroll_pos++;
            }
            else if (this_button_status.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
            {
                /* Clear hits and rescan */
                hit_count = 0;
                scroll_pos = 0;
                memset(hits, 0, sizeof(hits));
            }
        }
    }

    radio_set_antenna_mode(RADIO_ANTENNA_MODE_ISOLATED);
    SI446x_Change_State(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
    menu_sub_ghz_exit();
    xQueueReset(main_q_hdl);
    m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);
}
