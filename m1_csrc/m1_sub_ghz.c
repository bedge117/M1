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
#include <ctype.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_tasks.h"
#include "m1_sub_ghz_api.h"
//#include "m1_sub_ghz.h"
#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"
#include "subghz_key_encoder.h"
#include "subghz_raw_line_parser.h"
#include "subghz_raw_capture_alloc.h"
#include "subghz_secplus_v1_encoder.h"
#include "subghz_keeloq_encoder.h"
#include "subghz_keeloq_mfkeys.h"
#include "subghz_button_override.h"
#include "subghz_rssi_history.h"
#include "m1_ring_buffer.h"
#include "m1_core_config.h"
#include "m1_storage.h"
#include "m1_sdcard_man.h"
#include "flipper_subghz.h"
#include "m1_settings.h"
#include "m1_virtual_kb.h"
#include "m1_scene.h"
#include "uiView.h"

/*************************** D E F I N E S ************************************/

#define SUBGHZ_DMA_ALIGN			32 /* D-Cache line size on Cortex-M33 */

#define SUBGHZ_RAW_DATA_SAMPLES_MAX			64000 // data type of sample: uint16_t

#define SUBGHZ_TX_RAW_REPLAY_REPEAT_DEFAULT		4 // number of additional replays after the first transmit

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
#define SUBGHZ_ISM_BANDS_LIST_ASIA			3

#define	SUBGHZ_FCC_BASE_FREQ_300_000			(float)300.00001
#define	SUBGHZ_FCC_BASE_FREQ_310_000			(float)310.00001
#define	SUBGHZ_FCC_BASE_FREQ_315_000			(float)315.00001
#define	SUBGHZ_FCC_BASE_FREQ_345_000			(float)345.00001
#define	SUBGHZ_FCC_BASE_FREQ_372_000			(float)372.00001
#define	SUBGHZ_FCC_BASE_FREQ_390_000			(float)390.00001
#define	SUBGHZ_FCC_BASE_FREQ_433_000			(float)433.00001
#define	SUBGHZ_FCC_BASE_FREQ_433_920			(float)433.92001
#define SUBGHZ_FCC_BASE_FREQ_915_000			(float)915.00001

// Reference: FCC 15.205 Restricted bands of operation
// 322MHz-335.4MHz, 399.9MHz-410MHz
//#define SUBGHZ_FCC_ISM_BAND_304_100				(float)304.10001 // FZ
#define SUBGHZ_FCC_ISM_BAND_300_000				(float)300.00001
#define SUBGHZ_FCC_ISM_BAND_310_000				(float)310.00001
#define SUBGHZ_FCC_ISM_BAND_321_950				(float)321.95001

#define SUBGHZ_FCC_ISM_BAND_344_000				(float)344.00001
#define SUBGHZ_FCC_ISM_BAND_392_000				(float)392.00001

#define SUBGHZ_FCC_ISM_BAND_322_000				(float)322.00001
#define SUBGHZ_FCC_ISM_BAND_348_000				(float)348.00001

#define SUBGHZ_FCC_ISM_BAND_433_050				(float)433.05001
#define SUBGHZ_FCC_ISM_BAND_434_790				(float)434.79001

#define SUBGHZ_FCC_ISM_BAND_915_000				(float)915.00001
#define SUBGHZ_FCC_ISM_BAND_928_000				(float)928.00001

//************************** C O N S T A N T **********************************/

const char *subghz_ism_regions_text[SUBGHZ_ISM_BAND_REGIONS_LIST] =
{
	"North America",
	"Europe",
	"Asia",
	"Off"
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
	"915.000"
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
	{SUBGHZ_FCC_BASE_FREQ_315_000, 80},	// 315.000 - 335.000, step = 250KHz //80
	{SUBGHZ_FCC_BASE_FREQ_345_000, 4},	// 345.000 - 346.000 // 4
	{SUBGHZ_FCC_BASE_FREQ_372_000, 4},	// 372.000 - 373.000 // 4
	{SUBGHZ_FCC_BASE_FREQ_390_000, 4},	// 390.000 - 391.000 // 4
	{SUBGHZ_FCC_BASE_FREQ_433_000, 3},	// 433.000 - 435.000 // 8
	{SUBGHZ_FCC_BASE_FREQ_433_920, 2}, 	// 433.920 - not used // 0
	{SUBGHZ_FCC_BASE_FREQ_915_000, 4} 	// 915.000 - 916.000 // 4
};

static const float subghz_fcc_ism_bands_NA[SUBGHZ_ISM_BANDS_LIST_NA][2] =
{
	{SUBGHZ_FCC_ISM_BAND_300_000, SUBGHZ_FCC_ISM_BAND_321_950}, 	// 300.00MHz - 321.95MHz
	/*{SUBGHZ_FCC_ISM_BAND_344_000, SUBGHZ_FCC_ISM_BAND_392_000},		// 344.00MHz - 392.00MHz*/
	{SUBGHZ_FCC_ISM_BAND_433_050, SUBGHZ_FCC_ISM_BAND_434_790}, 	// 433.05MHz - 434.79MHz
	{SUBGHZ_FCC_ISM_BAND_915_000, SUBGHZ_FCC_ISM_BAND_928_000}		// 915.00MHz - 928.00MHz
};

static const float subghz_fcc_ism_bands_EU[SUBGHZ_ISM_BANDS_LIST_EU][2] =
{
	{SUBGHZ_FCC_ISM_BAND_300_000, SUBGHZ_FCC_ISM_BAND_321_950}, 	// 300.00MHz - 321.95MHz
	/*{SUBGHZ_FCC_ISM_BAND_344_000, SUBGHZ_FCC_ISM_BAND_392_000},		// 344.00MHz - 392.00MHz*/
	{SUBGHZ_FCC_ISM_BAND_433_050, SUBGHZ_FCC_ISM_BAND_434_790}, 	// 433.05MHz - 434.79MHz
	{SUBGHZ_FCC_ISM_BAND_915_000, SUBGHZ_FCC_ISM_BAND_928_000}		// 915.00MHz - 928.00MHz
};

static const float subghz_fcc_ism_bands_ASIA[SUBGHZ_ISM_BANDS_LIST_ASIA][2] =
{
	{SUBGHZ_FCC_ISM_BAND_300_000, SUBGHZ_FCC_ISM_BAND_321_950}, 	// 300.00MHz - 321.95MHz
	{SUBGHZ_FCC_ISM_BAND_322_000, SUBGHZ_FCC_ISM_BAND_348_000}, 	// 322.00MHz - 348.00MHz (APAC: 330/345 MHz gate remotes)
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
	{	.this_region = NULL,
		.bands_list = 0
	},
};

/* Frequency-sorted band order for UI navigation (L/R buttons) */
#define SUBGHZ_BAND_ORDER_COUNT		9
static const S_M1_SubGHz_Band subghz_band_order[SUBGHZ_BAND_ORDER_COUNT] = {
	SUB_GHZ_BAND_300, SUB_GHZ_BAND_310, SUB_GHZ_BAND_315,
	SUB_GHZ_BAND_345, SUB_GHZ_BAND_372, SUB_GHZ_BAND_390,
	SUB_GHZ_BAND_433, SUB_GHZ_BAND_433_92, SUB_GHZ_BAND_915
};

/* Frequency preset table, dimensions, and default index are defined in
 * subghz_freq_presets.h / subghz_freq_presets.c (hardware-independent
 * pure data, directly tested by tests/test_subghz_freq_presets.c).
 * Per-region hopper tables and subghz_get_hopper_freqs() live there too. */
#include "subghz_freq_presets.h"

/* Flipper-style modulation presets */
#define SUBGHZ_MOD_PRESET_COUNT     4
static const struct {
	const char *label;
	S_M1_SubGHz_Modulation mod;
} subghz_mod_presets[SUBGHZ_MOD_PRESET_COUNT] = {
	{ "AM270", MODULATION_OOK },
	{ "AM650", MODULATION_OOK },
	{ "FM238", MODULATION_FSK },
	{ "FM476", MODULATION_FSK }
};

/* Sub-GHz config state (shared between Read / Read RAW / Config screen) */
typedef struct {
	uint8_t freq_idx;       /* Index into subghz_freq_presets[] */
	uint8_t mod_idx;        /* Index into subghz_mod_presets[] */
	bool    hopping;
	bool    bin_raw;
	bool    sound;
	bool    autosave;       /* Auto-save decoded signals to SD */
	uint8_t save_fmt;       /* 0 = Flipper .sub, 1 = M1 native .sgh */
	int8_t  rssi_threshold; /* Hopper RSSI threshold (dBm, -50 to -100) */
	bool    remove_duplicates;  /* Phase 12: receiver history — dedupe consecutive identical decodes */
	bool    delete_old_signals; /* Phase 12: receiver history — evict oldest when ring is full */
} SubGHz_Config_t;

static SubGHz_Config_t subghz_cfg = {
	.freq_idx       = SUBGHZ_FREQ_DEFAULT_IDX,
	.mod_idx        = 1,    /* AM650 — Flipper default */
	.hopping        = false,
	.bin_raw        = false,
	.sound          = true,
	.autosave       = false,
	.save_fmt       = 0,    /* Default: Flipper .sub for Flipper compatibility */
	.rssi_threshold = -70,  /* dBm: stay on freq if RSSI >= this (Flipper default) */
	.remove_duplicates  = true,  /* Default ON — Flipper-parity behaviour */
	.delete_old_signals = true   /* Default ON — Flipper-parity behaviour */
};

/* Add Manually retired in Phase 8b-4 — see m1_subghz_scene_set_type.c /
 * m1_subghz_scene_set_key.c and Sub_Ghz/subghz_create_proto.c for the
 * scene-native protocol picker + hex-editor flow that replaced it. */

static uint8_t subghz_band_order_find(S_M1_SubGHz_Band band)
{
	for (uint8_t i = 0; i < SUBGHZ_BAND_ORDER_COUNT; i++)
		if (subghz_band_order[i] == band) return i;
	return 0; /* default to 300 MHz position */
}

/* TX power setting (shared between Radio Settings UI and Record/Replay TX calls) */
#define TX_POWER_LEVELS          4
static const uint8_t tx_power_values[TX_POWER_LEVELS] = { 10, 40, 80, 127 };
static const char *tx_power_labels[TX_POWER_LEVELS] = { "Low", "Med", "High", "Max" };
static uint8_t subghz_tx_power_idx = 3;  /* Default: Max */

/* Custom frequency for SUB_GHZ_BAND_CUSTOM mode */
static uint32_t subghz_custom_freq_hz = 433920000UL;

/* User-entered custom frequency (persisted, shown in Config under "Custom" preset) */
static uint32_t subghz_user_custom_freq_hz = 433920000UL;

/* Label string for the "Custom" preset (updated when user enters a value) */
static char subghz_custom_freq_label[12] = "Custom";

/* Last decoded protocol info during Record (for overlay + .sub save) */
static SubGHz_Dec_Info_t subghz_record_last_decoded;
static bool subghz_record_has_decoded = false;

/* Signal history for Read screen */
static SubGHz_History_t subghz_signal_history;
static uint8_t subghz_history_sel = 0;        /* Currently selected history index */
static uint8_t subghz_history_scroll_top = 0; /* Top visible index in list */
static bool    subghz_history_view_active = false; /* true = showing history list */
static bool    subghz_history_detail_active = false; /* true = showing signal detail */
#define SUBGHZ_HISTORY_VISIBLE_ITEMS  5  /* Items visible on 128x64 display */
#define SUBGHZ_HISTORY_ROW_HEIGHT     6  /* Pixels per row in history list */

/* Frequency hopping state (active during Record when subghz_cfg.hopping == true) */
static uint8_t  subghz_hopper_idx  = 0;   /* Current index into subghz_hopper_freqs[] */
static uint32_t subghz_hopper_freq = 0;   /* Frequency currently tuned (Hz) */
static bool     subghz_hopper_active = false; /* true while hopping during ACTIVE record */
#define SUBGHZ_HOPPER_DWELL_MS       150  /* ms to dwell per frequency before hopping */
#define SUBGHZ_HOPPER_RSSI_THRESHOLD -70  /* dBm: stay on freq if RSSI >= this */

/* RAW RSSI history visualization (Flipper-style spectrogram).
 * The constants and push/reset logic live in subghz_rssi_history.h so they
 * can be exercised in host unit tests without pulling in hardware headers.
 * SUBGHZ_RAW_* aliases keep the rest of this file unchanged. */
#define SUBGHZ_RAW_RSSI_HISTORY_SIZE SUBGHZ_RSSI_HISTORY_SIZE
#define SUBGHZ_RAW_TOP_SCALE          14  /* Y of top border / scale ticks */
#define SUBGHZ_RAW_BOTTOM_Y           48  /* Y of bottom border */
#define SUBGHZ_RAW_END_SCALE         115  /* X of right border */
#define SUBGHZ_RAW_THRESHOLD_MIN   SUBGHZ_RSSI_THRESHOLD_MIN
#define SUBGHZ_RAW_RSSI_DIVIDER    SUBGHZ_RSSI_DIVIDER
#define SUBGHZ_RAW_SIN_AMPLITUDE      11  /* Amplitude for idle sine animation */
#define SUBGHZ_RAW_CURSOR_SEG_W        2  /* Dashed cursor segment width (px) */
#define SUBGHZ_RAW_RSSI_LABEL_Y       40  /* Y for vertical "RSSI" label */
static SubghzRssiHistory s_rssi_history;
/* Convenience aliases so the rest of this file compiles without renaming. */
#define subghz_raw_rssi_history      s_rssi_history.buf
#define subghz_raw_rssi_current      s_rssi_history.current
#define subghz_raw_rssi_head         s_rssi_history.head
#define subghz_raw_rssi_history_end  s_rssi_history.end
static uint8_t  subghz_raw_sin_idx = 0;      /* Sine animation index (idle state) */

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

/***************************** V A R I A B L E S ******************************/

TIM_HandleTypeDef   timerhdl_subghz_tx;
TIM_HandleTypeDef   timerhdl_subghz_rx;
EXTI_HandleTypeDef 	si4463_exti_hdl;
DMA_HandleTypeDef hdma_subghz_tx;

S_M1_RingBuffer subghz_rx_rawdata_rb;
static uint16_t *subghz_front_buffer = NULL;
static void     *subghz_front_buffer_base = NULL; // Original malloc pointer (before alignment)
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
static void     *subghz_back_buffer_base = NULL; // Original malloc pointer (before alignment)
static uint16_t *double_buffer_ptr[2];
static uint32_t sdcard_dat_file_size, sdcard_dat_buffer_end_pos;
uint8_t subghz_tx_tc_flag;
static uint8_t subghz_tx_start_high = 1; // 1 = next buffer starts HIGH (mark), 0 = LOW (space)
uint8_t subghz_record_mode_flag = 0;
static uint32_t subghz_record_total_samples = 0;  /* Total samples saved to file during recording */
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

/* Map Hz frequency to stock band when possible — stock bands have
 * known-working radio configs for direct mode TX. */
static S_M1_SubGHz_Band subghz_freq_hz_to_band(uint32_t freq_hz)
{
	switch (freq_hz) {
		case 300000000: return SUB_GHZ_BAND_300;
		case 310000000: return SUB_GHZ_BAND_310;
		case 315000000: return SUB_GHZ_BAND_315;
		case 345000000: return SUB_GHZ_BAND_345;
		case 372000000: return SUB_GHZ_BAND_372;
		case 390000000: return SUB_GHZ_BAND_390;
		case 433000000: return SUB_GHZ_BAND_433;
		case 433920000: return SUB_GHZ_BAND_433_92;
		case 915000000: return SUB_GHZ_BAND_915;
		default:        return SUB_GHZ_BAND_CUSTOM;
	}
}

/* Helper: set subghz_custom_freq_hz + scan_config from config preset */
static void subghz_apply_config(void)
{
	if (subghz_cfg.freq_idx == SUBGHZ_FREQ_PRESET_CUSTOM)
	{
		/* Use user-entered custom frequency */
		subghz_custom_freq_hz = subghz_user_custom_freq_hz;
	}
	else
	{
		subghz_custom_freq_hz = subghz_freq_presets[subghz_cfg.freq_idx].freq_hz;
	}
	subghz_scan_config.band = subghz_freq_hz_to_band(subghz_custom_freq_hz);
	subghz_scan_config.modulation = subghz_mod_presets[subghz_cfg.mod_idx].mod;
}

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

void menu_sub_ghz_init(void);
void menu_sub_ghz_exit(void);

void sub_ghz_init(void);
void sub_ghz_frequency_reader(void);

static void sub_ghz_rx_init(void);
static void sub_ghz_rx_start(void);
static void sub_ghz_rx_pause(void);
static void sub_ghz_rx_deinit(void);
static uint8_t sub_ghz_ring_buffers_init(void);
static void sub_ghz_ring_buffers_deinit(void);
static uint8_t sub_ghz_rx_raw_save(bool header_init, bool last_data);
static void sub_ghz_tx_raw_init(void);
static void sub_ghz_tx_raw_deinit_impl(bool reset_main_q);
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

/* RAW RSSI history & static TX helpers (defined further below, used in display/UI) */
static void subghz_raw_rssi_push(float rssi_dbm, bool trace);
static void subghz_raw_rssi_draw(void);
static void subghz_raw_rssi_draw_scale(void);
static void subghz_raw_draw_frame(void);
static void subghz_raw_draw_sin(void);
static bool subghz_protocol_is_static(uint16_t protocol);

/* Flipper-matching feature functions */
/* (sub_ghz_add_manually retired in Phase 8b-4 — see SubGhzSceneSetType) */

/* Frequency hopping helpers (defined after forward declarations) */
static uint32_t subghz_hopper_retune_next(void);
static int16_t subghz_read_rssi(void);

/* Save format accessor (defined near end of file with other config accessors) */
uint8_t subghz_get_save_fmt_ext(void);

//************************** C O N S T A N T **********************************/


/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/* --- Frequency hopping helpers --- */

static uint32_t subghz_hopper_retune_next(void)
{
	subghz_hopper_idx = (subghz_hopper_idx + 1) % SUBGHZ_HOPPER_FREQ_COUNT;
	uint32_t freq = subghz_get_hopper_freqs(m1_device_stat.config.ism_band_region)[subghz_hopper_idx];

	sub_ghz_rx_pause();

	subghz_custom_freq_hz = freq;
	subghz_scan_config.band = subghz_freq_hz_to_band(freq);

	/* Retune via the full opmode path (handles band switch + frontend select) */
	sub_ghz_set_opmode(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
	SI446x_Change_Modem_OOK_PDTC(SUB_GHZ_433_92_NEW_PDTC);

	sub_ghz_rx_start();

	subghz_hopper_freq = freq;
	return freq;
}

static int16_t subghz_read_rssi(void)
{
	struct si446x_reply_GET_MODEM_STATUS_map *pstat;
	pstat = SI446x_Get_ModemStatus(0x00);
	return (int16_t)(pstat->CURR_RSSI / 2) - MODEM_RSSI_COMP - 70;
}

/* --- End hopping helpers --- */

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

		case SUB_GHZ_BAND_CUSTOM:
			/* Use base configs that have GPIO2=INPUT (0x04) for direct TX.
			 * For FSK at ANY frequency, load the 915 FSK config and retune —
			 * the 915 config is the only one with 2FSK modem settings.
			 * For OOK: BAND_315 for <420MHz, BAND_433_92 for 420-849MHz,
			 * BAND_915 for 850+MHz. */
			if (subghz_scan_config.modulation == MODULATION_FSK)
			{
				init_freq = SUB_GHZ_BAND_915;
				mod_type = MODEM_MOD_TYPE_FSK;
			}
			else if (subghz_custom_freq_hz >= 850000000UL)
			{
				init_freq = SUB_GHZ_BAND_915;
				mod_type = MODEM_MOD_TYPE_OOK;
			}
			else if (subghz_custom_freq_hz >= 420000000UL)
			{
				init_freq = SUB_GHZ_BAND_433_92;
				mod_type = MODEM_MOD_TYPE_OOK;
			}
			else
			{
				init_freq = SUB_GHZ_BAND_315;
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

	/* For CUSTOM band, select the antenna frontend based on the actual
	 * target frequency — init_freq may differ (e.g. 915 FSK config loaded
	 * for a 433 MHz 2FSK signal). */
	if (band == SUB_GHZ_BAND_CUSTOM)
	{
		S_M1_SubGHz_Band fe_band;
		if (subghz_custom_freq_hz >= 850000000UL)
			fe_band = SUB_GHZ_BAND_915;
		else if (subghz_custom_freq_hz >= 390000000UL)
			fe_band = SUB_GHZ_BAND_433_92;
		else
			fe_band = SUB_GHZ_BAND_315;
		SI446x_Select_Frontend(fe_band);
	}
	else
		SI446x_Select_Frontend(band);

	if (retune_freq_hz)
	{
		/* Ensure READY state before changing frequency.
		 * VCO recalibration is automatic on READY→TX/RX transition. */
		SI446x_Change_State(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);
		SI446x_Set_Frequency(retune_freq_hz);
	}

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
			/* Ensure GPIO2 is INPUT for direct mode TX data input.
			 * Some radio configs set GPIO2 to EN_LNA (0x13) which blocks TX. */
			SI446x_GPIO2_Set_Input();
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

		m1_u8g2_firstpage();
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
		} while (m1_u8g2_nextpage());

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

				if (new_freq >= SUBGHZ_MIN_FREQ_HZ && new_freq <= SUBGHZ_MAX_FREQ_HZ)
				{
					subghz_custom_freq_hz = new_freq;
					accepted = true;
					done = true;
				}
				else
				{
					/* Out of range — flash error */
					m1_message_box(&m1_u8g2, "Out of range!", "300.000 - 928.000 MHz", "", "BACK to retry");
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
  * @brief  Sine lookup table for idle animation (Flipper-style Lissajous).
  *         Returns values in [-127, 127] for the given angle byte.
  */
/*============================================================================*/
static int8_t subghz_raw_tab_sin(uint8_t x)
{
	static const uint8_t tab[64] = {
		0,   3,   6,   9,   12,  16,  19,  22,  25,  28,  31,  34,  37,
		40,  43,  46,  49,  51,  54,  57,  60,  63,  65,  68,  71,  73,
		76,  78,  81,  83,  85,  88,  90,  92,  94,  96,  98,  100, 102,
		104, 106, 107, 109, 111, 112, 113, 115, 116, 117, 118, 120, 121,
		122, 122, 123, 124, 125, 125, 126, 126, 126, 127, 127, 127
	};
	int8_t r = (int8_t)tab[((x & 0x40) ? (uint8_t)(~x) : x) & 0x3f];
	if (x & 0x80) return (int8_t)(-r);
	return r;
}

/*============================================================================*/
/**
  * @brief  Draw animated sine wave (Flipper-style Lissajous) in idle/start state.
  *         Uses subghz_raw_sin_idx as animation counter (0-62).
  */
/*============================================================================*/
static void subghz_raw_draw_sin(void)
{
	uint8_t mid_y = (SUBGHZ_RAW_TOP_SCALE + SUBGHZ_RAW_BOTTOM_Y) / 2;

	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	for (int i = SUBGHZ_RAW_END_SCALE - 2; i > 0; i--)
	{
		int y1 = (int)mid_y - subghz_raw_tab_sin((uint8_t)(i + subghz_raw_sin_idx * 16)) / SUBGHZ_RAW_SIN_AMPLITUDE;
		int y2 = (int)mid_y + subghz_raw_tab_sin((uint8_t)((i + subghz_raw_sin_idx * 16 + 1) * 2)) / SUBGHZ_RAW_SIN_AMPLITUDE;

		/* Clamp to waveform area */
		if (y1 < SUBGHZ_RAW_TOP_SCALE + 1) y1 = SUBGHZ_RAW_TOP_SCALE + 1;
		if (y1 > SUBGHZ_RAW_BOTTOM_Y - 1) y1 = SUBGHZ_RAW_BOTTOM_Y - 1;
		if (y2 < SUBGHZ_RAW_TOP_SCALE + 1) y2 = SUBGHZ_RAW_TOP_SCALE + 1;
		if (y2 > SUBGHZ_RAW_BOTTOM_Y - 1) y2 = SUBGHZ_RAW_BOTTOM_Y - 1;

		u8g2_DrawLine(&m1_u8g2, i, y1, i + 1, y2);
		u8g2_DrawLine(&m1_u8g2, i + 1, y1, i + 2, y2);
	}
}

/*============================================================================*/
/**
  * @brief  Push an RSSI reading into the history buffer.
  *         Delegates to subghz_rssi_history_push() (subghz_rssi_history.h) which
  *         contains the bug-fixed logic and can be exercised by host unit tests.
  * @param  rssi_dbm  RSSI in dBm (e.g. -80.0)
  * @param  trace     true = advance write position (new data point);
  *                   false = update current position only (display refresh)
  */
/*============================================================================*/
static void subghz_raw_rssi_push(float rssi_dbm, bool trace)
{
	subghz_rssi_history_push(&s_rssi_history, rssi_dbm, trace);
}

/*============================================================================*/
/**
  * @brief  Reset the RSSI history buffer.
  */
/*============================================================================*/
static void subghz_raw_rssi_reset(void)
{
	subghz_rssi_history_reset(&s_rssi_history);
	subghz_raw_sin_idx = 0;
}

/*============================================================================*/
/**
  * @brief  Draw timeline scale ticks along the top of the waveform area.
  *         Flipper-style: major ticks every 15px, minor ticks every 5px.
  *         Ticks scroll with the data when the buffer has wrapped.
  */
/*============================================================================*/
static void subghz_raw_rssi_draw_scale(void)
{
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

	if (!subghz_raw_rssi_history_end)
	{
		/* Not wrapped: fixed ticks from right */
		for (int i = SUBGHZ_RAW_END_SCALE; i > 0; i -= 15)
		{
			u8g2_DrawVLine(&m1_u8g2, i, SUBGHZ_RAW_TOP_SCALE, 5);
			if (i - 5 > 0)
				u8g2_DrawVLine(&m1_u8g2, i - 5, SUBGHZ_RAW_TOP_SCALE, 3);
			if (i - 10 > 0)
				u8g2_DrawVLine(&m1_u8g2, i - 10, SUBGHZ_RAW_TOP_SCALE, 3);
		}
	}
	else
	{
		/* Wrapped: ticks scroll with write position */
		int offset = subghz_raw_rssi_head % 15;
		for (int i = SUBGHZ_RAW_END_SCALE - offset; i > -15; i -= 15)
		{
			u8g2_DrawVLine(&m1_u8g2, i, SUBGHZ_RAW_TOP_SCALE, 5);
			if (SUBGHZ_RAW_END_SCALE > i + 5 && i + 5 > 0)
				u8g2_DrawVLine(&m1_u8g2, i + 5, SUBGHZ_RAW_TOP_SCALE, 3);
			if (SUBGHZ_RAW_END_SCALE > i + 10 && i + 10 > 0)
				u8g2_DrawVLine(&m1_u8g2, i + 10, SUBGHZ_RAW_TOP_SCALE, 3);
		}
	}
}

/*============================================================================*/
/**
  * @brief  Draw the waveform area frame (borders + "RSSI" label).
  *         Called unconditionally on every draw cycle — Flipper draws the frame
  *         even in idle/TX/sine states.
  */
/*============================================================================*/
static void subghz_raw_draw_frame(void)
{
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

	/* Three border lines enclosing the waveform area */
	u8g2_DrawHLine(&m1_u8g2, 0, SUBGHZ_RAW_TOP_SCALE, SUBGHZ_RAW_END_SCALE + 1);
	u8g2_DrawHLine(&m1_u8g2, 0, SUBGHZ_RAW_BOTTOM_Y, SUBGHZ_RAW_END_SCALE + 1);
	u8g2_DrawVLine(&m1_u8g2, SUBGHZ_RAW_END_SCALE, SUBGHZ_RAW_TOP_SCALE,
	               SUBGHZ_RAW_BOTTOM_Y - SUBGHZ_RAW_TOP_SCALE + 1);

	/* "RSSI" label drawn vertically on right side — small font to match
	 * Momentum style and avoid overlap during recording. */
	u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
	u8g2_SetFontDirection(&m1_u8g2, 3); /* bottom-to-top */
	u8g2_DrawStr(&m1_u8g2, M1_LCD_DISPLAY_WIDTH - 1, SUBGHZ_RAW_RSSI_LABEL_Y, "RSSI");
	u8g2_SetFontDirection(&m1_u8g2, 0); /* restore left-to-right */
}

static void subghz_raw_rssi_draw(void)
{
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

	/* Timeline scale ticks */
	subghz_raw_rssi_draw_scale();

	int cursor_x;
	const uint8_t cursor_w = SUBGHZ_RAW_CURSOR_SEG_W;
	const int bottom = SUBGHZ_RAW_BOTTOM_Y - 1; /* Bottom of drawable area */

	if (!subghz_raw_rssi_history_end)
	{
		/* Not wrapped: bars left-aligned from 0 to ind_write */
		for (int i = (int)subghz_raw_rssi_head; i >= 0; i--)
		{
			if (subghz_raw_rssi_history[i] > 0)
				u8g2_DrawVLine(&m1_u8g2, i, bottom - subghz_raw_rssi_history[i],
				               subghz_raw_rssi_history[i]);
		}

		/* Current RSSI at write position */
		if (subghz_raw_rssi_current > 0)
		{
			u8g2_DrawVLine(&m1_u8g2, subghz_raw_rssi_head + 1,
			               bottom - subghz_raw_rssi_current,
			               subghz_raw_rssi_current);
		}

		cursor_x = (int)subghz_raw_rssi_head;

		if (cursor_x > 3)
		{
			/* Current RSSI echo one pixel before cursor */
			if (subghz_raw_rssi_current > 0)
				u8g2_DrawVLine(&m1_u8g2, cursor_x - 1,
				               bottom - subghz_raw_rssi_current,
				               subghz_raw_rssi_current);

			/* Dashed vertical cursor line */
			for (uint8_t y = SUBGHZ_RAW_TOP_SCALE + 1; y < bottom; y += cursor_w * 2)
				u8g2_DrawVLine(&m1_u8g2, cursor_x, y, cursor_w);

			/* Small triangle at top of cursor */
			u8g2_DrawHLine(&m1_u8g2, cursor_x - 2, SUBGHZ_RAW_TOP_SCALE - 2, 5);
			u8g2_DrawHLine(&m1_u8g2, cursor_x - 1, SUBGHZ_RAW_TOP_SCALE - 1, 3);
		}
	}
	else
	{
		/* Wrapped: draw all entries, oldest first */
		int base = SUBGHZ_RAW_RSSI_HISTORY_SIZE - (int)subghz_raw_rssi_head;
		for (int i = SUBGHZ_RAW_RSSI_HISTORY_SIZE; i > 0; i--)
		{
			int ind = i - base;
			if (ind < 0) ind += SUBGHZ_RAW_RSSI_HISTORY_SIZE;
			if (subghz_raw_rssi_history[ind] > 0)
				u8g2_DrawVLine(&m1_u8g2, i, bottom - subghz_raw_rssi_history[ind],
				               subghz_raw_rssi_history[ind]);
		}

		cursor_x = SUBGHZ_RAW_RSSI_HISTORY_SIZE;

		/* Current RSSI at cursor */
		if (subghz_raw_rssi_current > 0)
		{
			u8g2_DrawVLine(&m1_u8g2, cursor_x - 1,
			               bottom - subghz_raw_rssi_current,
			               subghz_raw_rssi_current);
			u8g2_DrawVLine(&m1_u8g2, cursor_x + 1,
			               bottom - subghz_raw_rssi_current,
			               subghz_raw_rssi_current);
		}

		/* Dashed vertical cursor line */
		for (uint8_t y = SUBGHZ_RAW_TOP_SCALE + 1; y < bottom; y += cursor_w * 2)
			u8g2_DrawVLine(&m1_u8g2, cursor_x, y, cursor_w);

		/* Small triangle at top of cursor */
		u8g2_DrawHLine(&m1_u8g2, cursor_x - 2, SUBGHZ_RAW_TOP_SCALE - 2, 5);
		u8g2_DrawHLine(&m1_u8g2, cursor_x - 1, SUBGHZ_RAW_TOP_SCALE - 1, 3);
	}

	/* Dashed horizontal threshold line — spans the full waveform width.
	 * Drawn last so it is always visible as a reference even when bars reach
	 * or exceed this level.  The Y position maps the user-configured RSSI
	 * threshold (subghz_cfg.rssi_threshold) through the same scale used for
	 * bar heights: thresh_h = (threshold_dbm - min_dbm) / divider,
	 * thresh_y = bottom - thresh_h. */
	{
		/* Clamp threshold to the drawable range before converting.
		 * If rssi_threshold < SUBGHZ_RAW_THRESHOLD_MIN the subtraction
		 * yields a negative float; casting that to uint8_t wraps to a
		 * large value and thresh_y falls off the top of the display. */
		float thresh_dbm = (float)subghz_cfg.rssi_threshold;
		if (thresh_dbm < SUBGHZ_RAW_THRESHOLD_MIN)
			thresh_dbm = SUBGHZ_RAW_THRESHOLD_MIN;
		uint8_t thresh_h = (uint8_t)((thresh_dbm - SUBGHZ_RAW_THRESHOLD_MIN)
		                             / SUBGHZ_RAW_RSSI_DIVIDER);
		int thresh_y = bottom - (int)thresh_h;
		if (thresh_y > SUBGHZ_RAW_TOP_SCALE && thresh_y < SUBGHZ_RAW_BOTTOM_Y)
		{
			for (int x = 1; x < SUBGHZ_RAW_END_SCALE; x += 4)
				u8g2_DrawHLine(&m1_u8g2, x, thresh_y, 2);
		}
	}
}


/*============================================================================*/
/* Phase 5 — Protocol-Specific Emulation (static-code TX)                     */
/*============================================================================*/

/* Maximum pulse pairs: data_bits * 2 (mark+space per bit) + 2 (sync pulse) + margin */
#define SUBGHZ_TX_ENCODE_BUF_MAX  280  /* 128 bits × 2 + sync + margin */

/* TX pulse buffer — allocated on first use, freed when no longer needed */
static uint16_t *subghz_tx_encode_buf = NULL;
static uint16_t  subghz_tx_encode_len = 0;  /* Number of uint16_t samples */

/*============================================================================*/
/**
  * @brief  Check if a protocol is a static-code type safe for TX emulation.
  *         Static codes have a fixed key (no rolling code / hopping).
  *
  *         Uses the protocol registry's type field instead of a hardcoded
  *         switch-case — any protocol registered as SubGhzProtocolTypeStatic
  *         is automatically eligible for TX.  Rolling-code (Dynamic),
  *         Weather, and TPMS protocols are excluded.
  *
  * @param  protocol  protocol enum value (registry index)
  * @retval true if protocol is static-code and safe to transmit
  */
/*============================================================================*/
static bool subghz_protocol_is_static(uint16_t protocol)
{
	if (protocol >= subghz_protocol_registry_count)
		return false;
	return (subghz_protocol_registry[protocol].type == SubGhzProtocolTypeStatic);
}

/*============================================================================*/
/**
  * @brief  Encode a decoded static-code signal into a TX pulse duration array.
  *         Uses standard OOK PWM encoding:
  *           bit 0 = te_short HIGH, te_long LOW
  *           bit 1 = te_long HIGH, te_short LOW
  *         Followed by a sync gap (te_short HIGH, te_short × 30 LOW).
  *
  * @param  key       Decoded key value (MSB-first)
  * @param  bit_len   Number of data bits
  * @param  te_short  Short timing element in μs
  * @param  te_long   Long timing element in μs
  * @param  buf       Output buffer (uint16_t pulse durations)
  * @param  buf_max   Maximum number of uint16_t entries in buf
  * @retval Number of uint16_t entries written, or 0 on error
  */
/*============================================================================*/
static uint16_t subghz_encode_ook_pwm(uint64_t key, uint16_t bit_len,
                                       uint16_t te_short, uint16_t te_long,
                                       uint16_t *buf, uint16_t buf_max)
{
	uint16_t idx = 0;

	if (bit_len == 0 || bit_len > 64 || buf == NULL)
		return 0;

	/* Need 2 entries per bit + 2 for sync gap */
	if ((uint16_t)(bit_len * 2 + 2) > buf_max)
		return 0;

	/* Encode data bits MSB-first */
	for (int16_t b = bit_len - 1; b >= 0; b--)
	{
		if ((key >> b) & 1ULL)
		{
			/* bit 1: long mark, short space */
			buf[idx++] = te_long;
			buf[idx++] = te_short;
		}
		else
		{
			/* bit 0: short mark, long space */
			buf[idx++] = te_short;
			buf[idx++] = te_long;
		}
	}

	/* Sync/gap: short mark followed by long pause (30× te_short) */
	buf[idx++] = te_short;
	buf[idx++] = te_short * 30;

	return idx;
}

/*============================================================================*/
/**
  * @brief  Transmit a decoded static-code signal from the history.
  *         Pauses the current RX, encodes the signal as pulse durations,
  *         transmits via DMA, waits for completion, then restores RX.
  *
  *         The transmitter sends the encoded signal with 4 repeats (same as
  *         raw replay) at the entry's original capture frequency.
  *
  * @param  entry  pointer to the history entry to transmit
  * @retval true if transmitted successfully, false on error
  */
/*============================================================================*/
static bool subghz_transmit_static_signal(const SubGHz_History_Entry_t *entry)
{
	uint16_t proto_idx;
	uint16_t te_short, te_long;
	S_M1_SubGHz_Band tx_band;

	if (entry == NULL || entry->info.key == 0)
		return false;

	proto_idx = entry->info.protocol;
	if (!subghz_protocol_is_static(proto_idx))
		return false;

	/* Use TE from the decoded signal if available, otherwise from protocol table */
	te_short = (entry->info.te > 0) ? entry->info.te
	           : subghz_protocols_list[proto_idx].te_short;
	te_long  = subghz_protocols_list[proto_idx].te_long;

	/* Scale te_long proportionally if decoded TE differs from table default */
	if (entry->info.te > 0 && subghz_protocols_list[proto_idx].te_short > 0)
	{
		te_long = (uint16_t)((uint32_t)te_long * te_short
		          / subghz_protocols_list[proto_idx].te_short);
	}

	/* Allocate TX buffer if not already */
	if (subghz_tx_encode_buf == NULL)
	{
		subghz_tx_encode_buf = pvPortMalloc(SUBGHZ_TX_ENCODE_BUF_MAX * sizeof(uint16_t));
		if (subghz_tx_encode_buf == NULL)
			return false;
	}

	/* Encode the signal */
	subghz_tx_encode_len = subghz_encode_ook_pwm(
		entry->info.key, entry->info.bit_len,
		te_short, te_long,
		subghz_tx_encode_buf, SUBGHZ_TX_ENCODE_BUF_MAX);

	if (subghz_tx_encode_len == 0)
	{
		vPortFree(subghz_tx_encode_buf);
		subghz_tx_encode_buf = NULL;
		return false;
	}

	/* Determine TX band from the signal's capture frequency */
	tx_band = subghz_freq_hz_to_band(entry->frequency);

	/* Check region restrictions */
	if (sub_ghz_fcc_ism_band_check(tx_band, 0))
	{
		m1_buzzer_notification();
		m1_message_box(&m1_u8g2, "TX Blocked:", "Region restricts", "this frequency.", "Set Region to Off");
		vPortFree(subghz_tx_encode_buf);
		subghz_tx_encode_buf = NULL;
		return false;
	}

	/* Pause RX and prepare TX */
	sub_ghz_rx_pause();

	/* Set up TX hardware and radio */
	sub_ghz_tx_raw_init();

	/* Tune radio to the signal's capture frequency */
	subghz_custom_freq_hz = entry->frequency;
	sub_ghz_set_opmode(SUB_GHZ_OPMODE_TX, tx_band, 0,
	                   tx_power_values[subghz_tx_power_idx]);

	/* Show "Sending..." overlay */
	m1_message_box(&m1_u8g2, "Sending...",
	               protocol_text[proto_idx], "", "");

	/* Transmit with repeats */
	sub_ghz_transmit_raw(
		(uint32_t)subghz_tx_encode_buf,
		(uint32_t)&timerhdl_subghz_tx.Instance->ARR,
		subghz_tx_encode_len,
		SUBGHZ_TX_RAW_REPLAY_REPEAT_DEFAULT);

	/* Wait for TX to complete (DMA transfer) */
	uint32_t timeout = 2000; /* max 2 seconds */
	while (subghz_decenc_ctl.ntx_raw_repeat > 0 && timeout > 0)
	{
		vTaskDelay(pdMS_TO_TICKS(10));
		timeout -= 10;
	}

	/* Stop TX and clean up */
	sub_ghz_raw_tx_stop();
	sub_ghz_tx_raw_deinit();
	sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, tx_band, 0, 0);

	/* Free encode buffer */
	vPortFree(subghz_tx_encode_buf);
	subghz_tx_encode_buf = NULL;

	/* Restore RX on the original recording frequency */
	sub_ghz_set_opmode(SUB_GHZ_OPMODE_RX, subghz_scan_config.band, 0, 0);
	sub_ghz_rx_init();
	sub_ghz_rx_start();

	/* Show success */
	m1_message_box(&m1_u8g2, "Sent!",
	               protocol_text[proto_idx], "", "BACK to continue");

	return true;
}


/*============================================================================*/
/**
  * @brief  Save a history entry as a Flipper-compatible .sub file.
  *         Prompts for filename via virtual keyboard, auto-generating a
  *         default name from the protocol name and key value.
  * @param  entry  pointer to the history entry to save
  * @retval true if saved successfully, false otherwise
  */
/*============================================================================*/
static bool subghz_save_history_entry(const SubGHz_History_Entry_t *entry)
{
	char sub_path[64];
	char default_name[32];
	char new_name[32];

	if (entry == NULL || entry->info.key == 0)
		return false;

	/* Build a default filename from protocol + truncated key */
	snprintf(default_name, sizeof(default_name), "%.12s_%lX",
	         protocol_text[entry->info.protocol],
	         (unsigned long)(uint32_t)entry->info.key);

	/* Prompt user for filename (or accept default) */
	if (!m1_vkb_get_filename("Save signal as:", default_name, new_name))
		return false; /* User cancelled */

	uint8_t fmt = subghz_get_save_fmt_ext();
	const char *ext = (fmt == 1) ? ".sgh" : ".sub";
	snprintf(sub_path, sizeof(sub_path), "/SUBGHZ/%s%s", new_name, ext);

	bool saved = (fmt == 1)
	    ? flipper_subghz_save_m1native_key(sub_path,
	          entry->frequency, "FuriHalSubGhzPresetOok650Async",
	          protocol_text[entry->info.protocol],
	          entry->info.bit_len, entry->info.key, entry->info.te)
	    : flipper_subghz_save_key(sub_path,
	          entry->frequency, "FuriHalSubGhzPresetOok650Async",
	          protocol_text[entry->info.protocol],
	          entry->info.bit_len, entry->info.key, entry->info.te);
	if (saved)
	{
		m1_message_box(&m1_u8g2, "Signal saved:", sub_path + 8, "", "BACK to continue");
		return true;
	}
	else
	{
		m1_message_box(&m1_u8g2, "Save failed!", sub_path + 8, "", "BACK to continue");
		return false;
	}
}


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
		snprintf((char *)datfile_info.dat_filename, sizeof(datfile_info.dat_filename), "%s/%s", f_info->dir_name, f_info->file_name);

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
		{
			uint32_t freq_hz_file = (uint32_t)strtoul(str, &end_ptr, 10);
			if ( freq_hz_file==0 )
				break;
			subghz_replay_freq = (float)freq_hz_file / 1000000.0f; // Convert to MHz for display
			/* Try exact match first via lookup */
			subghz_replay_band = subghz_freq_hz_to_band(freq_hz_file);
			if (subghz_replay_band != SUB_GHZ_BAND_CUSTOM)
			{
				subghz_replay_channel = 0;
			}
			else
			{
				/* Try float range matching for channel calculation */
				for (subghz_replay_band=0; subghz_replay_band<SUB_GHZ_BAND_EOL; subghz_replay_band++)
				{
					freq_min = subghz_band_steps[subghz_replay_band][0];
					freq_max = freq_min;
					freq_max += 0.25f*subghz_band_steps[subghz_replay_band][1];
					if ( (subghz_replay_freq >= freq_min) && (subghz_replay_freq <= freq_max) )
						break;
				}
				if ( subghz_replay_band >= SUB_GHZ_BAND_EOL )
				{
					/* Use CUSTOM band with exact frequency from file */
					subghz_replay_band = SUB_GHZ_BAND_CUSTOM;
					subghz_replay_channel = 0;
					subghz_custom_freq_hz = freq_hz_file;
				}
				else
				{
					subghz_replay_channel = 0;
					while ( subghz_replay_freq > freq_min )
					{
						freq_min += 0.25f;
						subghz_replay_channel++;
					}
				}
			}
		}

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
/** @brief  Case-insensitive substring search (like POSIX strcasestr). */
static const char *stristr(const char *haystack, const char *needle)
{
	if (!needle[0]) return haystack;
	for (; *haystack; haystack++)
	{
		const char *h = haystack, *n = needle;
		while (*h && *n && (tolower((unsigned char)*h) == tolower((unsigned char)*n)))
		{
			h++;
			n++;
		}
		if (!*n) return haystack;
	}
	return NULL;
}


/*============================================================================*/
/* Async TX replay state                                                       */
/*============================================================================*/
/*
 * Path of the temporary M1 NOISE .sgh file produced by the Flipper-format
 * converter (sub_ghz_replay_prepare_flipper).  File-scope so it can be
 * returned to scene callers via out_tmp_path.
 */
#define FLIPPER_SUB_TMP_SGH   "/SUBGHZ/_flipper_tmp.sgh"

/*
 * Per-prepare button-override slot consumed by the Flipper-format converter
 * to implement Transmitter-scene LEFT/RIGHT button cycling for rolling-code
 * remotes.  -1 = no override (use the parsed file value).  0..15 = override
 * the protocol's button-field bits inside the parsed key_value, if and only
 * if the protocol is supported by subghz_button_override.  The converter
 * resets this slot to -1 at the end of every conversion, so each
 * set + prepare cycle is self-contained.
 */
static int8_t s_button_override = -1;

void sub_ghz_replay_set_button_override(int8_t button_index)
{
	if (button_index < 0 || button_index > 15)
		s_button_override = -1;
	else
		s_button_override = button_index;
}

/*
 * Tracks whether sub_ghz_replay_start_async() has armed TX and whether
 * a Q_EVENT_SUBGHZ_TX completion is still pending.  Set true on a
 * successful start, cleared on every teardown path (natural completion,
 * error, abort, ISM-block).  Late completion events arriving after a teardown
 * are filtered by this flag in sub_ghz_replay_continue_async().
 */
static volatile bool s_replay_async_active = false;

/* Forward declaration of the internal teardown shared by completion / abort
 * / start-failure paths.  Defined below alongside the async primitives. */
static void subghz_replay_async_teardown(void);

/*============================================================================*/
/**
 * @brief  Arm the radio for an async TX after a prepare_* call.
 *
 * See doc in m1_sub_ghz.h.
 *
 * On success the SI4463 is powered up, TIM1+DMA is armed, the LED fast-blink
 * is on, and the DMA TC ISR will post Q_EVENT_SUBGHZ_TX events.
 *
 * Failure paths fully tear down internal resources before returning so the
 * caller does NOT need to call abort.
 *
 * @retval 0 = success, 1 = ISM-restricted (user already informed via message
 *         box), 4 = ring-buffer OOM, 5 = streaming sample init failed,
 *         6 = TX init / sub_ghz_replay_start internal error.
 */
/*============================================================================*/
uint8_t sub_ghz_replay_start_async(void)
{
	/* Defensive: clear any stale "active" flag from a prior aborted run.
	 * subghz_replay_async_teardown() is a safe no-op when nothing is held. */
	if (s_replay_async_active)
		subghz_replay_async_teardown();

	if (sub_ghz_ring_buffers_init())
		return 4;

	if (sub_ghz_raw_samples_init())
	{
		sub_ghz_raw_samples_deinit(false);
		sub_ghz_ring_buffers_deinit();
		return 5;
	}

	menu_sub_ghz_init();

	M1_LOG_I(M1_LOGDB_TAG, "SGH replay (async): band=%d freq=%lu samples_init OK\r\n",
	         subghz_replay_band, subghz_custom_freq_hz);

	subghz_replay_ret_code = sub_ghz_replay_start(false, subghz_replay_band,
	                                              subghz_replay_channel, 255);

	M1_LOG_I(M1_LOGDB_TAG, "SGH replay (async): replay_start returned %d\r\n",
	         subghz_replay_ret_code);

	if (subghz_replay_ret_code == 1)
	{
		/* ISM region blocked TX — sub_ghz_replay_start already showed the
		 * user message box.  Full teardown: free buffers and power off radio.
		 * Do NOT reset the shared main event queue here, because this async
		 * start path can run while the Read Raw scene is still active and
		 * pending keypad/scene events must be preserved.  Do NOT set
		 * s_replay_async_active = true. */
		sub_ghz_raw_samples_deinit(false);
		sub_ghz_ring_buffers_deinit();
		menu_sub_ghz_exit();
		return 1;
	}
	else if (subghz_replay_ret_code == 0)
	{
		/* TX init failed inside sub_ghz_replay_start without ISM block.
		 * Surface a generic error and tear down with a non-resetting cleanup
		 * sequence, so unrelated queued scene/keypad input is kept. */
		char err_msg[48];
		snprintf(err_msg, sizeof(err_msg), "Band:%d Freq:%lu",
		         subghz_replay_band, subghz_custom_freq_hz);
		m1_message_box(&m1_u8g2, "Replay failed!", err_msg, "", "BACK to return");

		/* Start failed before s_replay_async_active was set, so call explicit
		 * no-queue-reset cleanup here instead of subghz_replay_async_teardown(). */
		sub_ghz_raw_samples_deinit(false);
		sub_ghz_ring_buffers_deinit();
		sub_ghz_tx_raw_deinit_impl(false);
		sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, SUB_GHZ_BAND_EOL, 0, 0);
		subghz_decenc_ctl.ntx_raw_repeat = 0;
		menu_sub_ghz_exit();
		return 6;
	}

	/* Success — first DMA burst is in flight, completion will arrive as
	 * Q_EVENT_SUBGHZ_TX.  Arm LED blink and flip the active flag last so
	 * a racing completion event sees a fully-consistent state. */
	double_buffer_ptr_id = 1;
	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_M,
	                  LED_FASTBLINK_ONTIME_M);
	s_replay_async_active = true;
	return 0;
}

/*============================================================================*/
/**
 * @brief  Internal teardown for async replay.
 *
 * Idempotent: safe to call multiple times.  Frees buffers, stops TX DMA,
 * turns off LED blink and powers down the SI4463.
 * Leaves s_replay_async_active = false.
 */
/*============================================================================*/
static void subghz_replay_async_teardown(void)
{
	if (!s_replay_async_active)
		return;
	s_replay_async_active = false;

	m1_led_fast_blink(LED_BLINK_ON_RGB, LED_FASTBLINK_PWM_OFF,
	                  LED_FASTBLINK_ONTIME_OFF);
	sub_ghz_raw_tx_stop();
	sub_ghz_raw_samples_deinit(false);
	sub_ghz_ring_buffers_deinit();
	sub_ghz_tx_raw_deinit_impl(false);
	sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, SUB_GHZ_BAND_EOL, 0, 0);
	subghz_decenc_ctl.ntx_raw_repeat = 0;
	menu_sub_ghz_exit();
}

/*============================================================================*/
/**
 * @brief  Advance the async TX on a Q_EVENT_SUBGHZ_TX completion event.
 *         See doc in m1_sub_ghz.h.
 */
/*============================================================================*/
sub_ghz_replay_async_status_t sub_ghz_replay_continue_async(bool repeat_on_idle)
{
	/* Late or stray completion: nothing armed.  This branch is the safety
	 * net for late Q_EVENT_SUBGHZ_TX events arriving after sub_ghz_replay_abort(). */
	if (!s_replay_async_active)
		return SUBGHZ_REPLAY_ASYNC_DONE;

	subghz_replay_ret_code = sub_ghz_replay_continue(subghz_replay_ret_code);

	if ((subghz_replay_ret_code & SUB_GHZ_RAW_DATA_PARSER_ERROR_MASK) != 0)
	{
		subghz_replay_async_teardown();
		return SUBGHZ_REPLAY_ASYNC_ERROR;
	}

	if (subghz_replay_ret_code == SUB_GHZ_RAW_DATA_PARSER_IDLE)
	{
		if (repeat_on_idle)
		{
			/* Continuous emulation — auto-restart for one more cycle.  Used
			 * by the legacy blocking wrappers (Saved-PACKET / Playlist). */
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
				return SUBGHZ_REPLAY_ASYNC_RUNNING;
			}
			/* Restart failed — fall through to teardown */
			subghz_replay_async_teardown();
			return SUBGHZ_REPLAY_ASYNC_ERROR;
		}

		/* One-shot — natural completion. */
		subghz_replay_async_teardown();
		return SUBGHZ_REPLAY_ASYNC_DONE;
	}

	return SUBGHZ_REPLAY_ASYNC_RUNNING;
}

/*============================================================================*/
/**
 * @brief  Abort an in-progress async replay synchronously.
 *         See doc in m1_sub_ghz.h.
 */
/*============================================================================*/
void sub_ghz_replay_abort(void)
{
	subghz_replay_async_teardown();
}

bool sub_ghz_replay_async_is_active(void)
{
	return s_replay_async_active;
}

/*============================================================================*/
/**
 * @brief  Flush the shared main queue when a legacy blocking replay wrapper
 *         returns control to its caller scene.
 *
 * The async Read Raw scene must preserve queued events, but Saved / Playlist /
 * Remote / Bind Wizard temporarily own main_q_hdl via their private blocking
 * replay loop.  They flush on return so queued keypad or late TX-complete
 * events do not leak into the caller scene.  main_q_hdl is null-guarded for
 * early init / teardown paths.
 */
/*============================================================================*/
static void subghz_replay_blocking_reset_queue(void)
{
	if (main_q_hdl != NULL)
		xQueueReset(main_q_hdl);
}

/*============================================================================*/
/**
 * @brief  Private mini event loop used by the legacy blocking wrappers
 *         (sub_ghz_replay_flipper_file / sub_ghz_replay_datafile).
 *
 * Runs entirely on top of the async primitives — receives main_q_hdl events,
 * drives sub_ghz_replay_continue_async(true) on every Q_EVENT_SUBGHZ_TX, and
 * exits when the user presses BACK (calling sub_ghz_replay_abort()) or when
 * an error tears the replay down naturally.
 *
 * The auto-restart-until-BACK semantics of the legacy loop are preserved by
 * passing repeat_on_idle=true to continue_async.  This keeps the Saved-PACKET
 * emulate path and Playlist behaviour unchanged.
 *
 * Preconditions: a sub_ghz_replay_prepare_* call has set the globals.
 *
 * @retval 0 = completed or internally-handled failure (legacy behaviour for
 *         ISM-block / generic TX init errors), 4/5 = allocation failures
 *         surfaced to callers so scenes can show "Memory error".
 */
/*============================================================================*/
static uint8_t subghz_replay_run_blocking(void)
{
	uint8_t start_ret = sub_ghz_replay_start_async();
	if (start_ret != 0)
	{
		/* ISM block (1) and TX init error (6) already showed a message box
		 * and tore everything down.  Return 0 to match the legacy contract:
		 * blocking wrappers absorb these and treat them as completed.
		 *
		 * Allocation failures (4/5) were historically surfaced so callers
		 * could show "Memory error"; preserve that behavior. */
		if (start_ret == 4 || start_ret == 5)
		{
			subghz_replay_blocking_reset_queue();
			return start_ret;
		}
		subghz_replay_blocking_reset_queue();
		return 0;
	}

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
				sub_ghz_replay_abort();
				running = false;
			}
			else if (btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
			{
				/* OK in the blocking wrapper is a manual retry hook for the
				 * (rare) case where the TX engine stalled in IDLE between
				 * completion events.  With repeat_on_idle=true in
				 * continue_async, the engine restarts itself, so this is
				 * primarily defensive and matches the legacy event loop. */
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
						sub_ghz_replay_abort();
						m1_message_box(&m1_u8g2, "Replay failed!",
						               "TX init error", "",
						               "BACK to return");
						running = false;
					}
				}
			}
		}
		else if (q_item.q_evt_type == Q_EVENT_SUBGHZ_TX)
		{
			sub_ghz_replay_async_status_t st =
			    sub_ghz_replay_continue_async(true);
			if (st != SUBGHZ_REPLAY_ASYNC_RUNNING)
			{
				/* Auto-restart failed or replay completed.  Teardown done. */
				running = false;
			}
		}
	}

	/* Safety net: ensure full teardown.  No-op when already torn down. */
	if (s_replay_async_active)
		sub_ghz_replay_abort();

	/* Legacy blocking callers own the main loop while replay is active, so any
	 * queued keypad or late TX-complete events from that private loop must not
	 * leak back into the caller scene after we return. */
	subghz_replay_blocking_reset_queue();

	return 0;
}

/*============================================================================*/
/**
 * @brief  Replay an M1 native .sgh NOISE file directly — no conversion.
 *
 * For M1 native NOISE files the original .sgh is already in the streaming
 * format expected by sub_ghz_raw_samples_init().  This function skips the
 * temp-file conversion path entirely and feeds the file straight into the
 * streaming engine, eliminating the "Memory error" failure that occurred when
 * C3.12/SiN360-produced .sgh files were routed through sub_ghz_replay_flipper_file().
 *
 * Callers obtain `frequency` and `modulation` from the already-loaded
 * flipper_subghz_signal_t (saved scene) or from flipper_subghz_probe()
 * (playlist scene), so no additional file I/O is required here.
 *
 * @param  sgh_path   Path to the M1 native .sgh NOISE file on the SD card
 * @param  frequency  Carrier frequency in Hz (from the file header)
 * @param  modulation MODULATION_OOK / MODULATION_FSK / MODULATION_ASK
 * @retval 0 = success, non-zero = error code (same as sub_ghz_replay_flipper_file)
 */
uint8_t sub_ghz_replay_datafile(const char *sgh_path,
                                 uint32_t    frequency,
                                 uint8_t     modulation)
{
	uint8_t ret = sub_ghz_replay_prepare_datafile(sgh_path, frequency, modulation);
	if (ret != 0)
		return ret;
	return subghz_replay_run_blocking();
}

/*============================================================================*/
/**
 * @brief  Prepare globals for an async M1 native .sgh stream.
 *         Shared by the blocking wrapper and the Read Raw scene's async path.
 *
 * See doc in m1_sub_ghz.h.
 */
/*============================================================================*/
uint8_t sub_ghz_replay_prepare_datafile(const char *sgh_path,
                                        uint32_t frequency,
                                        uint8_t modulation)
{
	if (sgh_path == NULL || frequency == 0)
		return 1;

	if (frequency < SUBGHZ_MIN_FREQ_HZ || frequency > SUBGHZ_MAX_FREQ_HZ)
		return 3;

	/* Set globals used by the shared start_async path */
	subghz_replay_freq    = (float)frequency / 1000000.0f;
	subghz_replay_mod     = modulation;
	subghz_custom_freq_hz = frequency;
	subghz_replay_band    = subghz_freq_hz_to_band(frequency);
	subghz_replay_channel = 0;
	subghz_scan_config.modulation = modulation;

	/* Force CUSTOM band for FSK (same logic as sub_ghz_replay_flipper_file) */
	if (modulation == MODULATION_FSK && subghz_replay_band != SUB_GHZ_BAND_CUSTOM)
		subghz_replay_band = SUB_GHZ_BAND_CUSTOM;

	/* Point the streaming engine at the original file — no temp file needed */
	strncpy((char *)datfile_info.dat_filename, sgh_path,
	        sizeof(datfile_info.dat_filename) - 1);
	datfile_info.dat_filename[sizeof(datfile_info.dat_filename) - 1] = '\0';

	return 0;
}

/*============================================================================*/
/**
 * @brief  Convert a Flipper .sub file to a temp M1 .sgh and set up globals.
 *         Internal helper shared by both the blocking wrapper
 *         (sub_ghz_replay_flipper_file) and the async scene path
 *         (sub_ghz_replay_prepare_flipper).
 *
 *         Handles RAW type .sub files and M1 PACKET .sgh files.  Streams
 *         via a temp file on SD card so there is no sample-count limit.
 *
 *         On failure all temp files have been unlinked already; the caller
 *         does NOT need to clean up.
 *
 * @param  sub_path  Path to the .sub / .sgh source file on the SD card
 * @retval 0 = success, non-zero = error
 */
/*============================================================================*/
static uint8_t subghz_replay_flipper_to_tmp(const char *sub_path)
{
#define FLIPPER_SUB_LINE_MAX  4096
#define FLIPPER_SUB_OUT_MAX   256

	FIL f_sub, f_sgh;
	FRESULT fr;
	char *line_buf;
	char *out_buf;
	uint32_t frequency = 0;
	uint8_t modulation = MODULATION_OOK;
	bool is_raw = false;
	bool is_key = false;
	bool has_data = false;
	bool in_raw_continuation = false;
	float freq_mhz;

	/* Parser state for cross-buffer RAW_Data line handling.
	 * When a long RAW_Data line exceeds the buffer, f_gets can split a
	 * number at the boundary (e.g. "12345" → "123" + "45").  The parser
	 * saves any trailing digits and recombines them on the next read. */
	SubGhzRawLineState raw_line_state;
	subghz_raw_line_state_init(&raw_line_state);

	/* KEY file fields */
	char key_protocol[32] = {0};
	char key_manufacture[48] = {0};
	uint64_t key_value = 0;
	uint32_t key_bit_count = 0;
	uint32_t key_te = 0;
	/* Phase 9d-3 — optional `CounterMode:` field on KeeLoq-family
	 * .sub files.  Defaults to INCREMENT for files that omit the
	 * field (matches the parse behaviour in flipper_subghz_load). */
	flipper_subghz_counter_mode_t key_counter_mode =
	    FLIPPER_SUBGHZ_COUNTER_MODE_INCREMENT;

	/* Latch the button-override slot at entry and clear the static slot
	 * immediately so every return path leaves it disabled (set + prepare
	 * cycles are self-contained).  Negative = no override active. */
	const int8_t button_override = s_button_override;
	s_button_override = -1;

	line_buf = pvPortMalloc(FLIPPER_SUB_LINE_MAX);
	if (!line_buf) return 1;
	out_buf = pvPortMalloc(FLIPPER_SUB_OUT_MAX);
	if (!out_buf) { vPortFree(line_buf); return 1; }

	/* ── 1. Open .sub source ── */
	fr = f_open(&f_sub, sub_path, FA_READ);
	if (fr != FR_OK)
	{
		vPortFree(line_buf); vPortFree(out_buf);
		return 1;
	}

	/* ── 2. Create temp .sgh file ── */
	f_mkdir("/SUBGHZ");
	fr = f_open(&f_sgh, FLIPPER_SUB_TMP_SGH, FA_WRITE | FA_CREATE_ALWAYS);
	if (fr != FR_OK)
	{
		f_close(&f_sub);
		vPortFree(line_buf); vPortFree(out_buf);
		return 1;
	}

	/* ── 3. Parse .sub file — collect header + data ── */
	while (f_gets(line_buf, FLIPPER_SUB_LINE_MAX, &f_sub))
	{
		/* Check if this read reached a real line ending */
		size_t len = strlen(line_buf);
		bool line_complete = (len > 0 &&
		    (line_buf[len - 1] == '\n' || line_buf[len - 1] == '\r'));

		/* Strip trailing CR/LF */
		while (len > 0 && (line_buf[len - 1] == '\r' || line_buf[len - 1] == '\n'))
			line_buf[--len] = '\0';

		/* Continuation of a long RAW_Data line that was split by f_gets */
		if (in_raw_continuation)
		{
			uint32_t samples[64];
			uint16_t nsamples = subghz_parse_raw_data_line(
				line_buf, line_complete, &raw_line_state, samples, 64);

			if (nsamples > 0)
			{
				int pos = snprintf(out_buf, FLIPPER_SUB_OUT_MAX, "%s",
				                   SUB_GHZ_DATAFILE_DATA_KEYWORD);
				for (uint16_t si = 0; si < nsamples; si++)
				{
					pos += snprintf(&out_buf[pos],
					                (size_t)(FLIPPER_SUB_OUT_MAX - pos),
					                " %lu", (unsigned long)samples[si]);
					if (pos >= FLIPPER_SUB_OUT_MAX - 16)
					{
						strcat(out_buf, "\r\n");
						f_puts(out_buf, &f_sgh);
						pos = snprintf(out_buf, FLIPPER_SUB_OUT_MAX, "%s",
						               SUB_GHZ_DATAFILE_DATA_KEYWORD);
					}
				}
				if (pos > 6) /* more than just "Data:" */
				{
					strcat(out_buf, "\r\n");
					f_puts(out_buf, &f_sgh);
					has_data = true;
				}
			}
			in_raw_continuation = !line_complete;
			continue;
		}

		if (strncmp(line_buf, "Filetype:", 9) == 0)
		{
			if (strstr(line_buf, "RAW") || strstr(line_buf, "NOISE"))
				is_raw = true;
			else if (strstr(line_buf, "Key") || strstr(line_buf, "PACKET"))
				is_key = true;
			f_puts("Filetype: M1 SubGHz NOISE\r\n", &f_sgh);
		}
		else if (strncmp(line_buf, "Version:", 8) == 0)
		{
			f_puts("Version: 0.8\r\n", &f_sgh);
		}
		else if (strncmp(line_buf, "Frequency:", 10) == 0)
		{
			frequency = (uint32_t)strtoul(line_buf + 10, NULL, 10);
			snprintf(out_buf, FLIPPER_SUB_OUT_MAX, "Frequency: %lu\r\n",
			         (unsigned long)frequency);
			f_puts(out_buf, &f_sgh);
		}
		else if (strncmp(line_buf, "Preset:", 7) == 0)
		{
			if (stristr(line_buf, "OOK"))
				modulation = MODULATION_OOK;
			else if (stristr(line_buf, "ASK"))
				modulation = MODULATION_ASK;
			else if (stristr(line_buf, "FSK"))
				modulation = MODULATION_FSK;
			snprintf(out_buf, FLIPPER_SUB_OUT_MAX, "Modulation: %s\r\n",
			         subghz_modulation_text[modulation]);
			f_puts(out_buf, &f_sgh);
		}
		else if (strncmp(line_buf, "Modulation:", 11) == 0)
		{
			/* M1 native .sgh format uses "Modulation:" instead of "Preset:" */
			if (stristr(line_buf, "OOK"))
				modulation = MODULATION_OOK;
			else if (stristr(line_buf, "ASK"))
				modulation = MODULATION_ASK;
			else if (stristr(line_buf, "FSK"))
				modulation = MODULATION_FSK;
			snprintf(out_buf, FLIPPER_SUB_OUT_MAX, "Modulation: %s\r\n",
			         subghz_modulation_text[modulation]);
			f_puts(out_buf, &f_sgh);
		}
		else if (strncmp(line_buf, "Protocol:", 9) == 0)
		{
			const char *p = line_buf + 9;
			while (*p == ' ') p++;
			strncpy(key_protocol, p, sizeof(key_protocol) - 1);
			key_protocol[sizeof(key_protocol) - 1] = '\0';
		}
		else if (strncmp(line_buf, "Bit:", 4) == 0)
		{
			key_bit_count = (uint32_t)strtoul(line_buf + 4, NULL, 10);
		}
		else if (strncmp(line_buf, "Bits:", 5) == 0)
		{
			/* M1 native .sgh PACKET format uses "Bits:" instead of "Bit:" */
			key_bit_count = (uint32_t)strtoul(line_buf + 5, NULL, 10);
		}
		else if (strncmp(line_buf, "Key:", 4) == 0)
		{
			/* Parse hex bytes big-endian: "00 00 00 00 00 52 A1 2E" */
			const char *p = line_buf + 4;
			key_value = 0;
			while (*p)
			{
				while (*p == ' ') p++;
				if (*p == '\0') break;
				char *endp;
				unsigned long bv = strtoul(p, &endp, 16);
				if (endp == p) break;
				key_value = (key_value << 8) | (bv & 0xFF);
				p = endp;
			}
		}
		else if (strncmp(line_buf, "Payload:", 8) == 0)
		{
			/* M1 native .sgh PACKET format: "Payload: 0x0000000052A12E" */
			const char *p = line_buf + 8;
			while (*p == ' ') p++;
			key_value = (uint64_t)strtoull(p, NULL, 16);
		}
		else if (strncmp(line_buf, "TE:", 3) == 0)
		{
			key_te = (uint32_t)strtoul(line_buf + 3, NULL, 10);
		}
		else if (strncmp(line_buf, "BT:", 3) == 0)
		{
			/* M1 native .sgh PACKET format uses "BT:" instead of "TE:" */
			key_te = (uint32_t)strtoul(line_buf + 3, NULL, 10);
		}
		else if (strncmp(line_buf, "Manufacture:", 12) == 0)
		{
			/* Flipper SubGhz Key File: manufacturer name used for KeeLoq MK lookup */
			const char *p = line_buf + 12;
			while (*p == ' ') p++;
			/* Guard: skip if no name follows the field prefix */
			if (*p == '\0') continue;
			strncpy(key_manufacture, p, sizeof(key_manufacture) - 1);
			key_manufacture[sizeof(key_manufacture) - 1] = '\0';
			{
				size_t len = strlen(key_manufacture);
				while (len > 0 && isspace((unsigned char)key_manufacture[len - 1]))
				{
					key_manufacture[--len] = '\0';
				}
			}
		}
		else if (strncmp(line_buf, "CounterMode:", 12) == 0)
		{
			/* Phase 9d-3 — optional KeeLoq-family CounterMode field.
			 * Recognises "Static" exactly (case-sensitive, matching the
			 * Flipper file format).  Missing line, empty value, "Increment",
			 * and any unknown value all yield INCREMENT, so every existing
			 * .sub file replays unchanged.  Mirrors the parse rules in
			 * flipper_subghz_load() for cross-tool consistency. */
			const char *p = line_buf + 12;
			while (*p == ' ' || *p == '\t') p++;
			/* Strip trailing whitespace before comparison. */
			char tmp[16] = {0};
			size_t i = 0;
			while (*p != '\0' && *p != '\r' && *p != '\n' && i < sizeof(tmp) - 1)
			{
				tmp[i++] = *p++;
			}
			tmp[i] = '\0';
			while (i > 0 && isspace((unsigned char)tmp[i - 1]))
			{
				tmp[--i] = '\0';
			}
			if (strcmp(tmp, "Static") == 0)
			{
				key_counter_mode = FLIPPER_SUBGHZ_COUNTER_MODE_STATIC;
			}
			else
			{
				key_counter_mode = FLIPPER_SUBGHZ_COUNTER_MODE_INCREMENT;
			}
		}
		else if (strncmp(line_buf, "RAW_Data:", 9) == 0)
		{
			/* Parse signed values, write absolute values as Data: lines.
			 * Flipper RAW_Data lines can be thousands of chars —
			 * f_gets may split them across multiple reads.
			 * Uses the extracted raw line parser for cross-buffer handling. */
			subghz_raw_line_state_init(&raw_line_state);

			uint32_t samples[64];
			uint16_t nsamples = subghz_parse_raw_data_line(
				line_buf + 9, line_complete, &raw_line_state, samples, 64);

			if (nsamples > 0)
			{
				int pos = snprintf(out_buf, FLIPPER_SUB_OUT_MAX, "%s",
				                   SUB_GHZ_DATAFILE_DATA_KEYWORD);
				for (uint16_t si = 0; si < nsamples; si++)
				{
					pos += snprintf(&out_buf[pos],
					                (size_t)(FLIPPER_SUB_OUT_MAX - pos),
					                " %lu", (unsigned long)samples[si]);
					if (pos >= FLIPPER_SUB_OUT_MAX - 16)
					{
						strcat(out_buf, "\r\n");
						f_puts(out_buf, &f_sgh);
						pos = snprintf(out_buf, FLIPPER_SUB_OUT_MAX, "%s",
						               SUB_GHZ_DATAFILE_DATA_KEYWORD);
					}
				}
				if (pos > 6) /* more than just "Data:" */
				{
					strcat(out_buf, "\r\n");
					f_puts(out_buf, &f_sgh);
					has_data = true;
				}
			}
			/* If f_gets truncated this line, mark for continuation */
			in_raw_continuation = !line_complete;
		}
		else if (strncmp(line_buf, "Data:", 5) == 0)
		{
			/* M1 native .sgh format — Data: lines already contain unsigned
			 * values.  Pass them through to the temp file as-is. */
			f_puts(line_buf, &f_sgh);
			f_puts("\r\n", &f_sgh);
			has_data = true;
		}
	}

	f_close(&f_sub);

	/* ── 3b. KEY file: encode protocol → raw timing ── */
	if (is_key && key_protocol[0] != '\0' && key_bit_count > 0)
	{
		/* Apply the per-prepare button override (Phase 4c).  For
		 * supported protocols (KeeLoq family) this mutates the parsed
		 * key_value to encode the requested button; for unsupported
		 * protocols it is a no-op.  The KeeLoq counter-mode encoder
		 * picks up the new button via its own extract_fields(); the
		 * generic OOK PWM encoder transmits the mutated value verbatim. */
		if (button_override >= 0)
		{
			uint64_t mutated = key_value;
			(void)subghz_button_override_apply(key_protocol, key_value,
			                                   (uint8_t)button_override,
			                                   &mutated);
			key_value = mutated;
		}

		/* Use the extracted key encoder for timing resolution + encoding */
		SubGhzKeyParams key_params;
		memset(&key_params, 0, sizeof(key_params));
		strncpy(key_params.protocol, key_protocol, sizeof(key_params.protocol) - 1);
		key_params.key_value = key_value;
		key_params.bit_count = key_bit_count;
		key_params.te        = key_te;

		uint32_t npairs = 0;
		SubGhzRawPair *pairs = NULL;
		uint32_t pairs_per_rep;

		/* Protocol-specific encoders for non-standard waveforms (e.g. Magellan) */
		if (subghz_key_has_custom_encoder(key_params.protocol))
		{
			uint32_t max_pairs = subghz_key_custom_required_pairs(&key_params, 3);
			if (max_pairs == 0)
			{
				f_close(&f_sgh);
				f_unlink(FLIPPER_SUB_TMP_SGH);
				vPortFree(line_buf); vPortFree(out_buf);
				return SUBGHZ_KEY_ERR_UNSUPPORTED;
			}
			pairs_per_rep = max_pairs / 3;
			pairs = (SubGhzRawPair *)pvPortMalloc(max_pairs * sizeof(SubGhzRawPair));
			if (!pairs)
			{
				f_close(&f_sgh);
				f_unlink(FLIPPER_SUB_TMP_SGH);
				vPortFree(line_buf); vPortFree(out_buf);
				return 1;
			}
			npairs = subghz_key_encode_custom(&key_params, pairs, max_pairs, 3);
		}
		else
		{
			/* Generic OOK PWM encoding */
			SubGhzKeyTiming key_timing;
			uint8_t resolve_ret = subghz_key_resolve_timing(&key_params, &key_timing);
			if (resolve_ret != SUBGHZ_KEY_OK)
			{
				/*
				 * KeeLoq counter-mode replay: when the timing resolver
				 * returns ERR_DYNAMIC (6) for a KeeLoq-family protocol AND
				 * a Manufacture name is present in the .sub file, attempt
				 * to look up the manufacturer master key and encode a
				 * counter-incremented hop word.
				 *
				 * The keystore is loaded lazily from:
				 *   0:/SUBGHZ/keeloq_mfcodes
				 * Export this file from a Flipper Zero using RocketGod's
				 * SubGHz Toolkit app.
				 */
				if (resolve_ret == SUBGHZ_KEY_ERR_DYNAMIC &&
				    keeloq_is_keeloq_protocol(key_params.protocol) &&
				    key_manufacture[0] != '\0')
				{
					/* Lazy-load the manufacturer key table */
					if (keeloq_mfkeys_count() == 0)
						keeloq_mfkeys_load();

					KeeLoqEncParams kl_params;
					kl_params.protocol    = key_params.protocol;
					kl_params.manufacture = key_manufacture;
					kl_params.key_value   = key_value;
					kl_params.bit_count   = key_bit_count;
					kl_params.te          = key_te;
					/* Phase 9d-3 — wire the parsed CounterMode field from
					 * the .sub file.  STATIC bypasses the counter-increment
					 * step in the KeeLoq encoder so the captured hop word
					 * replays verbatim; INCREMENT (the default for files
					 * that omit the field) preserves the historical
					 * decrypt → counter+1 → re-encrypt behaviour. */
					kl_params.static_counter =
					    (key_counter_mode == FLIPPER_SUBGHZ_COUNTER_MODE_STATIC);

					SubGhzRawPair *kl_pairs  = NULL;
					uint32_t       kl_npairs = 0;

					KeeLoqEncResult kl_res = keeloq_encode_replay(
					    &kl_params, &kl_pairs, &kl_npairs, 3);

					if (kl_res == KEELOQ_ENC_OK && kl_pairs && kl_npairs > 0)
					{
						pairs          = kl_pairs;
						npairs         = kl_npairs;
						pairs_per_rep  = keeloq_pairs_per_rep(
						                     KEELOQ_PREAMBLE_PULSES, 64U);
					}
					else
					{
						uint8_t ret_code = SUBGHZ_KEY_ERR_DYNAMIC;

						/*
						 * Preserve the existing dynamic-error behavior for
						 * "no key found"/generic encode failures, but surface
						 * allocation failure distinctly so callers/UI do not
						 * misreport it as a dynamic-code issue.
						 */
						if (kl_res == KEELOQ_ENC_NOMEM)
							ret_code = 1;

						f_close(&f_sgh);
						f_unlink(FLIPPER_SUB_TMP_SGH);
						vPortFree(line_buf); vPortFree(out_buf);
						return ret_code;
					}
				}
				else
				{
					f_close(&f_sgh);
					f_unlink(FLIPPER_SUB_TMP_SGH);
					vPortFree(line_buf); vPortFree(out_buf);
					return resolve_ret; /* 6 = rolling/weather/TPMS, 7 = unsupported */
				}
			}
			else
			{
				/* Encode repetitions of the signal into raw timing pairs.
				 * For low-TE signals (TE < 250µs), scale up repetitions so
				 * that each DMA burst carries enough signal duration for
				 * reliable receiver detection.  Target: each burst ≈ 140ms
				 * (matches default TE=370 with 3 reps). */
				uint8_t reps = subghz_low_te_calc_reps(key_te);
				uint32_t clamped_bits = (key_bit_count > 64) ? 64 : key_bit_count;
				pairs_per_rep = clamped_bits + 1; /* data bits + sync gap */
				uint32_t max_pairs = pairs_per_rep * reps;
				pairs = (SubGhzRawPair *)pvPortMalloc(max_pairs * sizeof(SubGhzRawPair));
				if (!pairs)
				{
					f_close(&f_sgh);
					f_unlink(FLIPPER_SUB_TMP_SGH);
					vPortFree(line_buf); vPortFree(out_buf);
					return 1;
				}

				npairs = subghz_key_encode(&key_params, &key_timing, pairs, max_pairs, reps);
			}
		}

		/* Write encoded pairs as Data: lines to the temp .sgh file.
		 * The caller controls how many times the file is replayed per TX session;
		 * combined with the scaled in-file reps, total TX duration scales with
		 * the caller's repeat count and the TE value. */
		if (npairs > 0)
		{
			const size_t out_buf_cap = FLIPPER_SUB_OUT_MAX;
			int pos = snprintf(out_buf, out_buf_cap, "%s",
			                   SUB_GHZ_DATAFILE_DATA_KEYWORD);
			if (pos < 0)
			{
				pos = 0;
				out_buf[0] = '\0';
			}
			else if ((size_t)pos >= out_buf_cap)
			{
				pos = (int)(out_buf_cap - 1);
			}

			for (uint32_t pi = 0; pi < npairs; pi++)
			{
				char pair_buf[32];
				int pair_len = snprintf(pair_buf, sizeof(pair_buf),
				                        " %lu %lu",
				                        (unsigned long)pairs[pi].high_us,
				                        (unsigned long)pairs[pi].low_us);

				if (pair_len < 0)
					continue;

				/* Flush before appending if the next pair would exceed out_buf. */
				if (((size_t)pos + (size_t)pair_len + 3) > out_buf_cap)
				{
					pos += snprintf(&out_buf[pos], out_buf_cap - (size_t)pos, "\r\n");
					f_puts(out_buf, &f_sgh);
					pos = snprintf(out_buf, out_buf_cap, "%s",
					               SUB_GHZ_DATAFILE_DATA_KEYWORD);
					if (pos < 0)
					{
						pos = 0;
						out_buf[0] = '\0';
					}
					else if ((size_t)pos >= out_buf_cap)
					{
						pos = (int)(out_buf_cap - 1);
					}
				}

				pos += snprintf(&out_buf[pos], out_buf_cap - (size_t)pos, "%s", pair_buf);

				/* Start a new Data: line at repetition boundaries
				 * (after each sync gap = last pair of each rep). */
				bool is_rep_end = ((pi + 1) % pairs_per_rep) == 0;
				if (is_rep_end)
				{
					pos += snprintf(&out_buf[pos], out_buf_cap - (size_t)pos, "\r\n");
					f_puts(out_buf, &f_sgh);
					if (pi + 1 < npairs)
					{
						pos = snprintf(out_buf, out_buf_cap, "%s",
						               SUB_GHZ_DATAFILE_DATA_KEYWORD);
						if (pos < 0)
						{
							pos = 0;
							out_buf[0] = '\0';
						}
						else if ((size_t)pos >= out_buf_cap)
						{
							pos = (int)(out_buf_cap - 1);
						}
					}
				}
			}
			has_data = true;
		}
		vPortFree(pairs);
	}

	f_close(&f_sgh);

	vPortFree(line_buf);
	vPortFree(out_buf);

	if (!has_data || frequency == 0)
	{
		f_unlink(FLIPPER_SUB_TMP_SGH);
		return 2; /* no data or missing frequency */
	}

	/* ── 4. Use exact frequency via CUSTOM mode ── */
	freq_mhz = (float)frequency / 1000000.0f;
	subghz_replay_freq = freq_mhz;
	subghz_replay_mod  = modulation;

	if (frequency < SUBGHZ_MIN_FREQ_HZ || frequency > SUBGHZ_MAX_FREQ_HZ)
	{
		f_unlink(FLIPPER_SUB_TMP_SGH);
		return 3; /* unsupported frequency */
	}

	subghz_custom_freq_hz = frequency;
	subghz_replay_band    = subghz_freq_hz_to_band(frequency);
	subghz_replay_channel = 0;
	/* Propagate parsed modulation so sub_ghz_set_opmode uses it for CUSTOM band */
	subghz_scan_config.modulation = modulation;

	/* Standard band configs (300–433.92 MHz) are OOK-only.  Force CUSTOM
	 * band for FSK so the modulation-aware CUSTOM handler loads the 915
	 * FSK radio config and retunes to the target frequency. */
	if (modulation == MODULATION_FSK && subghz_replay_band != SUB_GHZ_BAND_CUSTOM)
		subghz_replay_band = SUB_GHZ_BAND_CUSTOM;

	/* ── 5. Set up datfile_info → temp .sgh ── */
	strncpy((char *)datfile_info.dat_filename, FLIPPER_SUB_TMP_SGH,
	        sizeof(datfile_info.dat_filename) - 1);
	datfile_info.dat_filename[sizeof(datfile_info.dat_filename) - 1] = '\0';

	/* Conversion done; globals set.  Caller is responsible for driving TX
	 * (sub_ghz_replay_start_async + continue_async) and for unlinking the
	 * temp .sgh after completion / abort. */
	return 0;

#undef FLIPPER_SUB_LINE_MAX
#undef FLIPPER_SUB_OUT_MAX
} // subghz_replay_flipper_to_tmp

/*============================================================================*/
/**
 * @brief  Public converter API used by the Read Raw scene's async path.
 *         See doc in m1_sub_ghz.h.
 */
/*============================================================================*/
uint8_t sub_ghz_replay_prepare_flipper(const char *sub_path,
                                       const char **out_tmp_path)
{
	if (!sub_path || !out_tmp_path)
		return 1;

	uint8_t ret = subghz_replay_flipper_to_tmp(sub_path);
	if (ret != 0)
		return ret;
	*out_tmp_path = FLIPPER_SUB_TMP_SGH;
	return 0;
}

/*============================================================================*/
/**
 * @brief  Blocking wrapper used by the Saved-PACKET emulate path and the
 *         Playlist scene.  Reimplemented on top of the async primitives plus
 *         the private mini event loop in subghz_replay_run_blocking().
 *
 * @param  sub_path  Source .sub / .sgh file path.
 * @retval 0 = success, non-zero = converter error.
 */
/*============================================================================*/
uint8_t sub_ghz_replay_flipper_file(const char *sub_path)
{
	uint8_t ret = subghz_replay_flipper_to_tmp(sub_path);
	if (ret != 0)
		return ret;

	ret = subghz_replay_run_blocking();

	/* Temp file ownership ends with this call.  The blocking wrapper is the
	 * temp-file owner (the legacy path on which Saved-PACKET / Playlist
	 * depends).  Scene callers that want async TX must use
	 * sub_ghz_replay_prepare_flipper() + sub_ghz_replay_start_async() and
	 * unlink the temp file from their own scene state. */
	f_unlink(FLIPPER_SUB_TMP_SGH);
	return ret;
}



/*============================================================================*/
/*                                                                            */
/*  FREQUENCY ANALYZER                                                        */
/*  Momentum-style: sweep a band range each pass, find the strongest signal, */
/*  display with large frequency readout, RSSI bar, and threshold marker.    */
/*  L/R: cycle band  UP/DN: adjust threshold  OK: hold/resume  BACK: exit   */
/*                                                                            */
/*============================================================================*/

/* Bar-graph geometry */
#define FREQAN_RSSI_FLOOR   (-120)  /* dBm lower bound on bar scale */
#define FREQAN_RSSI_CEIL    (-30)   /* dBm upper bound on bar scale */
#define FREQAN_BAR_X         3      /* bar frame left edge (px) */
#define FREQAN_BAR_W        122     /* bar frame width (px) */
#define FREQAN_BAR_Y         38     /* bar frame top edge (px) */
#define FREQAN_BAR_H         8      /* bar frame height (px) */
/* Threshold defaults */
#define FREQAN_THR_DEFAULT  (-75)   /* initial threshold (dBm) */
#define FREQAN_THR_MIN     (-100)   /* minimum threshold (dBm) */
#define FREQAN_THR_MAX      (-50)   /* maximum threshold (dBm) */
#define FREQAN_THR_STEP       5     /* threshold step per key press (dBm) */

void sub_ghz_frequency_reader(void)
{
    /* Band filter table — defines which preset frequencies are scanned in each band.
     * Coarse stage iterates subghz_freq_presets[] filtered to [start_hz, end_hz].
     * Fine stage then sweeps ±FREQAN_FINE_RANGE_HZ around the coarse peak.
     * The 850 MHz boundary matches sub_ghz_set_opmode()'s CUSTOM mapping:
     * <850 MHz uses 433-style config/frontend; >=850 MHz uses 915-style. */
    static const struct {
        uint32_t            start_hz;
        uint32_t            end_hz;
        const char         *label;
        bool                use_915;   /* true: use 915-style radio config + frontend */
        S_M1_SubGHz_Band    frontend;  /* frontend switch position */
    } bands[] = {
        { 300000000UL, 386000000UL, "300-386MHz", false, SUB_GHZ_BAND_315     },
        { 387000000UL, 464000000UL, "387-464MHz", false, SUB_GHZ_BAND_433_92  },
        { 779000000UL, 849000000UL, "779-849MHz", false, SUB_GHZ_BAND_433_92  },
        { 850000000UL, 928000000UL, "850-928MHz", true,  SUB_GHZ_BAND_915     },
    };
    #define FREQAN_BAND_COUNT  4
    /* Fine-scan parameters (Momentum-style two-stage approach).
     * Using static const avoids preprocessor-scope leakage of in-function #defines. */
    static const uint32_t FREQAN_FINE_RANGE_HZ = 300000UL; /* ±300 kHz around coarse peak */
    static const uint32_t FREQAN_FINE_STEP_HZ  =  20000UL; /* 20 kHz → 31 pts total (15/side + centre) */
    /* Coarse grid step: covers inter-preset gaps so non-preset signals are detected.
     * 2 MHz step gives ~40 grid points per 80 MHz band; harmless overlap with presets. */
    static const uint32_t FREQAN_COARSE_STEP_HZ = 2000000UL;

    S_M1_Buttons_Status btn;
    S_M1_Main_Q_t       q_item;
    BaseType_t          ret;
    struct si446x_reply_GET_MODEM_STATUS_map *pstat;
    char    str[40];
    bool    running    = true;

    uint8_t  band_idx  = 1;                        /* default: 387-464 MHz (covers 433.92) */
    int16_t  threshold = FREQAN_THR_DEFAULT;
    uint32_t best_hz   = 0;
    int16_t  best_rssi = FREQAN_RSSI_FLOOR;
    bool     has_signal = false;
    bool     hold       = false;

    menu_sub_ghz_init();
    radio_init_rx_tx(SUB_GHZ_BAND_433, MODEM_MOD_TYPE_OOK, true);
    SI446x_Select_Frontend(bands[band_idx].frontend);
    radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);

    while (running)
    {
        /* ---- Sweep phase (skipped when held) ---- */
        if (!hold)
        {
            uint32_t band_lo = bands[band_idx].start_hz;
            uint32_t band_hi = bands[band_idx].end_hz;
            int16_t  sw_best = FREQAN_RSSI_FLOOR;
            uint32_t sw_freq = 0;
            bool     key_hit = false;

            /* Stage 1 (coarse): dual-source scan over the active band.
             * Part A — preset table: exact known protocol frequencies (e.g. 433.920 MHz,
             *   330.000 MHz) are always checked regardless of grid alignment.
             * Part B — coarse grid at FREQAN_COARSE_STEP_HZ: covers inter-preset gaps so
             *   signals at non-preset frequencies (e.g. 800 MHz in the 779-849 band) can
             *   still be found. Overlap with presets is harmless — a 2ms re-check costs
             *   little and the fine scan corrects the exact peak afterwards.
             * Both parts update the same sw_best / sw_freq accumulators. */
            for (int pass = 0; pass < 2 && !key_hit; pass++)
            {
                if (pass == 0)
                {
                    /* Part A: preset table */
                    for (int i = 0; i < SUBGHZ_FREQ_PRESET_COUNT && !key_hit; i++)
                    {
                        uint32_t f = subghz_freq_presets[i].freq_hz;
                        if (f < band_lo || f > band_hi) continue;

                        SI446x_Set_Frequency(f);
                        SI446x_Start_Rx(0);
                        HAL_Delay(2);

                        SI446x_Get_IntStatus(0, 0, 0);
                        pstat = SI446x_Get_ModemStatus(0x00);
                        int16_t r = (int16_t)(pstat->CURR_RSSI / 2) - MODEM_RSSI_COMP - 70;

                        if (r > sw_best) { sw_best = r; sw_freq = f; }

                        if (xQueuePeek(main_q_hdl, &q_item, 0) == pdTRUE &&
                            q_item.q_evt_type == Q_EVENT_KEYPAD)
                            key_hit = true;
                    }
                }
                else
                {
                    /* Part B: coarse grid — fills gaps between presets */
                    for (uint32_t f = band_lo; f <= band_hi && !key_hit;
                         f += FREQAN_COARSE_STEP_HZ)
                    {
                        SI446x_Set_Frequency(f);
                        SI446x_Start_Rx(0);
                        HAL_Delay(2);

                        SI446x_Get_IntStatus(0, 0, 0);
                        pstat = SI446x_Get_ModemStatus(0x00);
                        int16_t r = (int16_t)(pstat->CURR_RSSI / 2) - MODEM_RSSI_COMP - 70;

                        if (r > sw_best) { sw_best = r; sw_freq = f; }

                        if (xQueuePeek(main_q_hdl, &q_item, 0) == pdTRUE &&
                            q_item.q_evt_type == Q_EVENT_KEYPAD)
                            key_hit = true;
                    }
                }
            }

            /* Stage 2 (fine): ±300 kHz at 20 kHz step around the coarse peak.
             * Clamped to [band_lo, band_hi] so the sweep cannot wander outside
             * the selected band or across a radio-config/frontend boundary.
             * Always runs when a valid coarse peak exists — the threshold only
             * controls the buzzer / "Signal!" indicator, not frequency precision.
             * Removing the threshold gate here fixes the bug where the decimal
             * part always displayed ".000" (coarse-grid peaks are exact MHz).
             * 31 pts total (15/side + centre) × 2 ms ≈ 62 ms. */
            if (!key_hit && sw_freq > 0)
            {
                uint32_t fine_lo = (sw_freq > FREQAN_FINE_RANGE_HZ)
                                 ? sw_freq - FREQAN_FINE_RANGE_HZ : band_lo;
                uint32_t fine_hi = sw_freq + FREQAN_FINE_RANGE_HZ;
                if (fine_lo < band_lo) fine_lo = band_lo;
                if (fine_hi > band_hi) fine_hi = band_hi;

                int16_t  fine_best = FREQAN_RSSI_FLOOR;
                uint32_t fine_freq = sw_freq;

                for (uint32_t f = fine_lo; f <= fine_hi; f += FREQAN_FINE_STEP_HZ)
                {
                    SI446x_Set_Frequency(f);
                    SI446x_Start_Rx(0);
                    HAL_Delay(2);

                    SI446x_Get_IntStatus(0, 0, 0);
                    pstat = SI446x_Get_ModemStatus(0x00);
                    int16_t r = (int16_t)(pstat->CURR_RSSI / 2) - MODEM_RSSI_COMP - 70;

                    if (r > fine_best) { fine_best = r; fine_freq = f; }

                    if (xQueuePeek(main_q_hdl, &q_item, 0) == pdTRUE &&
                        q_item.q_evt_type == Q_EVENT_KEYPAD)
                        { key_hit = true; break; }
                }

                best_hz   = fine_freq;
                best_rssi = fine_best;
            }
            else
            {
                best_hz   = sw_freq;
                best_rssi = sw_best;
            }

            has_signal = (best_rssi >= threshold);
            if (has_signal) m1_buzzer_notification();
        }

        /* ---- Draw phase ---- */
        /* Bar fill and threshold-marker pixel positions */
        int16_t rc = best_rssi;
        if (rc < FREQAN_RSSI_FLOOR) rc = FREQAN_RSSI_FLOOR;
        if (rc > FREQAN_RSSI_CEIL)  rc = FREQAN_RSSI_CEIL;
        uint8_t bar_fill = (uint8_t)(
            ((int32_t)(rc - FREQAN_RSSI_FLOOR) * (FREQAN_BAR_W - 2)) /
            (FREQAN_RSSI_CEIL - FREQAN_RSSI_FLOOR));
        if (bar_fill > FREQAN_BAR_W - 2) bar_fill = FREQAN_BAR_W - 2;

        int16_t tc = threshold;
        if (tc < FREQAN_RSSI_FLOOR) tc = FREQAN_RSSI_FLOOR;
        if (tc > FREQAN_RSSI_CEIL)  tc = FREQAN_RSSI_CEIL;
        uint8_t thr_x = (uint8_t)(FREQAN_BAR_X +
            ((int32_t)(tc - FREQAN_RSSI_FLOOR) * (FREQAN_BAR_W - 1)) /
            (FREQAN_RSSI_CEIL - FREQAN_RSSI_FLOOR));

        m1_u8g2_firstpage();
        do {
            /* Title row */
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 0, 9,
                         hold ? "Freq Analyzer [HOLD]" : "Frequency Analyzer");
            u8g2_DrawHLine(&m1_u8g2, 0, 11, 128);

            /* Best frequency — large font */
            u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_1B);
            if (best_hz > 0)
                snprintf(str, sizeof(str), "%lu.%03lu MHz",
                         best_hz / 1000000UL,
                         (best_hz % 1000000UL) / 1000UL);
            else
                snprintf(str, sizeof(str), "---.--- MHz");
            u8g2_DrawStr(&m1_u8g2, 0, 25, str);

            /* RSSI reading + optional signal indicator */
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            snprintf(str, sizeof(str), "%d dBm%s",
                     (int)best_rssi, has_signal ? "  Signal!" : "");
            u8g2_DrawStr(&m1_u8g2, 0, 35, str);

            /* Bar: outline frame */
            u8g2_DrawFrame(&m1_u8g2, FREQAN_BAR_X, FREQAN_BAR_Y,
                           FREQAN_BAR_W, FREQAN_BAR_H);

            /* Bar: fill proportional to RSSI */
            if (bar_fill > 0)
                u8g2_DrawBox(&m1_u8g2,
                             FREQAN_BAR_X + 1, FREQAN_BAR_Y + 1,
                             bar_fill, FREQAN_BAR_H - 2);

            /* Threshold marker: vertical line through bar */
            u8g2_DrawVLine(&m1_u8g2, thr_x,
                           FREQAN_BAR_Y - 2, FREQAN_BAR_H + 4);

            /* Threshold value and active band label */
            snprintf(str, sizeof(str), "Thr:%d  %s",
                     (int)threshold, bands[band_idx].label);
            u8g2_DrawStr(&m1_u8g2, 0, 54, str);

            /* Controls hint (\x18\x19 = up/down arrow glyphs in the font) */
            u8g2_DrawStr(&m1_u8g2, 0, 63, "L/R:Band \x18\x19:Thr OK:Hold");

        } while (m1_u8g2_nextpage());

        /* ---- Input phase (50 ms timeout keeps display refreshing) ---- */
        ret = xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(50));
        if (ret != pdTRUE || q_item.q_evt_type != Q_EVENT_KEYPAD) continue;
        xQueueReceive(button_events_q_hdl, &btn, 0);

        if (btn.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK)
        {
            running = false;
        }
        else if (btn.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)
        {
            hold = !hold;
            if (!hold)
            {
                best_hz    = 0;
                best_rssi  = FREQAN_RSSI_FLOOR;
                has_signal = false;
            }
        }
        else if (btn.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            band_idx = (band_idx + 1) % FREQAN_BAND_COUNT;
            best_hz = 0; best_rssi = FREQAN_RSSI_FLOOR;
            has_signal = false; hold = false;
            radio_init_rx_tx(
                bands[band_idx].use_915 ? SUB_GHZ_BAND_915 : SUB_GHZ_BAND_433,
                MODEM_MOD_TYPE_OOK, true);
            SI446x_Select_Frontend(bands[band_idx].frontend);
            radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);
        }
        else if (btn.event[BUTTON_LEFT_KP_ID] == BUTTON_EVENT_CLICK)
        {
            band_idx = (band_idx + FREQAN_BAND_COUNT - 1) % FREQAN_BAND_COUNT;
            best_hz = 0; best_rssi = FREQAN_RSSI_FLOOR;
            has_signal = false; hold = false;
            radio_init_rx_tx(
                bands[band_idx].use_915 ? SUB_GHZ_BAND_915 : SUB_GHZ_BAND_433,
                MODEM_MOD_TYPE_OOK, true);
            SI446x_Select_Frontend(bands[band_idx].frontend);
            radio_set_antenna_mode(RADIO_ANTENNA_MODE_RX);
        }
        else if (btn.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (threshold < FREQAN_THR_MAX) threshold += FREQAN_THR_STEP;
        }
        else if (btn.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK)
        {
            if (threshold > FREQAN_THR_MIN) threshold -= FREQAN_THR_STEP;
        }
    } /* while (running) */

    radio_set_antenna_mode(RADIO_ANTENNA_MODE_ISOLATED);
    SI446x_Change_State(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
    menu_sub_ghz_exit();
    xQueueReset(main_q_hdl);
    m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);

} /* sub_ghz_frequency_reader() */


/*============================================================================*/


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

	/* Disable the Update interrupt during RX.  TIM1 is shared between RX
	 * (input capture on CH1) and TX (PWM on CH4N).  The Update vector
	 * (TIM1_UP_IRQHandler) contains TX-specific DMA / CCR4 toggling.
	 * Firing it during RX is harmless on first glance, but if
	 * timerhdl_subghz_tx.Instance has not yet been initialised the handler
	 * dereferences a NULL pointer, and even when it is valid the 65 ms
	 * overflow tick is wasted work.  Only the CC1 interrupt is needed for
	 * input capture. */
	__HAL_TIM_URS_ENABLE(&timerhdl_subghz_rx);
	__HAL_TIM_CLEAR_FLAG( &timerhdl_subghz_rx, TIM_FLAG_UPDATE);
	__HAL_TIM_DISABLE_IT( &timerhdl_subghz_rx, TIM_IT_UPDATE);

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
	/* Guard: only start if the channel is in READY state.
	 * Starting from RESET (TIM1 clock off, happens when called before
	 * sub_ghz_rx_init) or BUSY (already running) returns HAL_ERROR which
	 * triggers Error_Handler and a hard crash.  The RESET case occurs in
	 * resume_rx() when hopping is active: subghz_retune_freq_hz_ext() is
	 * called before sub_ghz_rx_init_ext(), so the TIM1 clock is still
	 * disabled from the prior stop_rx() → sub_ghz_rx_deinit() call. */
	if (HAL_TIM_GetChannelState(&timerhdl_subghz_rx, SUBGHZ_RX_TIMER_RX_CHANNEL)
	        != HAL_TIM_CHANNEL_STATE_READY)
		return;

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
	/* Guard: only pause if the channel is actively running (BUSY).
	 * HAL_TIM_IC_Stop_IT accesses TIM1 registers; if the TIM1 clock is
	 * disabled (after sub_ghz_rx_deinit) any register access causes a
	 * HardFault.  The RESET/READY cases are safe to skip — the timer is
	 * either not initialised or already stopped. */
	if (HAL_TIM_GetChannelState(&timerhdl_subghz_rx, SUBGHZ_RX_TIMER_RX_CHANNEL)
	        != HAL_TIM_CHANNEL_STATE_BUSY)
		return;

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
	/* Guard: release any stale buffers from a prior operation that was not
	 * cleanly deinit'd.  Without this, calling init() twice in a row leaks
	 * the first allocation and attempts a second malloc on a fragmented heap,
	 * which is the mechanism behind the heap-exhaustion failure path described
	 * in the original bug report.  deinit() is a no-op when pointers are NULL. */
	sub_ghz_ring_buffers_deinit();

	subghz_front_buffer_size = SUBGHZ_RAW_DATA_SAMPLES_MAX;
	uint16_t next_front_buffer_size;

	while ( subghz_front_buffer_size >= 256 )
	{
		/* Allocate from the FreeRTOS heap (pvPortMalloc / vPortFree).
		 * The newlib heap (_sbrk-backed) is much smaller and is shared with
		 * the SD-manager write buffer and other C-library allocations.  The
		 * FreeRTOS heap has ample free space (typically >180 KB) for the
		 * large front-buffer without starving those other allocators.
		 * NOTE: the base pointer is over-allocated by SUBGHZ_DMA_ALIGN-1 so
		 * the aligned subghz_front_buffer pointer stays within the allocation;
		 * always free subghz_front_buffer_base, never the aligned pointer. */
		subghz_front_buffer_base = pvPortMalloc(subghz_front_buffer_size*sizeof(uint16_t) + SUBGHZ_DMA_ALIGN - 1);
		if ( !subghz_front_buffer_base )
			goto retry_smaller_capture_buffer;
		subghz_front_buffer = (uint16_t *)(((uintptr_t)subghz_front_buffer_base + SUBGHZ_DMA_ALIGN - 1) & ~(uintptr_t)(SUBGHZ_DMA_ALIGN - 1));
		subghz_ring_read_buffer = pvPortMalloc(SUBGHZ_RAW_DATA_SAMPLES_TO_RW*2); // Each sample has a 2-byte value
		if ( !subghz_ring_read_buffer )
			goto retry_smaller_capture_buffer;
		subghz_sdcard_write_buffer = pvPortMalloc(SUBGHZ_FORTMATTED_DATA_SAMPLES_TO_RW);
		if ( !subghz_sdcard_write_buffer )
			goto retry_smaller_capture_buffer;
		/* The SD manager still needs heap for its write-buffer reserve after
		 * these capture buffers are allocated. Probe that reserve now so Read
		 * Raw does not accept a capture buffer size that makes SD startup fail
		 * in m1_sdm_memory_init().  Probe with pvPortMalloc/vPortFree so the
		 * reserve exercises the *same* FreeRTOS heap-4 allocator the SD write
		 * buffer now uses (see m1_sdm_memory_init, issue #610). */
		if ( !subghz_raw_capture_reserve_heap((size_t)M1_SDM_MIN_BUFFER_SIZE*M1_SDM_BUFFER_ARRAY_SIZE, pvPortMalloc, vPortFree) )
			goto retry_smaller_capture_buffer;
		m1_ringbuffer_init(&subghz_rx_rawdata_rb, (uint8_t *)subghz_front_buffer, subghz_front_buffer_size, sizeof(uint16_t));

		M1_LOG_I(M1_LOGDB_TAG, "sub_ghz_ring_buffers_init %d\r\n", subghz_front_buffer_size);

		return 0;

retry_smaller_capture_buffer:
		next_front_buffer_size = subghz_front_buffer_size / 2;
		sub_ghz_ring_buffers_deinit();
		if ( next_front_buffer_size < 256 )
			break;
		subghz_front_buffer_size = next_front_buffer_size;
	} // while ( subghz_front_buffer_size >= 256 )

	sub_ghz_ring_buffers_deinit();
	M1_LOG_I(M1_LOGDB_TAG, "sub_ghz_ring_buffers_init FAILED freertos_heap_free=%lu\r\n",
	         (unsigned long)xPortGetFreeHeapSize());
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

	/* Timer Clock: 1MHz (1us per tick) — matches sample durations in .sgh files */
	tim_prescaler_val = (uint32_t) (HAL_RCC_GetPCLK2Freq() / 1000000) - 1;

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
		/* Must use vPortFree — allocated with pvPortMalloc in ring_buffers_init */
		vPortFree(subghz_front_buffer_base);
		subghz_front_buffer = NULL;
		subghz_front_buffer_base = NULL;
		subghz_front_buffer_size = 0;
	} // if ( subghz_front_buffer )

	if ( subghz_ring_read_buffer )
	{
		vPortFree(subghz_ring_read_buffer);
		subghz_ring_read_buffer = NULL;
	} // if ( subghz_ring_read_buffer )

	if ( subghz_sdcard_write_buffer )
	{
		vPortFree(subghz_sdcard_write_buffer);
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
static void sub_ghz_tx_raw_deinit_impl(bool reset_main_q)
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
		__HAL_TIM_DISABLE_IT(&timerhdl_subghz_tx, TIM_IT_UPDATE);
	} // if ( timerhdl_subghz_tx.Instance != NULL )

	SUBGHZ_TX_TIMER_CLK_DIS();
	HAL_NVIC_DisableIRQ(SUBGHZ_TX_TIMER_IRQn);

	//SI446x_Set_Tx_Power(12);
	sub_ghz_set_opmode(SUB_GHZ_OPMODE_ISOLATED, SUB_GHZ_BAND_EOL, 0, 0);

	if (reset_main_q && main_q_hdl != NULL)
		xQueueReset(main_q_hdl);
}

static void sub_ghz_tx_raw_deinit(void)
{
	sub_ghz_tx_raw_deinit_impl(true);
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

	/* Guard: release any stale buffers and attempt to clean up file state
	 * from a prior call that was not cleanly deinit'd. */
	sub_ghz_raw_samples_deinit(false);

	do
	{
		error = m1_sdm_get_logging_error();
		if ( error )
			break;
		/* Allocate from the FreeRTOS heap (pvPortMalloc / vPortFree).
		 * The newlib heap (_sbrk-backed) was shrunk significantly by the
		 * SiN360 binary-SPI BSS expansion and can no longer reliably hold
		 * these buffers, even on a fresh boot.  Mirrors the v0.9.1.14 fix
		 * applied to sub_ghz_ring_buffers_init() for Read Raw / Record. */
		sdcard_dat_buffer_base = pvPortMalloc(M1_SDM_MIN_BUFFER_SIZE);
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
			/* NOTE: the base pointer is over-allocated by SUBGHZ_DMA_ALIGN-1 so
			 * the aligned subghz_back_buffer pointer stays within the allocation;
			 * always free subghz_back_buffer_base, never the aligned pointer. */
			subghz_back_buffer_base = pvPortMalloc(subghz_back_buffer_size*sizeof(uint16_t) + SUBGHZ_DMA_ALIGN - 1);
			if ( subghz_back_buffer_base )
			{
				subghz_back_buffer = (uint16_t *)(((uintptr_t)subghz_back_buffer_base + SUBGHZ_DMA_ALIGN - 1) & ~(uintptr_t)(SUBGHZ_DMA_ALIGN - 1));
				break;
			}
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

		psdcard_dat_buffer = pvPortMalloc(sdcard_dat_read_size + 1);
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
			/* Advance past this keyword line in the original buffer.
			 * Scan past the token text, then skip any CR/LF characters so
			 * both CRLF (Windows/M1) and LF-only (Unix/C3.12) files work.
			 * The old "strlen(token) + 2" hard-coded CRLF and mis-positioned
			 * sdcard_buffer_run_ptr for LF-only files, corrupting data reads. */
			sdcard_buffer_run_ptr += strlen(token);
			while (*sdcard_buffer_run_ptr == '\r' || *sdcard_buffer_run_ptr == '\n')
				sdcard_buffer_run_ptr++;
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
		vPortFree(psdcard_dat_buffer);

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
		/* Must use vPortFree — allocated with pvPortMalloc in raw_samples_init */
		vPortFree(subghz_back_buffer_base);
		subghz_back_buffer = NULL;
		subghz_back_buffer_base = NULL;
	}
	if ( sdcard_dat_buffer_base )
	{
		vPortFree(sdcard_dat_buffer_base);
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
							{
								double_buffer_ptr[buffer_ptr_id][raw_samples_count++] = number;
							}
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

	// Save these data for repeat
	subghz_decenc_ctl.ntx_raw_len = (len<<1); // Convert length to byte
	subghz_decenc_ctl.ntx_raw_src = source;
	subghz_decenc_ctl.ntx_raw_dest = dest;
	subghz_decenc_ctl.ntx_raw_repeat = repeat;
	subghz_tx_tc_flag = 0;

	/* Enable the DMA channel */
	HAL_StatusTypeDef ret = TIM_DMA_Start_IT(timerhdl_subghz_tx.hdma[TIM_DMA_ID_UPDATE], source, dest, subghz_decenc_ctl.ntx_raw_len);
	if ( ret != HAL_OK)
		return;

	timerhdl_subghz_tx.Instance->CCR4 = 0; // initial value

	// Generate Update Event (set UG bit) to reload the DMA source data[0] to the ARR register
	HAL_TIM_GenerateEvent(&timerhdl_subghz_tx, TIM_EVENTSOURCE_UPDATE);
	// Do it again to reload the DMA source data[1] to the ARR register, and reload the DMA source data[0] to the ARR shadow register
	HAL_TIM_GenerateEvent(&timerhdl_subghz_tx, TIM_EVENTSOURCE_UPDATE);

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
		subghz_decenc_ctl.ntx_raw_len = (len<<1); // Convert length to byte
		subghz_decenc_ctl.ntx_raw_src = source;
	} // if ( (source!=0) && (len!=0) )

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

	timerhdl_subghz_tx.Instance->CCR4 = 0; // initial value

	// Generate Update Event (set UG bit) to reload the DMA source data[0] to the ARR register
	HAL_TIM_GenerateEvent(&timerhdl_subghz_tx, TIM_EVENTSOURCE_UPDATE);
	// Do it again to reload the DMA source data[1] to the ARR register, and reload the DMA source data[0] to the ARR shadow register
	HAL_TIM_GenerateEvent(&timerhdl_subghz_tx, TIM_EVENTSOURCE_UPDATE);

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
	/* Flipper RAW_Data format: positive values for HIGH periods, negative for LOW.
	 * sign_text[0] = no sign (HIGH = first edge after rising-edge sync),
	 * sign_text[1] = "-"   (LOW  = second edge). */
	char *sign_text[2] = {"", "-"};
	uint8_t *pfillbuffer;
	uint8_t sign;

	prn_buffer = pvPortMalloc(64);
	if (prn_buffer == NULL)
		return 1;
	pfillbuffer = subghz_sdcard_write_buffer;
	if ( header_init )
	{
		/* Write a Flipper-compatible RAW file header so the capture can be
		 * opened directly by Magellan and other Flipper-ecosystem tools. */
		if (subghz_scan_config.band == SUB_GHZ_BAND_CUSTOM)
			freq32 = subghz_custom_freq_hz;
		else
			freq32 = subghz_band_steps[subghz_scan_config.band][0]*1000000;

		/* Map M1 modulation to the closest Flipper preset string */
		const char *preset;
		switch (subghz_scan_config.modulation)
		{
			case MODULATION_FSK:
				preset = "FuriHalSubGhzPreset2FSKDev47_6KhzAsync";
				break;
			default: /* OOK, ASK */
				preset = "FuriHalSubGhzPresetOok650Async";
				break;
		}

		sprintf((char *)pfillbuffer, "Filetype: " FLIPPER_SUBGHZ_RAW_FILETYPE "\r\nVersion: 1\r\n");
		sprintf(prn_buffer, "Frequency: %lu\r\n", freq32);
		strcat((char *)pfillbuffer, prn_buffer);
		sprintf(prn_buffer, "Preset: %s\r\n", preset);
		strcat((char *)pfillbuffer, prn_buffer);
		strcat((char *)pfillbuffer, "Protocol: RAW\r\n");
		m1_sdm_fill_buffer(pfillbuffer, strlen((char *)pfillbuffer));
		vPortFree(prn_buffer);
		return 0;
	} // if ( header_init )

	/* Data lines use Flipper's "RAW_Data:" keyword and signed integers.
	 * This format is explicitly handled by the M1 raw-data parser via
	 * strncmp("RAW_Data:", 9) in sub_ghz_rx_raw_save(). */
	sprintf((char *)pfillbuffer, "RAW_Data:");
	n_samples_to_rw = SUBGHZ_RAW_DATA_SAMPLES_TO_RW;
	if ( last_data )
	{
		n_samples_to_rw = ringbuffer_get_data_slots(&subghz_rx_rawdata_rb);
		if ( n_samples_to_rw > SUBGHZ_RAW_DATA_SAMPLES_TO_RW ) // This should never happen!
			n_samples_to_rw = SUBGHZ_RAW_DATA_SAMPLES_TO_RW;
	} // if ( last_data )
	m1_ringbuffer_read(&subghz_rx_rawdata_rb, subghz_ring_read_buffer, n_samples_to_rw);
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
		strcat((char *)pfillbuffer, prn_buffer);
		pdata++;
		sign ^= 1;
	}
	strcat((char *)pfillbuffer, "\r\n");

	m1_sdm_fill_buffer(pfillbuffer, strlen((char *)pfillbuffer));

	if ( prn_buffer!=NULL )
		vPortFree(prn_buffer);
	return 0;
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
		/* Return 1 immediately so callers can distinguish ISM-blocked from
		 * other errors.  The bottom-of-function remap (ret_code=1 → 0) is
		 * intentionally bypassed here. */
		m1_buzzer_notification();
		m1_message_box(&m1_u8g2, "TX Blocked:", "Region restricts", "this frequency.", "Set Region to Off");
		return 1;
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

	/* Region "Off" — no filtering, allow all frequencies */
	if (subghz_regions_list[m1_device_stat.config.ism_band_region].bands_list == 0)
		return 0;

	if (band == SUB_GHZ_BAND_CUSTOM)
		freq = (float)subghz_custom_freq_hz / 1000000.0f;
	else if (band < SUB_GHZ_BAND_EOL)
		freq = subghz_band_steps[band][0] + channel * CHANNEL_STEP;
	else
		return 1; /* unknown band — reject */

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
#define SPECTRUM_MIN_FREQ   SUBGHZ_MIN_FREQ_HZ /* Si4463 + frontend lower limit */
#define SPECTRUM_MAX_FREQ   SUBGHZ_MAX_FREQ_HZ /* Si4463 upper limit */

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
    };
    static const uint32_t sweep_spans[] = {
        15000000UL,
        50000000UL,
        10000000UL,
        10000000UL,
    };
    #define NUM_SWEEP_RANGES  4

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

        /* Clamp center_freq to the supported hardware range so zoom-out/pan
         * in custom_view cannot sweep into frequencies the Si4463 and antenna
         * frontend cannot handle (<300 MHz or >928 MHz). */
        {
            uint32_t half = span / 2;
            if (center_freq < SPECTRUM_MIN_FREQ + half)
                center_freq = SPECTRUM_MIN_FREQ + half;
            if (center_freq > SPECTRUM_MAX_FREQ - half)
                center_freq = SPECTRUM_MAX_FREQ - half;
        }

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
        m1_u8g2_firstpage();
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

        } while (m1_u8g2_nextpage());

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
    m1_u8g2_firstpage();
    do {
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 2, 12, "Weather Station");
        u8g2_DrawStr(&m1_u8g2, 2, 28, "Listening 433.92MHz...");
        u8g2_DrawStr(&m1_u8g2, 2, 56, "Press BACK to exit");
    } while (m1_u8g2_nextpage());

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

                m1_u8g2_firstpage();
                do {
                    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
                    u8g2_DrawStr(&m1_u8g2, 2, 12, "Weather Station");
                    u8g2_DrawStr(&m1_u8g2, 2, 24, line1);
                    u8g2_SetFont(&m1_u8g2, M1_DISP_RUN_MENU_FONT_B);
                    u8g2_DrawStr(&m1_u8g2, 2, 38, line2);
                    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
                    u8g2_DrawStr(&m1_u8g2, 2, 50, line3);
                    u8g2_DrawStr(&m1_u8g2, 2, 62, "BACK to exit");
                } while (m1_u8g2_nextpage());

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

    /* Protocol selection — defaults.
     * SECPLUS_PROTO_SENTINEL (0xFF) marks the SecPlus v1 counter slot — it
     * is NOT used as a registry index; the TX loop treats it specially. */
    static const uint8_t SECPLUS_PROTO_SENTINEL = 0xFF;
    uint8_t proto_idx = 0;
    static const uint8_t brute_protos[] = {
        PRINCETON, CAME_12BIT, NICE_FLO, LINEAR_10BIT, HOLTEK_HT12E,
        0xFF   /* SecPlus v1 counter — sentinel, not a registry index */
    };
    static const char *brute_names[] = {
        "Princeton", "CAME", "Nice FLO", "Linear", "Holtek",
        "SecPlus v1 Ctr"
    };
    static const uint8_t brute_bits[] = { 24, 12, 12, 10, 12, 32 };
    /* Radio band to use for TX — must match the protocol's operating frequency */
    static const S_M1_SubGHz_Band brute_bands[] = {
        SUB_GHZ_BAND_433_92,  /* Princeton */
        SUB_GHZ_BAND_433_92,  /* CAME */
        SUB_GHZ_BAND_433_92,  /* Nice FLO */
        SUB_GHZ_BAND_300,     /* Linear — 300 MHz */
        SUB_GHZ_BAND_433_92,  /* Holtek */
        SUB_GHZ_BAND_315,     /* SecPlus v1 — 315 MHz (Chamberlain/LiftMaster) */
    };
    enum {
        NUM_BRUTE_PROTOS = sizeof(brute_protos) / sizeof(brute_protos[0])
    };
    _Static_assert(
        (sizeof(brute_names) / sizeof(brute_names[0])) == NUM_BRUTE_PROTOS,
        "brute_names length must match brute_protos");
    _Static_assert(
        (sizeof(brute_bits) / sizeof(brute_bits[0])) == NUM_BRUTE_PROTOS,
        "brute_bits length must match brute_protos");
    _Static_assert(
        (sizeof(brute_bands) / sizeof(brute_bands[0])) == NUM_BRUTE_PROTOS,
        "brute_bands length must match brute_protos");

    uint32_t code = 0;
    uint32_t max_code;
    uint16_t pulse_buf[256];
    uint16_t pulse_count;
    uint8_t  state = 0;  /* 0=select protocol, 1=running, 2=done */
    uint16_t te_short, te_long;
    bool is_secplus = false;  /* true when SecPlus v1 counter mode is active */

    menu_sub_ghz_init();

    /* Select protocol screen */
    while (running && state == 0)
    {
        bool secplus_sel = (brute_protos[proto_idx] == SECPLUS_PROTO_SENTINEL);
        m1_u8g2_firstpage();
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 12, "Brute Force");
            snprintf(line1, sizeof(line1), "> %s (%d bit)",
                     brute_names[proto_idx], brute_bits[proto_idx]);
            u8g2_DrawStr(&m1_u8g2, 2, 28, line1);
            if (secplus_sel) {
                u8g2_DrawStr(&m1_u8g2, 2, 40, "Ctr: 0->3^20-1");
                u8g2_DrawStr(&m1_u8g2, 2, 52, "Fixed:0x0 @ 315MHz");
            } else {
                max_code = (1UL << brute_bits[proto_idx]) - 1;
                snprintf(line2, sizeof(line2), "Codes: 0-%lu", max_code);
                u8g2_DrawStr(&m1_u8g2, 2, 40, line2);
                u8g2_DrawStr(&m1_u8g2, 2, 52, "UP/DN:Proto OK:Start");
            }
            u8g2_DrawStr(&m1_u8g2, 2, 62, "BACK to exit");
        } while (m1_u8g2_nextpage());

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
                is_secplus = (brute_protos[proto_idx] == SECPLUS_PROTO_SENTINEL);
                if (!is_secplus)
                {
                    max_code = (1UL << brute_bits[proto_idx]) - 1;
                    te_short = subghz_protocols_list[brute_protos[proto_idx]].te_short;
                    te_long  = subghz_protocols_list[brute_protos[proto_idx]].te_long;
                }
                else
                {
                    max_code = SUBGHZ_SECPLUS_V1_MAX_ROLLING;
                    te_short = 0;
                    te_long  = 0;
                }
            }
        }
    }

    if (state == 1)
    {
        /* Init radio for TX on the correct band for the selected protocol */
        S_M1_SubGHz_Band tx_band = brute_bands[proto_idx];
        radio_init_rx_tx(tx_band, MODEM_MOD_TYPE_OOK, true);
        SI446x_Select_Frontend(tx_band);
        radio_set_antenna_mode(RADIO_ANTENNA_MODE_TX);
    }

    /* Brute force loop */
    while (running && state == 1)
    {
        if (is_secplus)
        {
            /* SecPlus v1 counter mode: encode ternary 2-sub-packet OOK signal.
             * fixed code = 0 (broadcast/any fixed code), rolling = current code.
             * The encoded packet is loaded as raw timing data into the ring buffer
             * using the same TX path as OOK brute force. */
            SubGhzSecPlusV1Packet sp_pkt;
            uint16_t idx = 0;

            if (subghz_secplus_v1_encode(0, code, &sp_pkt))
            {
                /* Sub-packet 1: symbols[0..20] (21 symbols = SYMS_PER_PKT) */
                for (uint8_t s = 0; s < SUBGHZ_SECPLUS_V1_SYMS_PER_PKT; s++)
                {
                    /* Each symbol is a (LOW, HIGH) pair: space first, then mark */
                    if (idx + 1 < sizeof(pulse_buf) / sizeof(pulse_buf[0]))
                    {
                        /* Space (LOW) */
                        pulse_buf[idx++] = sp_pkt.symbols[s].low_us  & SUBGHZ_OTA_SPACE_BIT_MASK;
                        /* Mark (HIGH) */
                        pulse_buf[idx++] = sp_pkt.symbols[s].high_us | SUBGHZ_OTA_PULSE_BIT_MASK;
                    }
                }
                /* Inter-sub-packet gap */
                if (idx < sizeof(pulse_buf) / sizeof(pulse_buf[0]))
                    pulse_buf[idx++] = (uint16_t)(SUBGHZ_SECPLUS_V1_INTER_PKT_GAP_US & SUBGHZ_OTA_SPACE_BIT_MASK);

                /* Sub-packet 2: symbols[21..41] (21 symbols = SYMS_PER_PKT) */
                for (uint8_t s = SUBGHZ_SECPLUS_V1_SYMS_PER_PKT;
                     s < SUBGHZ_SECPLUS_V1_TOTAL_SYMS; s++)
                {
                    if (idx + 1 < sizeof(pulse_buf) / sizeof(pulse_buf[0]))
                    {
                        /* Space (LOW) */
                        pulse_buf[idx++] = sp_pkt.symbols[s].low_us  & SUBGHZ_OTA_SPACE_BIT_MASK;
                        /* Mark (HIGH) */
                        pulse_buf[idx++] = sp_pkt.symbols[s].high_us | SUBGHZ_OTA_PULSE_BIT_MASK;
                    }
                }
                /* End gap */
                if (idx < sizeof(pulse_buf) / sizeof(pulse_buf[0]))
                    pulse_buf[idx++] = (uint16_t)(SUBGHZ_SECPLUS_V1_END_GAP_US & SUBGHZ_OTA_SPACE_BIT_MASK);
            }
            pulse_count = idx;
        }
        else
        {
            /* Standard OOK PWM brute force */
            brute_force_encode_pwm(code, brute_bits[proto_idx],
                                  te_short, te_long,
                                  pulse_buf, &pulse_count);
        }

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
            m1_u8g2_firstpage();
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
            } while (m1_u8g2_nextpage());

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
        m1_u8g2_firstpage();
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 12, "Brute Force");
            u8g2_DrawStr(&m1_u8g2, 2, 28, "Complete!");
            snprintf(line1, sizeof(line1), "%lu codes sent", max_code + 1);
            u8g2_DrawStr(&m1_u8g2, 2, 44, line1);
            u8g2_DrawStr(&m1_u8g2, 2, 62, "BACK to exit");
        } while (m1_u8g2_nextpage());

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
        m1_u8g2_firstpage();
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

            /* Current dBm — value at X=0, "dBm" label anchored at X=25 */
            snprintf(info_str, sizeof(info_str), "%d", rssi);
            u8g2_DrawStr(&m1_u8g2, 0, 22, info_str);
            u8g2_DrawStr(&m1_u8g2, 25, 22, "dBm");

            /* Peak dBm — derive value/unit positions from rendered widths to avoid overlap */
            {
                const char *peak_label = "Pk:";
                const char *dbm_label = "dBm";
                const uint8_t peak_label_x = 68;
                const uint8_t peak_section_gap = 2;
                uint8_t peak_value_x;
                uint8_t peak_dbm_x;

                snprintf(info_str, sizeof(info_str), "%d", peak_rssi);
                peak_value_x = peak_label_x + u8g2_GetStrWidth(&m1_u8g2, peak_label) + peak_section_gap;
                peak_dbm_x = peak_value_x + u8g2_GetStrWidth(&m1_u8g2, info_str) + peak_section_gap;

                u8g2_DrawStr(&m1_u8g2, peak_label_x, 22, peak_label);
                u8g2_DrawStr(&m1_u8g2, peak_value_x, 22, info_str);
                u8g2_DrawStr(&m1_u8g2, peak_dbm_x, 22, dbm_label);
            }

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

        } while (m1_u8g2_nextpage());

        /* Check for button input — 150 ms timeout gives ~6 Hz refresh, readable without flicker */
        ret = xQueueReceive(main_q_hdl, &q_item, pdMS_TO_TICKS(150));
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
        m1_u8g2_firstpage();
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

        } while (m1_u8g2_nextpage());

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
} /* end sub_ghz_freq_scanner() */


/*============================================================================*/
/* Scene Manager Bridge Functions                                             */
/*                                                                            */
/* These non-static wrappers expose internal static functions to the new      */
/* scene-based UI files.  The _ext suffix avoids name collisions.             */
/*============================================================================*/

#include "m1_subghz_scene.h"

/* Frequency & modulation label arrays for scene_config.c */
const char *subghz_freq_labels[SUBGHZ_FREQ_PRESET_COUNT + 1];  /* +1 for Custom entry */
const char *subghz_mod_labels[SUBGHZ_MOD_PRESET_COUNT];

/* Hopper frequency getter for scene_read.c — reads ism_band_region at runtime */
const uint32_t *subghz_get_hopper_freqs_ext(void)
{
	return subghz_get_hopper_freqs(m1_device_stat.config.ism_band_region);
}

static bool subghz_labels_initialized = false;

static void subghz_init_labels(void)
{
	if (subghz_labels_initialized)
		return;
	for (uint8_t i = 0; i < SUBGHZ_FREQ_PRESET_COUNT; i++)
		subghz_freq_labels[i] = subghz_freq_presets[i].label;
	/* Add the "Custom" sentinel label at index 62 */
	subghz_freq_labels[SUBGHZ_FREQ_PRESET_CUSTOM] = subghz_custom_freq_label;
	for (uint8_t i = 0; i < SUBGHZ_MOD_PRESET_COUNT; i++)
		subghz_mod_labels[i] = subghz_mod_presets[i].label;
	subghz_labels_initialized = true;
}

/* RSSI read */
int16_t subghz_read_rssi_ext(void)
{
	return subghz_read_rssi();
}

/* Apply config presets to radio */
void subghz_apply_config_ext(uint8_t freq_idx, uint8_t mod_idx)
{
	subghz_cfg.freq_idx = freq_idx;
	subghz_cfg.mod_idx  = mod_idx;
	subghz_apply_config();
}

/* Protocol static check */
bool subghz_protocol_is_static_ext(uint16_t protocol)
{
	return subghz_protocol_is_static(protocol);
}

/* Transmit static signal */
bool subghz_transmit_static_signal_ext(const SubGHz_History_Entry_t *entry)
{
	return subghz_transmit_static_signal(entry);
}

/* Get frequency in Hz for a given preset index */
uint32_t subghz_get_freq_hz_ext(uint8_t freq_idx)
{
	if (freq_idx == SUBGHZ_FREQ_PRESET_CUSTOM)
		return subghz_user_custom_freq_hz;
	if (freq_idx >= SUBGHZ_FREQ_PRESET_COUNT)
		return 433920000UL;  /* safe default */
	return subghz_freq_presets[freq_idx].freq_hz;
}

/* Retune radio to an arbitrary frequency (Hz) without full RX re-init.
 * Used by hopper to change frequency on the fly during active RX.
 *
 * Lightweight path: keeps the previously-loaded radio config and modem
 * mode intact and only retunes the synthesizer + frontend antenna mux.
 * Same approach as the Frequency Analyzer / Spectrum Analyzer fast-sweep
 * code (see SI446x_Set_Frequency() call sites elsewhere in this file).
 *
 * Why we don't call sub_ghz_set_opmode() here: that function unconditionally
 * reloads the full SI4463 config from SPI on every invocation, and when the
 * target frequency falls in the 915 MHz band it forces a swap to the FSK
 * radio config.  Doing that on a 200 ms hopper tick churns hundreds of SPI
 * writes per second, repeatedly mode-swaps the modem from OOK→FSK→OOK as
 * the hopper crosses the 850 MHz boundary, and could lock up the device
 * (the hopper would visibly hang on 915 MHz and the watchdog would then
 * hard-reboot the unit). */
void subghz_retune_freq_hz_ext(uint32_t freq_hz)
{
	S_M1_SubGHz_Band fe_band;

	sub_ghz_rx_pause();

	subghz_custom_freq_hz = freq_hz;
	subghz_scan_config.band = subghz_freq_hz_to_band(freq_hz);

	/* Frontend antenna mux: pick the matching frontend for the new RF band.
	 * The radio modem config (OOK direct-mode) loaded at scene_on_enter is
	 * preserved — only synthesizer frequency + frontend change. */
	if (freq_hz >= 850000000UL)
		fe_band = SUB_GHZ_BAND_915;
	else if (freq_hz >= 390000000UL)
		fe_band = SUB_GHZ_BAND_433_92;
	else
		fe_band = SUB_GHZ_BAND_315;
	SI446x_Select_Frontend(fe_band);

	/* VCO must be off (READY state) before changing FREQ_CONTROL.
	 * Calibration happens automatically on the READY→RX transition. */
	SI446x_Change_State(SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);
	SI446x_Set_Frequency(freq_hz);
	SI446x_Start_Rx(0);

	sub_ghz_rx_start();
}

/* RAW RSSI history bridge functions */
void subghz_raw_rssi_draw_ext(void)
{
	subghz_raw_rssi_draw();
}

void subghz_raw_rssi_reset_ext(void)
{
	subghz_raw_rssi_reset();
}

void subghz_raw_rssi_push_ext(float rssi_dbm, bool trace)
{
	subghz_raw_rssi_push(rssi_dbm, trace);
}

void subghz_raw_rssi_set_current_ext(float rssi_dbm)
{
	/* Update the live RSSI cursor indicator without mutating the history
	 * buffer.  Used in Start state during silence so captured burst bars
	 * are not erased when RSSI drops back below threshold. */
	subghz_raw_rssi_current = subghz_rssi_to_u8(rssi_dbm);
}

void subghz_raw_draw_sin_ext(void)
{
	subghz_raw_draw_sin();
}

void subghz_raw_draw_frame_ext(void)
{
	subghz_raw_draw_frame();
}

void subghz_raw_sin_advance_ext(void)
{
	if (++subghz_raw_sin_idx > 62)
		subghz_raw_sin_idx = 0;
}

/* RX control bridge functions (expose static helpers to scene files) */
void sub_ghz_rx_init_ext(void)
{
	sub_ghz_rx_init();
}

void sub_ghz_rx_start_ext(void)
{
	sub_ghz_rx_start();
}

void sub_ghz_rx_pause_ext(void)
{
	sub_ghz_rx_pause();
}

void sub_ghz_rx_deinit_ext(void)
{
	sub_ghz_rx_deinit();
}

void sub_ghz_set_opmode_ext(uint8_t opmode, uint8_t band, uint8_t channel, uint8_t tx_power)
{
	sub_ghz_set_opmode(opmode, band, channel, tx_power);
}

uint8_t sub_ghz_ring_buffers_init_ext(void)
{
	return sub_ghz_ring_buffers_init();
}

void sub_ghz_ring_buffers_deinit_ext(void)
{
	sub_ghz_ring_buffers_deinit();
}

void sub_ghz_tx_raw_deinit_ext(void)
{
	sub_ghz_tx_raw_deinit();
}

uint8_t sub_ghz_rx_raw_save_ext(bool header_init, bool last_data)
{
	return sub_ghz_rx_raw_save(header_init, last_data);
}

/* TX power accessors (expose static data to scene Config) */
uint8_t subghz_get_tx_power_idx_ext(void) { return subghz_tx_power_idx; }
void    subghz_set_tx_power_idx_ext(uint8_t idx) { if (idx < TX_POWER_LEVELS) subghz_tx_power_idx = idx; }
const char *subghz_get_tx_power_label_ext(uint8_t idx) { return (idx < TX_POWER_LEVELS) ? tx_power_labels[idx] : "?"; }
uint8_t subghz_get_tx_power_count_ext(void) { return TX_POWER_LEVELS; }

/* Save format accessors (0 = Flipper .sub, 1 = M1 native .sgh) */
uint8_t subghz_get_save_fmt_ext(void) { return subghz_cfg.save_fmt; }
void    subghz_set_save_fmt_ext(uint8_t fmt) { if (fmt <= 1) subghz_cfg.save_fmt = fmt; }

/* RSSI threshold accessor (dBm, -50 to -100; -70 is the Flipper default) */
int8_t  subghz_get_rssi_threshold_ext(void)  { return subghz_cfg.rssi_threshold; }
void    subghz_set_rssi_threshold_ext(int8_t v)
{
    if (v > -50)  v = -50;
    if (v < -100) v = -100;
    subghz_cfg.rssi_threshold = v;
}

/* Radio config accessors — used by settings_save/load and scene init */
uint8_t subghz_get_freq_idx_ext(void)        { return subghz_cfg.freq_idx; }
void    subghz_set_freq_idx_ext(uint8_t idx)
{
    /* Accept both real presets (0..61) and the Custom sentinel (62) */
    if (idx <= SUBGHZ_FREQ_PRESET_CUSTOM)
        subghz_cfg.freq_idx = idx;
}

/* User custom frequency (persisted; used when freq_idx == SUBGHZ_FREQ_PRESET_CUSTOM) */
uint32_t subghz_get_user_custom_freq_ext(void)       { return subghz_user_custom_freq_hz; }
void     subghz_set_user_custom_freq_ext(uint32_t hz)
{
    /* Clamp to SI4463 operating range (300–928 MHz) */
    if (hz < SUBGHZ_MIN_FREQ_HZ)  hz = SUBGHZ_MIN_FREQ_HZ;
    if (hz > SUBGHZ_MAX_FREQ_HZ)  hz = SUBGHZ_MAX_FREQ_HZ;
    subghz_user_custom_freq_hz = hz;
    /* Refresh display label */
    snprintf(subghz_custom_freq_label, sizeof(subghz_custom_freq_label),
             "%lu.%02lu",
             (unsigned long)(hz / 1000000UL),
             (unsigned long)((hz % 1000000UL) / 10000UL));
    /* Propagate to freq_labels table if already initialised */
    if (subghz_labels_initialized)
        subghz_freq_labels[SUBGHZ_FREQ_PRESET_CUSTOM] = subghz_custom_freq_label;
}
uint8_t subghz_get_mod_idx_ext(void)         { return subghz_cfg.mod_idx; }
void    subghz_set_mod_idx_ext(uint8_t idx)  { if (idx < SUBGHZ_MOD_PRESET_COUNT) subghz_cfg.mod_idx = idx; }
bool    subghz_get_hopping_ext(void)         { return subghz_cfg.hopping; }
void    subghz_set_hopping_ext(bool v)       { subghz_cfg.hopping = v; }
bool    subghz_get_sound_ext(void)           { return subghz_cfg.sound; }
void    subghz_set_sound_ext(bool v)         { subghz_cfg.sound = v; }
bool    subghz_get_autosave_ext(void)        { return subghz_cfg.autosave; }
void    subghz_set_autosave_ext(bool v)      { subghz_cfg.autosave = v; }

/* Phase 12 — receiver history quality-of-life toggles */
bool    subghz_get_remove_duplicates_ext(void)     { return subghz_cfg.remove_duplicates; }
void    subghz_set_remove_duplicates_ext(bool v)   { subghz_cfg.remove_duplicates = v; }
bool    subghz_get_delete_old_signals_ext(void)    { return subghz_cfg.delete_old_signals; }
void    subghz_set_delete_old_signals_ext(bool v)  { subghz_cfg.delete_old_signals = v; }

/* ── Raw recording file management (used by Read Raw scene) ─────────────── */

/**
 * @brief  Initialize the SD card file for raw recording and write the header.
 *
 * Sets up datfile_info, opens a new .sub file via the SDM background task,
 * writes the Flipper-compatible header (filetype, version, frequency,
 * modulation), and resets the ring buffer.
 *
 * @return 0 on success, non-zero on error (SD card init failed)
 */
uint8_t sub_ghz_raw_recording_init_ext(void)
{
	uint8_t ret;
	static char infix[5];  /* static: datfile_info stores a pointer to this */

	datfile_info.dir_name    = SUB_GHZ_FILEPATH;
	datfile_info.file_ext    = ".sub";  /* Flipper-compatible extension */
	datfile_info.file_prefix = SUB_GHZ_FILE_PREFIX;

	if (subghz_cfg.freq_idx < SUBGHZ_FREQ_PRESET_COUNT)
		strncpy(infix, subghz_freq_presets[subghz_cfg.freq_idx].label, 3);
	else
		strncpy(infix, "cst", 3);  /* Custom frequency — no label in preset table */
	infix[3] = '\0';
	datfile_info.file_infix  = infix;
	datfile_info.file_suffix = subghz_mod_presets[subghz_cfg.mod_idx].label;

	ret = m1_sdm_file_init(&datfile_info);
	if (ret)
		return ret;  /* SD card error */

	subghz_record_total_samples = 0;

	m1_sdm_task_init();
	m1_sdm_task_start();

	/* Write header (filetype, version, frequency, modulation) */
	sub_ghz_rx_raw_save(true, false);

	/* Reset ring buffer so we start clean */
	xQueueReset(main_q_hdl);
	m1_ringbuffer_reset(&subghz_rx_rawdata_rb);

	return 0;
}

/**
 * @brief  Flush pending ring buffer data to SD and return the count.
 *
 * Called when Q_EVENT_SUBGHZ_RX arrives during raw recording.  Checks
 * how many samples are available; if >= SUBGHZ_RAW_DATA_SAMPLES_TO_RW,
 * writes a batch to the SD card.
 *
 * @return Number of samples flushed (0 if insufficient data).
 */
uint32_t sub_ghz_raw_recording_flush_ext(void)
{
	uint32_t avail = ringbuffer_get_data_slots(&subghz_rx_rawdata_rb);
	if (avail >= SUBGHZ_RAW_DATA_SAMPLES_TO_RW)
	{
		subghz_record_total_samples += SUBGHZ_RAW_DATA_SAMPLES_TO_RW;
		sub_ghz_rx_raw_save(false, false);

		/* RSSI visualization is now updated by the scene via
		 * subghz_raw_rssi_push_ext() — no pulse-to-waveform
		 * conversion needed here. */

		vTaskDelay(10);  /* Yield so SDM background task can write to SD */
		return SUBGHZ_RAW_DATA_SAMPLES_TO_RW;
	}
	return 0;
}

/**
 * @brief  Stop recording: flush remaining data and close the SD file.
 */
void sub_ghz_raw_recording_stop_ext(void)
{
	/* Flush whatever remains in the ring buffer */
	if (ringbuffer_get_data_slots(&subghz_rx_rawdata_rb) > 0)
		sub_ghz_rx_raw_save(false, true);

	m1_sdm_task_stop();
	m1_sdm_task_deinit();
}

/**
 * @brief  Return the auto-generated filename from the last recording.
 *
 * The returned pointer is valid until the next call to
 * sub_ghz_raw_recording_init_ext() or until the module exits.
 */
const char *sub_ghz_raw_recording_get_filename_ext(void)
{
	return (const char *)datfile_info.dat_filename;
}

/**
 * @brief  Return total samples saved to file during last recording.
 */
uint32_t sub_ghz_raw_recording_get_total_samples_ext(void)
{
	return subghz_record_total_samples;
}

/*============================================================================*/
/* Scene-based entry point — replaces the old menu items                      */
/*============================================================================*/

void sub_ghz_scene_entry(void)
{
	subghz_init_labels();
	subghz_scene_app_run();
	m1_app_send_q_message(main_q_hdl, Q_EVENT_MENU_EXIT);
}
