/* See COPYING.txt for license details. */

/*
*
*  m1_sub_ghz.h
*
*  M1 sub-ghz functions
*
* M1 Project
*
*/

#ifndef M1_SUB_GHZ_H_
#define M1_SUB_GHZ_H_

#include "m1_io_defs.h"
#include "m1_ring_buffer.h"
#include <stdbool.h>

#define SUBGHZ_RX_TIMER                 TIM1        /*!< Timer used for Sub-GHz decoding */
/* TIM prescaler is computed to have 1 μs as time base. TIM frequency (in MHz) / (prescaler+1) */
#define SUBGHZ_RX_TIM_PRESCALER         74           /*!< TIM prescaler */
#define SUBGHZ_RX_TIMER_CLK             __HAL_RCC_TIM1_CLK_ENABLE      /*!< Clock of the used timer */
#define SUBGHZ_RX_TIMER_CLK_DIS         __HAL_RCC_TIM1_CLK_DISABLE
#define SUBGHZ_RX_TIMER_IRQn            TIM1_CC_IRQn             /*!< IR TIM IRQ */
#define SUBGHZ_RX_TIMER_RX_CHANNEL   	TIM_CHANNEL_1            /*!< IR TIM Channel */
#define SUBGHZ_RX_TIMER_CH_ACTIV  		HAL_TIM_ACTIVE_CHANNEL_1

#define SUBGHZ_TX_CARRIER_TIMER         TIM1        /*!< Timer used for IR encoding */
#define SUBGHZ_TX_TIM_PRESCALER         149          /*!< TIM prescaler: 75MHz/150 = 500kHz = 2us/tick */
#define SUBGHZ_TX_TIMER_CLK     		__HAL_RCC_TIM1_CLK_ENABLE      /*!< Clock of the used timer */
#define SUBGHZ_TX_TIMER_CLK_DIS 		__HAL_RCC_TIM1_CLK_DISABLE
#define SUBGHZ_TX_TIMER_TX_CHANNEL   	TIM_CHANNEL_4 //CH4N
#define SUBGHZ_TX_TIMER_ENC_CH_ACTIV  	HAL_TIM_ACTIVE_CHANNEL_4
#define SUBGHZ_TX_TIMER_IRQn            TIM1_UP_IRQn

#define SUBGHZ_TX_CARRIER_FREQ_36KHZ_PERIOD		2083	// clock tick period = 1/75MHz, carrier period = 1/36KHz = 75MHz/36KHz = 2083 clock tick periods
#define SUBGHZ_TX_CARRIER_FREQ_30_KHZ			(uint32_t)30000
#define SUBGHZ_TX_CARRIER_FREQ_32_KHZ           (uint32_t)32000
#define SUBGHZ_TX_CARRIER_FREQ_36_KHZ           (uint32_t)36000
#define SUBGHZ_TX_CARRIER_FREQ_38_KHZ           (uint32_t)38000
#define SUBGHZ_TX_CARRIER_FREQ_40_KHZ           (uint32_t)40000
#define SUBGHZ_TX_CARRIER_FREQ_56_KHZ           (uint32_t)56000
#define SUBGHZ_TX_CARRIER_FREQ_455_KHZ          (uint32_t)455000

#define SUBGHZ_TX_CARRIER_PRESCALE_FACTOR		10

#define SUBGHZ_RAW_DATA_SAMPLES_TO_RW			512
#define SUBGHZ_FORTMATTED_DATA_SAMPLES_TO_RW    3840//5120//2560//3072//3840

#define SUBGHZ_MODULATION_LIST					3

// Defines for ISM bands regions
#define SUBGHZ_ISM_BAND_REGIONS_LIST			4

#define SUBGHZ_ISM_BAND_REGION_NORTH_AMERICA	0
#define SUBGHZ_ISM_BAND_REGION_EUROPE_REGION_1	1
#define SUBGHZ_ISM_BAND_REGION_ASIA				2

#define SUBGHZ_ISM_BAND_REGION					SUBGHZ_ISM_BAND_REGION_NORTH_AMERICA
// End - Defines for ISM bands regions

/* SI4463 operating range — single source of truth for all frequency validators */
#define SUBGHZ_MIN_FREQ_HZ    300000000UL   /* 300.000 MHz — PLL lower bound with outdiv=12 */
#define SUBGHZ_MAX_FREQ_HZ    928000000UL   /* 928.000 MHz — ISM upper edge */

// SUBGHZ_GPIO_0(RX)	PORTE.9	<--> TIM1_CH1
// SUBGHZ_GPIO_2(TX) 	PORTD.5	<--> TIM1_CH4N
#define SUBGHZ_RX_GPIO_PORT         SI4463_GPIO0_GPIO_Port
#define SUBGHZ_RX_GPIO_PIN         	SI4463_GPIO0_Pin
#define SUBGHZ_RX_GPIO_PORT_CLK		__HAL_RCC_GPIOE_CLK_ENABLE

#define SUBGHZ_TX_GPIO_PORT         SI4463_GPIO2_GPIO_Port
#define SUBGHZ_TX_GPIO_PIN         	SI4463_GPIO2_Pin
#define SUBGHZ_TX_GPIO_PORT_CLK		__HAL_RCC_GPIOD_CLK_ENABLE

#define SUBGHZ_GPIO_AF_TX          	GPIO_AF1_TIM1
#define SUBGHZ_GPIO_AF_RX         	GPIO_AF1_TIM1

#define TIM_FORCED_ACTIVE      		((uint16_t)0x0050)
#define TIM_FORCED_INACTIVE    		((uint16_t)0x0040)

#define SUBGHZ_OTA_PULSE_BIT_MASK	0x0001 // LSB bit = 1 for Mark, using OR operator
#define SUBGHZ_OTA_SPACE_BIT_MASK	0xFFFE // LSB bit = 0 for Space, using AND operator

/* RX timer period: use full 16-bit range (65535) so the unsigned
 * subtraction  (current_CCR − previous_CCR) & 0xFFFF  yields the
 * correct edge-to-edge duration for gaps up to 65.5 ms.  The old
 * value of 20000 caused aliasing for inter-packet gaps near multiples
 * of 20 ms.  TX timeout is kept separate. */
#define SUBGHZ_RX_TIMEOUT_TIME      0xFFFF
#define SUBGHZ_TX_TIMEOUT_TIME      20000

// RF_Input_Level_dBm = (RSSI_value / 2) – MODEM_RSSI_COMP – 70
#define MODEM_RSSI_COMP 	0x40 // = 64d is appropriate for most applications.

typedef enum
{
	SUB_GHZ_BAND_300 = 0,
	SUB_GHZ_BAND_310,
	SUB_GHZ_BAND_315,
	SUB_GHZ_BAND_345,
	SUB_GHZ_BAND_372,
	SUB_GHZ_BAND_390,
	SUB_GHZ_BAND_433,
	SUB_GHZ_BAND_433_92,
	SUB_GHZ_BAND_915,
	SUB_GHZ_BAND_EOL,
	/* Arbitrary frequency via SI446x_Set_Frequency() */
	SUB_GHZ_BAND_CUSTOM = 0x80
} S_M1_SubGHz_Band;

typedef enum
{
	SUB_GHZ_OPMODE_ISOLATED = 0,
	SUB_GHZ_OPMODE_RX,
	SUB_GHZ_OPMODE_TX,
	SUB_GHZ_OPMODE_EOL
} S_M1_SubGHz_OpMode;

typedef enum {
	PULSE_DET_NORMAL = 0,
	PULSE_DET_IDLE,
	PULSE_DET_ACTIVE, // Ready and waiting for a valid frame gap to continue
	PULSE_DET_EOP,	// End-of-packet condition met
	PULSE_DET_RISING,
	PULSE_DET_FALLING,
	PULSE_DET_UNKNOWN
} S_M1_SubGHz_Pulse_Det;

typedef enum {
	MODULATION_OOK = 0,
	MODULATION_ASK,
	MODULATION_FSK,
	MODULATION_UNKNOWN
} S_M1_SubGHz_Modulation;

typedef struct
{
	uint8_t channel;
	S_M1_SubGHz_Band band;
	S_M1_SubGHz_Modulation modulation;
} S_M1_SubGHz_Scan_Config;

void menu_sub_ghz_init(void);
void menu_sub_ghz_exit(void);

void sub_ghz_init(void);
void sub_ghz_frequency_reader(void);
void sub_ghz_spectrum_analyzer(void);
void sub_ghz_weather_station(void);
void sub_ghz_brute_force(void);
void sub_ghz_rssi_meter(void);
void sub_ghz_freq_scanner(void);
uint8_t sub_ghz_replay_flipper_file(const char *sub_path);

/**
 * @brief  Replay an M1 native .sgh NOISE file directly without conversion.
 *
 * Bypasses the temp-file conversion path used by sub_ghz_replay_flipper_file()
 * and feeds the original .sgh file straight into the streaming engine.
 * Use this whenever flipper_subghz_signal_t::is_m1_native is true and the
 * type is FLIPPER_SUBGHZ_TYPE_RAW.
 *
 * @param  sgh_path   Path to the .sgh NOISE file (e.g. "/SUBGHZ/foo.sgh")
 * @param  frequency  Carrier frequency in Hz (from the loaded signal metadata)
 * @param  modulation MODULATION_OOK / MODULATION_FSK / MODULATION_ASK
 * @retval 0 = success, non-zero = error code
 */
uint8_t sub_ghz_replay_datafile(const char *sgh_path, uint32_t frequency, uint8_t modulation);

/* ========================================================================== *
 * Async TX replay primitives (Phase 1 of #384).
 *
 * Use these from a scene's on_event handler to run TX without blocking the
 * main task.  The Q_EVENT_SUBGHZ_TX consumer must be the scene's own event
 * handler — it calls sub_ghz_replay_continue_async() on every completion event.
 *
 * Typical scene usage:
 *
 *   1. prepare_datafile()  OR  prepare_flipper(&tmp_path)  to set globals.
 *      If prepare_flipper() succeeded, store *tmp_path in scene state so the
 *      scene can f_unlink() the temp file after the TX completes or is aborted.
 *   2. sub_ghz_replay_start_async()   — arms TIM1+DMA and returns immediately.
 *   3. On every Q_EVENT_SUBGHZ_TX received in the scene's on_event:
 *        sub_ghz_replay_continue_async(false)
 *      Returning SUBGHZ_REPLAY_ASYNC_DONE or _ERROR means teardown completed
 *      internally — the scene transitions back to its pre-TX state.
 *   4. On BACK (or scene exit) during TX:
 *        sub_ghz_replay_abort()
 *      Synchronous teardown.  A late Q_EVENT_SUBGHZ_TX arriving after this
 *      call is safe — continue_async() returns DONE immediately when no replay
 *      is active.
 *
 * Blocking wrappers (sub_ghz_replay_flipper_file / sub_ghz_replay_datafile)
 * are reimplemented on top of these primitives plus a private mini event
 * loop, so the Saved-PACKET emulate path and Playlist behaviour are
 * unchanged.
 * ========================================================================== */

/** Async TX status returned by sub_ghz_replay_continue_async(). */
typedef enum {
    SUBGHZ_REPLAY_ASYNC_RUNNING = 0, /**< TX still in progress — more events pending */
    SUBGHZ_REPLAY_ASYNC_DONE,        /**< TX completed cleanly — teardown done */
    SUBGHZ_REPLAY_ASYNC_ERROR,       /**< TX failed — teardown done */
} sub_ghz_replay_async_status_t;

/**
 * @brief  Prepare globals for an M1 native .sgh stream.  Validates inputs and
 *         sets the replay frequency/band/modulation globals plus
 *         datfile_info.dat_filename so a subsequent sub_ghz_replay_start_async()
 *         call streams from the given path.
 *
 * No hardware is touched here.  No temp file is created.
 *
 * @retval 0 = success, 1 = invalid args, 3 = unsupported frequency
 */
uint8_t sub_ghz_replay_prepare_datafile(const char *sgh_path,
                                        uint32_t frequency, uint8_t modulation);

/**
 * @brief  Convert a Flipper .sub file (or M1 PACKET .sgh) into a temp .sgh
 *         and prepare globals for streaming.
 *
 * On success, *out_tmp_path is set to a static string giving the path of the
 * temp file on the SD card.  Ownership of that file transfers to the caller:
 * the caller MUST f_unlink() it after the TX completes or is aborted.
 *
 * On failure, the temp file (if any was created) has already been unlinked.
 *
 * @param  sub_path      Path to the source Flipper .sub / M1 PACKET file.
 * @param  out_tmp_path  Required; *out_tmp_path is filled in on success with
 *                       the temp .sgh path so the caller can unlink it after
 *                       TX completion / abort.
 *
 * @retval 0 = success, 1 = invalid args, non-zero otherwise = same error code set as
 *         sub_ghz_replay_flipper_file()
 */
uint8_t sub_ghz_replay_prepare_flipper(const char *sub_path,
                                       const char **out_tmp_path);

/**
 * @brief  Arm the radio for async TX after a prepare_* call.
 *
 * Allocates ring buffers, opens the streaming file, calls menu_sub_ghz_init(),
 * and starts the first TIM1+DMA TX burst.  Returns immediately; further work
 * happens in the DMA TC ISR which posts Q_EVENT_SUBGHZ_TX.
 *
 * On failure, all internal resources have been released and the radio is
 * powered off — the caller does NOT need to call abort.
 *
 * @retval 0 = success (TX armed and running), non-zero = error code:
 *         1 = ISM-restricted (handled internally with a user message), 4 =
 *         ring-buffer OOM, 5 = streaming sample OOM, other = TX init error.
 */
uint8_t sub_ghz_replay_start_async(void);

/**
 * @brief  Advance the async TX on a Q_EVENT_SUBGHZ_TX completion event.
 *
 * @param  repeat_on_idle  true  → auto-restart on natural end (continuous
 *                                  emulation; matches the legacy blocking
 *                                  loop's auto-restart-until-BACK behaviour).
 *                         false → return DONE on first natural end (one-shot;
 *                                  the scene's TX state expects this).
 *
 * On DONE or ERROR, all teardown is performed internally before returning.
 * Subsequent calls return DONE immediately.
 *
 * Safe to call when no replay is active — returns DONE.
 */
sub_ghz_replay_async_status_t sub_ghz_replay_continue_async(bool repeat_on_idle);

/**
 * @brief  Abort any in-progress async replay synchronously.
 *
 * Performs full teardown: stops TX DMA, frees ring buffers and streaming
 * samples, turns the RGB blink off, deasserts SI4463 ENA via
 * menu_sub_ghz_exit().  Safe to call when no replay is active (no-op).
 *
 * A late Q_EVENT_SUBGHZ_TX arriving after this returns is safe:
 * continue_async() detects the inactive state and returns DONE without
 * touching hardware.
 */
void sub_ghz_replay_abort(void);

/**
 * @brief  True iff an async replay is currently armed/running.
 *
 * The Read Raw scene uses this to decide whether a BACK event during a TX
 * state should call abort, and to gate late Q_EVENT_SUBGHZ_TX events that
 * may race the abort.
 */
bool sub_ghz_replay_async_is_active(void);

/**
 * @brief  Set the button-override slot consumed by the next
 *         sub_ghz_replay_prepare_flipper() / sub_ghz_replay_flipper_file()
 *         call.
 *
 * Used by the Transmitter scene to implement LEFT/RIGHT button cycling
 * for rolling-code remotes.  When @p button_index is in 0..15 and the
 * loaded Flipper SubGhz Key File's protocol is in the supported set
 * (see subghz_button_override.h — currently KeeLoq / Jarolift /
 * Star Line), the converter mutates the parsed `key_value` to encode
 * the requested button before invoking the OOK PWM encoder.  For all
 * other protocols, the override is silently ignored.
 *
 * The slot is automatically cleared by the converter at the end of
 * the conversion (whether successful or not), so each set + prepare
 * cycle is self-contained.  Calling this with @p button_index < 0
 * also clears the slot.
 *
 * @param  button_index  0..15 = override active for the next prepare;
 *                       <0    = no override (use the file's value).
 */
void sub_ghz_replay_set_button_override(int8_t button_index);

/* Scene-based UI entry point (new Flipper-inspired architecture) */
void sub_ghz_scene_entry(void);

extern EXTI_HandleTypeDef 	si4463_exti_hdl;
extern TIM_HandleTypeDef   	timerhdl_subghz_tx;
extern TIM_HandleTypeDef   	timerhdl_subghz_rx;
extern DMA_HandleTypeDef	hdma_subghz_tx;
extern uint8_t subghz_tx_tc_flag;
extern S_M1_RingBuffer subghz_rx_rawdata_rb;
extern uint8_t subghz_record_mode_flag;
#endif /* M1_SUB_GHZ_H_ */
