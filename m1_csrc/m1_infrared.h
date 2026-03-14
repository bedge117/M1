/* See COPYING.txt for license details. */

/*
*
*  m1_infrared.h
*
*  M1 Infrared functions
*
* M1 Project
*
*/

#ifndef M1_INFRARED_H_
#define M1_INFRARED_H_

#include <stdbool.h>
#include "stm32h5xx_hal.h"
#include "app_freertos.h"
#include "m1_compile_cfg.h"
#include "queue.h"
#include "irmp.h"

#define IR_DECODE_TIMER                 TIM2        /*!< Timer used for IR decoding */
/* TIM prescaler is computed to have 1 μs as time base. TIM frequency (in MHz) / (prescaler+1) */
#define IR_RX_TIM_PRESCALER          	74           /*!< TIM prescaler */
#define IR_DECODE_TIMER_CLK             __HAL_RCC_TIM2_CLK_ENABLE      /*!< Clock of the used timer */
#define IR_DECODE_TIMER_CLK_DIS         __HAL_RCC_TIM2_CLK_DISABLE
#define IR_DECODE_TIMER_IRQn            TIM2_IRQn             /*!< IR TIM IRQ */
#define IR_DECODE_TIMER_RX_CHANNEL   	TIM_CHANNEL_4            /*!< IR TIM Channel */
#define IR_DECODE_TIMER_DEC_CH_ACTIV  	HAL_TIM_ACTIVE_CHANNEL_4

#define IR_ENCODE_CARRIER_TIMER         TIM1        /*!< Timer used for IR encoding */
#define IR_ENCODE_BASEBAND_TIMER        TIM16
#define IR_TX_TIM_PRESCALER          	0           /*!< TIM prescaler */
#define IR_ENCODE_CARRIER_TIMER_CLK     	__HAL_RCC_TIM1_CLK_ENABLE      /*!< Clock of the used timer */
#define IR_ENCODE_CARRIER_TIMER_CLK_DIS 	__HAL_RCC_TIM1_CLK_DISABLE
#define IR_ENCODE_BASEBAND_TIMER_CLK     	__HAL_RCC_TIM16_CLK_ENABLE
#define IR_ENCODE_BASEBAND_TIMER_CLK_DIS 	__HAL_RCC_TIM16_CLK_DISABLE
#define IR_ENCODE_TIMER_TX_CHANNEL   	TIM_CHANNEL_4            /*!< IR TIM Channel */
#define IR_ENCODE_TIMER_ENC_CH_ACTIV  	HAL_TIM_ACTIVE_CHANNEL_4
#define IR_ENCODE_TIMER_IRQn            TIM16_IRQn             /*!< IR TIM IRQ */

#define IR_ENCODE_CARRIER_FREQ_36KHZ_PERIOD		2083	// clock tick period = 1/75MHz, carrier period = 1/36KHz = 75MHz/36KHz = 2083 clock tick periods
#define IR_ENCODE_CARRIER_FREQ_30_KHZ			(uint32_t)30000
#define IR_ENCODE_CARRIER_FREQ_32_KHZ           (uint32_t)32000
#define IR_ENCODE_CARRIER_FREQ_36_KHZ           (uint32_t)36000
#define IR_ENCODE_CARRIER_FREQ_38_KHZ           (uint32_t)38000
#define IR_ENCODE_CARRIER_FREQ_40_KHZ           (uint32_t)40000
#define IR_ENCODE_CARRIER_FREQ_56_KHZ           (uint32_t)56000
#define IR_ENCODE_CARRIER_FREQ_455_KHZ          (uint32_t)455000

#define IR_ENCODE_CARRIER_PRESCALE_FACTOR		10

#define IR_ENC_HPERIOD_RC5      ((uint32_t)1333)        /*!< RC5 Encoder modulation frequency base period */
#define IR_ENC_LPERIOD_RC5      ((uint32_t)46630)       /*!< RC5 Encoder pulse base period */
#define IR_ENC_HPERIOD_SIRC     ((uint32_t)1200)        /*!< SIRC Encoder modulation frequency base period */
#define IR_ENC_LPERIOD_SIRC     ((uint32_t)28799)       /*!< SIRC Encoder pulse base period */

// IR_RX	PORTC.4	<--> TIM2_CH4
// IR_DRV 	PORTC.5	<--> TIM1_CH4N
#define IR_GPIO_PORT           GPIOC                    /*!< Port which IR input is connected */
#define IR_GPIO_PORT_CLK       __HAL_RCC_GPIOC_CLK_ENABLE      /*!< IR pin GPIO Clock Port */
#define IR_RX_GPIO_PIN         	GPIO_PIN_4               /*!< Pin which IR is connected */
#define IR_TX_GPIO_PIN         	GPIO_PIN_5
#define IR_GPIO_AF_TR          	GPIO_AF1_TIM1
#define IR_GPIO_AF_RX         	GPIO_AF1_TIM2

#define TIM_FORCED_ACTIVE      ((uint16_t)0x0050)
#define TIM_FORCED_INACTIVE    ((uint16_t)0x0040)

#define IR_OTA_PULSE_BIT_MASK	0x0001 // LSB bit = 1 for Mark, using OR operator
#define IR_OTA_SPACE_BIT_MASK	0xFFFE // LSB bit = 0 for Space, using AND operator

typedef enum {
	EDGE_DET_FALLING = 0,
	EDGE_DET_RISING,
	EDGE_DET_IDLE,
	EDGE_DET_ACTIVE
} S_M1_IR_Det;

typedef enum
{
	IR_TX_INIT = 0,
	IR_TX_ACTIVE,
	IR_TX_POST_PROCESS,
	IR_TX_DELAY,
	IR_TX_COMPLETED
} S_M1_IR_Tx_States;

void menu_infrared_init(void);
void menu_infrared_exit(void);

void infrared_universal_remotes(void);
void infrared_learn_new_remote(void);
void infrared_saved_remotes(void);
void infrared_encode_sys_init(void);
void infrared_encode_sys_deinit(void);
S_M1_IR_Tx_States infrared_transmit(uint8_t init);

extern uint32_t TIM_GetCounterCLKValue(uint16_t prescaler);
extern void HAL_TIM_PeriodElapsedCallback_IR(TIM_HandleTypeDef *htim);

extern volatile S_M1_IR_Det IrRx_Edge_Det;

extern TIM_HandleTypeDef    Timerhdl_IrCarrier;
extern TIM_HandleTypeDef    Timerhdl_IrTx;
extern TIM_HandleTypeDef    Timerhdl_IrRx;

extern volatile uint8_t ir_ota_data_tx_active;
extern uint16_t ir_ota_data_tx_len;
extern volatile uint16_t ir_ota_data_tx_counter;
extern uint16_t *pir_ota_data_tx_buffer;

#endif /* M1_INFRARED_H_ */
