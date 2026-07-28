/* See COPYING.txt for license details. */

/*
*
* m1_buzzer.c
*
* Driver and library for buzzer controller
*
* M1 Project
*
*/


/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_buzzer.h"

/*************************** D E F I N E S ************************************/

//************************** C O N S T A N T **********************************/

#define BUZZER_NOTIFICATION_DURATION	250 //ms

//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/

TIM_HandleTypeDef    Timerhdl_Buzzer;

static bool buzzer_busy = false;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void buzzer_sys_init(uint16_t frequency);
static void buzzer_sys_deinit(TimerHandle_t xTimer);

void m1_buzzer_set(uint16_t frequency, uint16_t duration);
void m1_buzzer_notification(void);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
  * @brief  Initialize the buzzer controller
  * Output modulated (PWM carrier + base band) data on SPEAKER GPIO pin
 */
/*============================================================================*/
void buzzer_sys_init(uint16_t frequency)
{
#ifdef M1_APP_BUZZER_USE_TIMER8
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
#endif // #ifdef M1_APP_BUZZER_USE_TIMER8
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	GPIO_InitTypeDef gpio_init_struct;
	uint32_t tim_prescaler_val;

	BUZZER_TIMER_CLK();

	Timerhdl_Buzzer.Instance = BUZZER_TIMER;

	/*Configure GPIO pin */
	gpio_init_struct.Pin = SPK_CTRL_Pin;
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	gpio_init_struct.Alternate = BUZZER_GPIO_AF_TR;
	HAL_GPIO_Init(SPK_CTRL_GPIO_Port, &gpio_init_struct);

	tim_prescaler_val = (uint32_t) (HAL_RCC_GetPCLK2Freq() / (frequency*BUZZER_CARRIER_PRESCALE_FACTOR)) - 1;

	Timerhdl_Buzzer.Init.Prescaler = tim_prescaler_val;
	Timerhdl_Buzzer.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timerhdl_Buzzer.Init.Period = BUZZER_CARRIER_PRESCALE_FACTOR - 1;
	Timerhdl_Buzzer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Timerhdl_Buzzer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_PWM_Init(&Timerhdl_Buzzer) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&Timerhdl_Buzzer, &sMasterConfig) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = Timerhdl_Buzzer.Init.Period/2; /* Duty cycle = 50% */
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	//sConfigOC.OCNPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	//sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	if (HAL_TIM_PWM_ConfigChannel(&Timerhdl_Buzzer, &sConfigOC, BUZZER_TIMER_TX_CHANNEL) != HAL_OK)
	{
		//_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}
	//Timerhdl_Buzzer.Instance->CCER &= ~(TIM_CCER_CCxE_MASK + TIM_CCER_CCxNE_MASK);

#ifdef M1_APP_BUZZER_USE_TIMER8
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&Timerhdl_Buzzer, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
#endif // #ifdef M1_APP_BUZZER_USE_TIMER8

	//HAL_TIMEx_PWMN_Start(&Timerhdl_Buzzer, BUZZER_TIMER_TX_CHANNEL);
	HAL_TIM_PWM_Start(&Timerhdl_Buzzer, BUZZER_TIMER_TX_CHANNEL);

	/* TIM Disable */
	//__HAL_TIM_DISABLE(&Timerhdl_Buzzer);
} // void buzzer_sys_init(uint16_t frequency)



/*============================================================================*/
/**
  * @brief  De-initializes the peripherals (RCC,GPIO, TIM)
  * @param  None
  * @retval None
  */
/*============================================================================*/
void buzzer_sys_deinit(TimerHandle_t xTimer)
{
	GPIO_InitTypeDef gpio_init_struct;

	HAL_TIM_PWM_Stop(&Timerhdl_Buzzer, BUZZER_TIMER_TX_CHANNEL);

	BUZZER_TIMER_CLK_DIS();

	HAL_NVIC_DisableIRQ(BUZZER_TIMER_IRQn);

	gpio_init_struct.Pin = SPK_CTRL_Pin;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPK_CTRL_GPIO_Port, &gpio_init_struct);
	HAL_GPIO_WritePin(SPK_CTRL_GPIO_Port, SPK_CTRL_Pin, GPIO_PIN_RESET);

	buzzer_busy = false; // unlock

	//HAL_GPIO_DeInit(SPK_CTRL_GPIO_Port, SPK_CTRL_Pin);

	/* One-shot timer: free its control block (its own expiry callback) — otherwise
	 * a heap_4 timer block leaks on every beep. */
	xTimerDelete(xTimer, 0);
} // static void buzzer_sys_deinit(TimerHandle_t xTimer)



/*============================================================================*/
/**
  * @brief  Turn on the buzzer
  * @param  frequency in Hertz
  * 		duration in millisecond
  * @retval None
  */
/*============================================================================*/
void m1_buzzer_set(uint16_t frequency, uint16_t duration_ms)
{
	if ( buzzer_busy )
		return;

	buzzer_busy = true; // lock

	if ( frequency==0 )
		return;

	if ( duration_ms==0 )
		return;

	TimerHandle_t buzzer_play =	xTimerCreate("m1_buzzer_play", duration_ms/portTICK_PERIOD_MS, pdFALSE, NULL, buzzer_sys_deinit);
	assert_param(buzzer_play != NULL);
	buzzer_sys_init(frequency); // Start buzzer
	xTimerStart(buzzer_play, 0); // Schedule to stop buzzer
} // void m1_buzzer_set(uint16_t frequency, uint16_t duration_ms)


/*============================================================================*/
/**
  * @brief  Play a standard notification sound
  * @param
  * @retval None
  */
/*============================================================================*/
void m1_buzzer_notification(void)
{
	 m1_buzzer_set(BUZZER_FREQ_04_KHZ, BUZZER_NOTIFICATION_DURATION);
} // void m1_buzzer_notification(void)


/*============================================================================*/
/**
  * @brief  Play a standard notification sound
  * @param
  * @retval None
  */
/*============================================================================*/
void m1_buzzer_notification2(void)
{
	m1_buzzer_set(BUZZER_FREQ_02_KHZ, BUZZER_NOTIFICATION_DURATION);
} // void m1_buzzer_notification2(void)

/*============================================================================*/
/**
  * @brief  Play a standard notification sound
  * @param
  * @retval None
  */
/*============================================================================*/
void m1_buzzer_demoTest(uint8_t freqStp)
{
	switch (freqStp)
	{
	case 0:
		m1_buzzer_set(BUZZER_FREQ_02_KHZ, BUZZER_NOTIFICATION_DURATION);
		break;
	case 1:
		m1_buzzer_set(BUZZER_FREQ_04_KHZ, BUZZER_NOTIFICATION_DURATION);
		break;
	case 2:
		m1_buzzer_set(BUZZER_FREQ_06_KHZ, BUZZER_NOTIFICATION_DURATION);
		break;	
	case 3:
		m1_buzzer_set(BUZZER_FREQ_07_KHZ, BUZZER_NOTIFICATION_DURATION);
		break;
	case 4:
		m1_buzzer_set(BUZZER_FREQ_08_KHZ, BUZZER_NOTIFICATION_DURATION);
		break;	
	case 5:
		m1_buzzer_set(BUZZER_FREQ_12_KHZ, BUZZER_NOTIFICATION_DURATION);
		break;
	case 6:
		m1_buzzer_set(BUZZER_FREQ_14_KHZ, BUZZER_NOTIFICATION_DURATION);
		break;
	case 7:
		m1_buzzer_set(BUZZER_FREQ_16_KHZ, BUZZER_NOTIFICATION_DURATION);
		break;	

	default:
		break;
	}
} // void m1_buzzer_notification2(void)
