/*
 * cyber_diver_power_board.h
 *
 *  Created on: Sep 11, 2024
 *      Author: johnt
 */

#ifndef INC_POWER_BOARD_H_
#define INC_POWER_BOARD_H_

#include "stm32f4xx_hal.h"

#include "power_board_config.h"
#include "power_board_estimator.h"
#include "power_board_interface.h"
#include "pid.h"


//#define PB_TEST  /* runs a test at startup of the current sensing */


#define PB_DEBOUNCE_TIME_MS 250
#define PB_VBAT_SAMPLE_TIME_MS 250
#define PB_VBAT_PER_BIT 0.0068729003f
#define PB_VBAT_MIN 23.0f

#define PB_PWM_TIM_COUNTS_PER_SECOND 84000000
#define PB_PWM1_CH TIM_CHANNEL_2
#define PB_PWM2_CH TIM_CHANNEL_1
#define PB_TIM_SAMPLE_CH TIM_CHANNEL_2
#define PB_TIM_SAMPLE_CCR 100  /* tune this to account for phase delay in the sensing circuitry and AD conversion */


typedef struct
{
	PowerBoardConfig config;
	PowerBoardEstimator estimator;
	PowerBoardInterface interface;
	PIDController pid;


	/* HAL handles to coil PWM timers */
	TIM_HandleTypeDef* hPWM1;  /* TIM3_CH2 */
	TIM_HandleTypeDef* hPWM2;  /* TIM1_CH1 */


	/* sampling timer and ADC handle */
	TIM_HandleTypeDef* hTIM_sample;  /* TIM2_CH2 */
	ADC_HandleTypeDef* hadc_cur;


	/* master timer for synchronization */
	TIM_HandleTypeDef* hTIM_master;  /* TIM4 one shot */


	/* user button */
	uint32_t button_time_last_press;


	/* battery voltage monitoring */
	ADC_HandleTypeDef* hadc_vbat;
	uint32_t vbat_time_last_sample;


} PowerBoard;

void PowerBoard_Init(PowerBoard* pb,
		TIM_HandleTypeDef* hPWM1,
		TIM_HandleTypeDef* hPWM2,
		TIM_HandleTypeDef* hTIM_sample,
		TIM_HandleTypeDef* hTIM_master,
		ADC_HandleTypeDef* hadc_cur,
		ADC_HandleTypeDef* hadc_vbat,
		SPI_HandleTypeDef* hspi);

void PowerBoard_LoopUpdate(PowerBoard* pb);  /* main loop update */
void PowerBoard_ControllerUpdate(PowerBoard* pb);  /* time critical controller update */

void PowerBoard_StartControl(PowerBoard* pb);  /* functions to start/stop all the hardware/interrupts used for the control loop */
void PowerBoard_StopControl(PowerBoard* pb);

void PowerBoard_DisableCommunicationInterrupts();
void PowerBoard_EnableCommunicationInterrupts();

#ifdef PB_TEST
void PowerBoard_RunTest(PowerBoard* pb);
#endif

#endif /* INC_POWER_BOARD_H_ */
