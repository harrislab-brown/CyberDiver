/*
 * power_board.c
 *
 *  Created on: Sep 11, 2024
 *      Author: johnt
 */

#include "power_board.h"
#include "main.h"


void PowerBoard_Init(PowerBoard* pb,
		TIM_HandleTypeDef* hPWM1,
		TIM_HandleTypeDef* hPWM2,
		TIM_HandleTypeDef* hTIM_sample,
		TIM_HandleTypeDef* hTIM_master,
		ADC_HandleTypeDef* hadc_cur,
		ADC_HandleTypeDef* hadc_vbat,
		SPI_HandleTypeDef* hspi)
{

	/* pass in the HAL hardware handles */
	pb->hPWM1 = hPWM1;
	pb->hPWM2 = hPWM2;
	pb->hTIM_sample = hTIM_sample;
	pb->hTIM_master = hTIM_master;
	pb->hadc_cur = hadc_cur;
	pb->hadc_vbat = hadc_vbat;

	PowerBoard_StopControl(pb);

	PowerBoardConfig_Init(&pb->config);  /* default configuration values */
	PowerBoardEstimator_Init(&pb->estimator);
	PIDController_Init(&pb->pid, &pb->config.pid_config);

	HAL_Delay(100);  /* give time for MCUs to power up before looking for SPI transactions */

#ifdef PB_TEST
	PowerBoard_RunTest(pb);
#endif

	PowerBoard_EnableCommunicationInterrupts();
	PowerBoardInterface_Init(&(pb->interface), hspi);  /* start the SPI communication with the main board */
}


void PowerBoard_LoopUpdate(PowerBoard* pb)  /* main loop update */
{
	/* check physical button acting as H-bridge E-stop */
	if (HAL_GetTick() - pb->button_time_last_press > PB_DEBOUNCE_TIME_MS
			&& HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET
			&& PowerBoardInterface_GetMode(&(pb->interface)) == PB_RUNNING)
	{
		HAL_GPIO_TogglePin(COIL_EN_GPIO_Port, COIL_EN_Pin);
		pb->button_time_last_press = HAL_GetTick();
	}

	/* handle changed to the interface operating mode */
	if (PowerBoardInterface_ModeChanged(&(pb->interface)))
	{
		if (PowerBoardInterface_GetMode(&(pb->interface)) == PB_IDLE)
		{
			PowerBoard_StopControl(pb);
		}
		else if (PowerBoardInterface_GetMode(&(pb->interface)) == PB_RUNNING)
		{
			pb->config = PowerBoardInterface_GetConfig(&pb->interface);  /* update the configuration based on what we received from the control board */
			PowerBoard_StartControl(pb);
		}
	}

	/* check the battery voltage and stop operation if too low */
	if (HAL_GetTick() - pb->vbat_time_last_sample > PB_VBAT_SAMPLE_TIME_MS)
	{
		HAL_ADC_Start(pb->hadc_vbat);
		HAL_ADC_PollForConversion(pb->hadc_vbat, 10);

		if ((float)HAL_ADC_GetValue(pb->hadc_vbat) * PB_VBAT_PER_BIT < PB_VBAT_MIN)
		{
			/* low battery !!! */
			HAL_GPIO_WritePin(COIL_EN_GPIO_Port, COIL_EN_Pin, GPIO_PIN_RESET);  /* disable the H-bridge */
			__disable_irq();
			while (1)
			{
				HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);  /* fast blink blue lED */
				HAL_Delay(250);
			}
		}

		pb->vbat_time_last_sample = HAL_GetTick();
	}
}


void PowerBoard_ControllerUpdate(PowerBoard* pb)  /* time critical controller update */
{
#ifdef PB_TEST  /* set the INT2 pin so a scope can measure control loop timing */
	PWR_INT2_GPIO_Port->BSRR = PWR_INT2_Pin;
#endif

	/* get the coil current from the state estimator */
	float measurement = PowerBoardEstimator_GetCurrent(&(pb->estimator));

	/* get the set point from the SPI interface */
	float setpoint = PowerBoardInterface_GetSetpoint(&(pb->interface));

	/* update the PID controller */
	float duty_cycle = PIDController_Update(&(pb->pid), setpoint, measurement);

	/* add the velocity feed-forward term */
	duty_cycle += pb->config.Kff_vel * PowerBoardInterface_GetVelocity(&(pb->interface));

	/*
	 *  TODO: might be better to do the feed-forward calculation on the control board side, and send the duty cycle to add
	 *  Especially if the back EMF coefficient depends strongly on displacement, we may need to measure it and implement the
	 *  calibration curve on the control board side.
	 */

	/* update the H-bridge duty cycle */
	__HAL_TIM_SET_COMPARE(pb->hPWM1, PB_PWM1_CH, (float)pb->hPWM1->Instance->ARR / 2.0f * (1 + duty_cycle));  /* tested to make sure the sign here is correct */
	__HAL_TIM_SET_COMPARE(pb->hPWM2, PB_PWM2_CH, (float)pb->hPWM2->Instance->ARR / 2.0f * (1 - duty_cycle));

	/* tell the estimator the duty cycle so it can pick when to sample during the next period */
	PowerBoardEstimator_SetDuty(&(pb->estimator), duty_cycle);

	/* add the most recent data to the SPI interface */
	PowerBoardInterface_SetData(&(pb->interface), measurement, duty_cycle);

#ifdef PB_TEST
	PWR_INT2_GPIO_Port->BSRR = (uint32_t)PWR_INT2_Pin << 16U;
#endif
}

/* functions to start/stop all the hardware/interrupts used for the control loop */
/* this will update registers based on the configuration, so call it whenever we receive a new configuration */
void PowerBoard_StartControl(PowerBoard* pb)
{

	/* reset the controller objects */
	PowerBoardEstimator_Reset(&(pb->estimator));
	PIDController_Reset(&(pb->pid));

	/* coil PWM timers */
	uint32_t coil_pwm_freq_hz = 1.0f / pb->config.pid_config.T;
	uint32_t arr = PB_PWM_TIM_COUNTS_PER_SECOND / (1.0f / pb->config.pid_config.T) / 2;
	pb->config.pid_config.T = (float)(2.0f * arr) / (float)(PB_PWM_TIM_COUNTS_PER_SECOND);  /* update the value in the configuration so that it divides evenly into the timer frequency */

	pb->hPWM1->Instance->ARR = arr;
	pb->hPWM2->Instance->ARR = arr;
	__HAL_TIM_SET_COMPARE(pb->hPWM1, PB_PWM1_CH, arr / 2);
	__HAL_TIM_SET_COMPARE(pb->hPWM2, PB_PWM2_CH, arr / 2);

	pb->hPWM2->Instance->CR1 &= ~(TIM_CR1_CEN);  /* activate the coil PWM timer channels, HAL seems to enable them immediately despite slave mode so use registers instead */
	pb->hPWM2->Instance->CNT = 0;
	pb->hPWM2->Instance->SR = 0;
	pb->hPWM2->Instance->BDTR |= TIM_BDTR_MOE;  /* !!! extra step to enable main output on advanced timers !!! */
	pb->hPWM2->Instance->CCER |= (TIM_CCx_ENABLE << PB_PWM2_CH);

	pb->hPWM1->Instance->CR1 &= ~(TIM_CR1_CEN);
	pb->hPWM1->Instance->CNT = 0;
	pb->hPWM1->Instance->SR = 0;
	pb->hPWM1->Instance->CCER |= (TIM_CCx_ENABLE << PB_PWM1_CH);

	/* ADC sampling timer */
	pb->hTIM_sample->Instance->ARR = PB_PWM_TIM_COUNTS_PER_SECOND / (coil_pwm_freq_hz * PB_NUM_SAMPLES_PER_CYCLE) - 1;
	__HAL_TIM_SET_COMPARE(pb->hTIM_sample, PB_TIM_SAMPLE_CH, PB_TIM_SAMPLE_CCR);

	 /* activate the timer that triggers the ADC conversions */
	pb->hTIM_sample->Instance->CR1 &= ~(TIM_CR1_CEN);
	pb->hTIM_sample->Instance->CNT = 0;
	pb->hTIM_sample->Instance->SR = 0;  /* clear all interrupt flags */
#ifdef PB_TEST
	pb->hTIM_sample->Instance->DIER |= TIM_IT_CC2;  /* enable timer interrupt and blip a GPIO to check sampling time with scope */
#endif
	pb->hTIM_sample->Instance->CCER |= (TIM_CCx_ENABLE << PB_TIM_SAMPLE_CH);

	/* start the ADC DMA conversions */
	HAL_ADC_Start_DMA(pb->hadc_cur, (uint32_t*)pb->estimator.adc_current, 2 * PB_NUM_SAMPLES_PER_CYCLE);

	/* enable interrupts relating to the controller */
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
#ifdef PB_TEST
	NVIC_EnableIRQ(TIM2_IRQn);
#endif

	/* enable the coil */
	HAL_GPIO_WritePin(COIL_EN_GPIO_Port, COIL_EN_Pin, GPIO_PIN_SET);

	/* start the master timer to synchronize the others */
	HAL_TIM_Base_Start(pb->hTIM_master);
}

void PowerBoard_StopControl(PowerBoard* pb)  /* stop the controller and hardware */
{

	/* disable the coil */
	HAL_GPIO_WritePin(COIL_EN_GPIO_Port, COIL_EN_Pin, GPIO_PIN_RESET);

	/* disable interrupts relating to the controller */
	NVIC_DisableIRQ(DMA2_Stream0_IRQn);
#ifdef PB_TEST
	NVIC_DisableIRQ(TIM2_IRQn);
#endif

	/* stop the timers (master first) */
	HAL_TIM_Base_Stop(pb->hTIM_master);
	HAL_TIM_PWM_Stop(pb->hTIM_sample, PB_TIM_SAMPLE_CH);
	HAL_TIM_PWM_Stop(pb->hPWM1, PB_PWM1_CH);
	HAL_TIM_PWM_Stop(pb->hPWM2, PB_PWM2_CH);

	/* stop the ADC DMA */
	HAL_ADC_Stop_DMA(pb->hadc_cur);

}


void PowerBoard_DisableCommunicationInterrupts()
{
	NVIC_DisableIRQ(DMA1_Stream3_IRQn);
	NVIC_DisableIRQ(DMA1_Stream4_IRQn);
	NVIC_DisableIRQ(EXTI9_5_IRQn);
}

void PowerBoard_EnableCommunicationInterrupts()
{
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}


#ifdef PB_TEST

#include <math.h>

#define PB_COIL_SETTLING_TIME_MS 10
#define PB_COIL_TEST_IDLE_TIME_MS 100
#define PB_COIL_TEST_STEPS 11  /* how many duty cycle values to test between -1 and +1 */
#define PB_COIL_TEST_N 10000  /* how many periods to sample at each duty cycle */

/*
 * Performs a test of the power board H-brid5ge PWM and current sensing.
 * Run the test without the control board connected, since we use the INT pins
 * on the communication cable to indicate when the control loop calculation is running for scope
 *
 * The test will enable the coil and sweep through a few duty_cycle values
 * At each duty cycle, it will record the mean and std current reading at each of the 4 samples for tuning phase delay
 * Then it will disable the coil and wait in an infinite loop
 */

void PowerBoard_RunTest(PowerBoard* pb)
{
	PowerBoard_DisableCommunicationInterrupts();

	float mean[PB_NUM_SAMPLES_PER_CYCLE * PB_COIL_TEST_STEPS];  /* store the results of the test */
	float std[PB_NUM_SAMPLES_PER_CYCLE * PB_COIL_TEST_STEPS];

	for (uint32_t j = 0; j < PB_COIL_TEST_STEPS; j++)
	{
		pb->interface.data_rx[0].current_amps = -1.0f + j * 2.0f / (PB_COIL_TEST_STEPS - 1);
		PowerBoard_StartControl(pb);
		HAL_Delay(PB_COIL_SETTLING_TIME_MS);  /* wait to reach steady state */

		/* get the 4 samples per period from the estimator - accumulate the mean and std */
		uint16_t samples[PB_NUM_SAMPLES_PER_CYCLE];
		uint32_t n = 0;
		float duty_mean[PB_NUM_SAMPLES_PER_CYCLE] = {0};
		float duty_std[PB_NUM_SAMPLES_PER_CYCLE] = {0};

		while (n < PB_COIL_TEST_N)
		{
			if (PowerBoardEstimator_GetIndividualSamples(&pb->estimator, samples))
			{
				n++;
				for (uint32_t i = 0; i < PB_NUM_SAMPLES_PER_CYCLE; i++)
				{
					/* https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance */
					float delta = (float)samples[i] - duty_mean[i];
					duty_mean[i] += delta / n;
					duty_std[i] += delta * ((float)samples[i] - duty_mean[i]);
				}
			}
		}

		for (uint32_t i = 0; i < PB_NUM_SAMPLES_PER_CYCLE; i++)
		{
			mean[i + PB_NUM_SAMPLES_PER_CYCLE * j] = duty_mean[i];
			std[i + PB_NUM_SAMPLES_PER_CYCLE * j] = sqrt(duty_std[i] / (n - 1));
		}

		PowerBoard_StopControl(pb);
		HAL_Delay(PB_COIL_TEST_IDLE_TIME_MS);
	}

	__disable_irq();


	/*
	 * We can use the debugger here to recover the test results. GDB commands to do this:
	 *
	 * >>set logging file [FILE]  // logging destination file
	 * >>set print elements 0  // turns off the element limit for printing
	 * >>set print repeats 0  // prints each element verbatim if there are repeated sequences
     * >>set logging on  // starts logging
     * >>print mean  // print the desired arrays
     * >>print std
     * >>set logging off  // close and save the log file
     *
     * Then can open in MATLAB for further processing
	 */


	while (1);
}
#endif
