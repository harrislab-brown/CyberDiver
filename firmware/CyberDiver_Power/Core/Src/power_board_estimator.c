/*
 * power_board_estimator.c
 *
 *  Created on: Sep 11, 2024
 *      Author: johnt
 */

#include "power_board_estimator.h"

void PowerBoardEstimator_Init(PowerBoardEstimator* estimator)
{

	estimator->adc_buf_offset = PB_NUM_SAMPLES_PER_CYCLE;
	estimator->samples_available = 0;

	for (uint32_t i = 0; i < PB_NUM_SAMPLES_PER_CYCLE; i++)
		estimator->use_sample[i] = 1;  /* by default, use all the samples */

}

void PowerBoardEstimator_Reset(PowerBoardEstimator* estimator)
{
	estimator->adc_buf_offset = 0;
}

float PowerBoardEstimator_GetCurrent(PowerBoardEstimator* estimator)
{
	float current = 0.0f;
	uint32_t samples = 0;

	estimator->adc_buf_offset = estimator->adc_buf_offset ? 0 : PB_NUM_SAMPLES_PER_CYCLE;  /* use the other half of the double buffer on the next update */

	for (uint32_t i = 0; i < PB_NUM_SAMPLES_PER_CYCLE; i++)
	{
		current += estimator->use_sample[i] * estimator->adc_current[i + estimator->adc_buf_offset];
		samples += estimator->use_sample[i];
	}

	current /= samples;
	current = (current - 0.5f * PB_ADC_MAX_READING) * PB_AMPS_PER_BIT;


	/* do any digital filtering or update state estimators here */


	estimator->samples_available = 1;
	return current;
}

void PowerBoardEstimator_SetDuty(PowerBoardEstimator* estimator, float duty_cycle)  /* tell the estimator the duty cycle for the next period so it can elect to ignore samples too close to edges */
{
	/* ignore current samples too close to PWM edges - these measurements will be corrupted by common mode transients */


	/* duty cycle close to 0 -> only use samples 0 and 2 */
	/* duty cycle close to +/- 1 -> only use samples 1 and 3 */
	/* otherwise, use all of the samples */


	duty_cycle = fabs(duty_cycle);

	if (duty_cycle < PB_SAMPLE_DISCARD_THRESHOLD)
	{
		estimator->use_sample[0] = 1;
		estimator->use_sample[1] = 0;
		estimator->use_sample[2] = 1;
		estimator->use_sample[3] = 0;
	}
	else if (fabs(duty_cycle - 1.0f) < PB_SAMPLE_DISCARD_THRESHOLD)
	{
		estimator->use_sample[0] = 0;
		estimator->use_sample[1] = 1;
		estimator->use_sample[2] = 0;
		estimator->use_sample[3] = 1;
	}
	else
	{
		estimator->use_sample[0] = 1;
		estimator->use_sample[1] = 1;
		estimator->use_sample[2] = 1;
		estimator->use_sample[3] = 1;
	}
}

uint32_t PowerBoardEstimator_GetIndividualSamples(PowerBoardEstimator* estimator, uint16_t* samples)  /* for calibration, returns true if new samples are available and then passes them through the argument */
{
	if (estimator->samples_available)
	{
		estimator->samples_available = 0;

		for (uint32_t i = 0; i < PB_NUM_SAMPLES_PER_CYCLE; i++)
			*(samples + i) = *(estimator->adc_current + estimator->adc_buf_offset + i);

		return 1;
	}

	return 0;
}
