/*
 * power_board_estimator.h
 *
 *  Created on: Sep 11, 2024
 *      Author: johnt
 */

#ifndef INC_POWER_BOARD_ESTIMATOR_H_
#define INC_POWER_BOARD_ESTIMATOR_H_

#include <stdint.h>
#include <math.h>

/* responsible for estimating the system state (true coil current) based on the ADC measurements */

#define PB_NUM_SAMPLES_PER_CYCLE 4  /* we sample 4 types per period in the SVPWM-style H-bridge modulation, and the samples should all be the mean current value at steady state */
#define PB_ADC_MAX_READING 4095.0f
#define PB_ADC_MAX_VOLTAGE 3.3f
#define PB_R_SENSE 0.006f
#define PB_CUR_AMP_GAIN 50.0f
#define PB_AMPS_PER_BIT PB_ADC_MAX_VOLTAGE / (PB_ADC_MAX_READING * PB_R_SENSE * PB_CUR_AMP_GAIN)

#define PB_SAMPLE_DISCARD_THRESHOLD 0.25f


typedef struct
{

} PowerBoardEstimator_Config;

typedef struct
{
	PowerBoardEstimator_Config* config;

	uint16_t adc_current[2 * PB_NUM_SAMPLES_PER_CYCLE];
	volatile uint32_t adc_buf_offset;  /* offset index for double buffering the ADC readings */

	/*
	 * Each cycle, we want to keep track of when the MOSFET switching occurs so we can reject samples that fall too close to an edge (common mode ringing corrupts the sample)
	 * This array contains flags of whether to use or discard each sample
	 */
	uint8_t use_sample[4];
	volatile uint32_t samples_available;


} PowerBoardEstimator;

void PowerBoardEstimator_Init(PowerBoardEstimator* estimator);
void PowerBoardEstimator_Reset(PowerBoardEstimator* estimator);

float PowerBoardEstimator_GetCurrent(PowerBoardEstimator* estimator);
void PowerBoardEstimator_SetDuty(PowerBoardEstimator* estimator, float duty_cycle);  /* tell the estimator the duty cycle for the next period so it can elect to ignore samples too close to edges */

uint32_t PowerBoardEstimator_GetIndividualSamples(PowerBoardEstimator* estimator, uint16_t* samples);  /* for calibration, returns true if new samples are available and then passes them through the argument */

#endif /* INC_POWER_BOARD_ESTIMATOR_H_ */
