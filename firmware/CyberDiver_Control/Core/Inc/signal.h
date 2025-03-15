/*
 * signal.h
 *
 *  Created on: Jul 7, 2024
 *      Author: johnt
 */

#ifndef INC_SIGNAL_H_
#define INC_SIGNAL_H_

#include <stdint.h>

#define SIGNAL_BUFFER_LEN 256

typedef struct
{
	float data[SIGNAL_BUFFER_LEN];  /* storage for the data in the signal */
	uint32_t data_index;
} Signal;

void Signal_Init(Signal* signal);
void Signal_AddDataPoint(Signal* signal, float value);
float Signal_GetValue(Signal* signal);
float Signal_GetOldValue(Signal* signal, uint32_t index);  /* index = 0 -> current value, index = 1 -> value just before that, etc. */

/*
 * design intent: by storing all of our data in this signal struct, we automatically have the large array of old data for double buffering.
 * Additionally, we can easily implement filters for the PID controllers or cyber-phsyical calculations in the future
 * The sampling rate could possibly be changed on the fly, so we can explicitly pass the array of previous time stamps as another Signal object
 * to the function that applies the filter
 */
float Signal_Convolve(Signal* signal, float* taps, uint32_t len);

#endif /* INC_SIGNAL_H_ */
