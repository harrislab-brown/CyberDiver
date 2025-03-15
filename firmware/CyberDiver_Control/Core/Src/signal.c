/*
 * signal.c
 *
 *  Created on: Jul 7, 2024
 *      Author: johnt
 */

#include "signal.h"

void Signal_Init(Signal* signal)
{
	for (uint32_t i = 0; i < SIGNAL_BUFFER_LEN; i++)
	{
		signal->data[i] = 0.0f;  /* zero out the data buffer */
	}
	signal->data_index = 0;
}

void Signal_AddDataPoint(Signal* signal, float value)
{
	signal->data[signal->data_index++] = value;
	if (signal->data_index == SIGNAL_BUFFER_LEN)
	{
		signal->data_index = 0;
	}
}

float Signal_GetValue(Signal* signal)
{
	return signal->data[signal->data_index];
}

float Signal_GetOldValue(Signal* signal, uint32_t index)
{
	/* returns the value at sample time (N - index) */
	/* if index = 0 we should return the latest data that was sampled */
	/* this will be data_index - 1 because data_index tells where to put the NEXT sample */

	index++;

	if (index > SIGNAL_BUFFER_LEN)
	{
		/* error: values not stored this far back */
		return 0.0f;
	}


	if (index > signal->data_index)
	{
		/* need to wrap the index */
		return signal->data[signal->data_index - index + SIGNAL_BUFFER_LEN];
	}
	else
	{
		return signal->data[signal->data_index - index];
	}
}

float Signal_Convolve(Signal* signal, float* taps, uint32_t len)
{
	/* computes the convolution of the signal with the kernel, for applying filters, estimating derivatives, etc. */
	/* indices of taps correspond to how many time delays, so taps[0] is applied to sample N, taps[1] applied to sample N-1, etc. */

	float ret = 0.0f;

	for (uint32_t i = 0; i < len; i++)
	{
		ret += taps[i] * Signal_GetOldValue(signal, i);
	}

	return ret;
}
