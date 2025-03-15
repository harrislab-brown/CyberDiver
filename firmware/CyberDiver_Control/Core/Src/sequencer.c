/*
 * sequencer.c
 *
 *  Created on: Jul 6, 2024
 *      Author: johnt
 */

#include "sequencer.h"

void Sequencer_Init(Sequencer* sequencer, volatile uint32_t* time_micros_ptr)
{
	sequencer->time_micros_ptr = time_micros_ptr;

	sequencer->sequence_time_array = NULL;
	sequencer->sequence_len = 0;
	sequencer->sequence_index = 0;
	sequencer->time = 0;

	sequencer->is_running = 0;
	sequencer->is_looping = 0;
}

void Sequencer_SetSequence(Sequencer* sequencer, const uint32_t* sequence_time_array, const uint32_t sequence_len, uint8_t is_looping)
{
	sequencer->sequence_time_array = sequence_time_array;
	sequencer->sequence_len = sequence_len;
	sequencer->is_looping = is_looping;
}

uint32_t Sequencer_Update(Sequencer* sequencer, uint32_t* index)
{

	/*
	 * each element in the time array says how long to hold that step for
	 */

	if (sequencer->is_running && sequencer->sequence_time_array != NULL)
	{
		if (*sequencer->time_micros_ptr > sequencer->time)
		{
			*index = sequencer->sequence_index;
			sequencer->time += sequencer->sequence_time_array[sequencer->sequence_index];
			sequencer->sequence_index++;

			if (sequencer->sequence_index == sequencer->sequence_len)  /* reached the end of the sequence */
			{
				if (sequencer->is_looping)
					sequencer->sequence_index = 0;
				else
					sequencer->is_running = 0;
			}

			return 1;
		}
	}

	return 0;

}

void Sequencer_Start(Sequencer* sequencer)
{
	sequencer->time = *sequencer->time_micros_ptr;
	sequencer->sequence_index = 0;
	sequencer->is_running = 1;
}

void Sequencer_Stop(Sequencer* sequencer)
{
	sequencer->is_running = 0;
}

uint32_t Sequencer_IsRunning(Sequencer* sequencer)
{
	return sequencer->is_running;
}

