/*
 * cyber_diver_encoder.c
 *
 *  Created on: Oct 28, 2024
 *      Author: johnt
 */


#include "cyber_diver_encoder.h"


void CyberDiverEncoder_Init(CyberDiverEncoder* encoder, CyberDiverEncoder_Config* config, TIM_HandleTypeDef* htim)
{
	encoder->config = config;
	encoder->htim = htim;
	encoder->is_homed = 0;
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}


void CyberDiverEncoder_Update(CyberDiverEncoder* encoder)
{
	/* home the encoder here without blocking */

	if (!encoder->is_homed)
	{
		GPIO_PinState new_pin_state = HAL_GPIO_ReadPin(ENC_IW_GPIO_Port, ENC_IW_Pin);

		if (new_pin_state != encoder->old_pin_state)
		{
			/* caught an edge */

			if (!encoder->seen_first_edge)
			{
				encoder->count_at_first_edge = encoder->htim->Instance->CNT;
				encoder->seen_first_edge = 1;
			}
			else
			{
				int32_t index_width = encoder->htim->Instance->CNT - encoder->count_at_first_edge;
				if (abs(index_width) > CD_ENCODER_INDEX_MIN_WIDTH && abs(index_width) < CD_ENCODER_INDEX_MAX_WIDTH)
				{
					encoder->htim->Instance->CNT = index_width / 2;  /* zero out the count */
					encoder->is_homed = 1;
				}
				else
				{
					/* measured index window width out of bounds or captured the same edge twice, so start all over */
					encoder->seen_first_edge = 0;
				}
			}

			encoder->old_pin_state = new_pin_state;
		}
	}
}


void CyberDiverEncoder_Home(CyberDiverEncoder* encoder)
{
	encoder->is_homed = 0;
	encoder->seen_first_edge = 0;
	encoder->count_at_first_edge = 0;
	encoder->old_pin_state = HAL_GPIO_ReadPin(ENC_IW_GPIO_Port, ENC_IW_Pin);  /* capture initial pin state */
}

void CyberDiverEncoder_SetOffset(CyberDiverEncoder* encoder, float offset)
{
	encoder->config->neutral_position_offset_mm = offset;
}

float CyberDiverEncoder_GetOffset(CyberDiverEncoder* encoder)
{
	return encoder->config->neutral_position_offset_mm;
}

int32_t CyberDiverEncoder_GetCounts(CyberDiverEncoder* encoder)
{
	return (int32_t)encoder->htim->Instance->CNT;
}

float CyberDiverEncoder_GetPosition(CyberDiverEncoder* encoder)
{
	return (int32_t)encoder->htim->Instance->CNT * CD_ENCODER_MM_PER_COUNT - encoder->config->neutral_position_offset_mm;
}

uint32_t CyberDiverEncoder_GetHomed(CyberDiverEncoder* encoder)
{
	return encoder->is_homed;
}
