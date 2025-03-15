/*
 * cyber_diver_encoder.h
 *
 *  Created on: Oct 28, 2024
 *      Author: johnt
 */

#ifndef INC_CYBER_DIVER_ENCODER_H_
#define INC_CYBER_DIVER_ENCODER_H_

#include <stdlib.h>
#include "main.h"



#define CD_ENCODER_MM_PER_COUNT -0.001f  /* note that this has a sign too */
#define CD_ENCODER_INDEX_MIN_WIDTH 30  /* counts between the index window edges if the encoder is working properly is 40 um +/- 25% */
#define CD_ENCODER_INDEX_MAX_WIDTH 50



typedef struct
{
	float neutral_position_offset_mm;  /* where to define the zero position relative to the encoder index */
} CyberDiverEncoder_Config;

typedef struct
{
	CyberDiverEncoder_Config* config;
	TIM_HandleTypeDef* htim;
	uint32_t is_homed;

	/* variables used during the homing procedure */
	GPIO_PinState old_pin_state;
	uint32_t seen_first_edge;
	uint32_t count_at_first_edge;

} CyberDiverEncoder;

void CyberDiverEncoder_Init(CyberDiverEncoder* encoder, CyberDiverEncoder_Config* config, TIM_HandleTypeDef* htim);
void CyberDiverEncoder_Update(CyberDiverEncoder* encoder);

void CyberDiverEncoder_Home(CyberDiverEncoder* encoder);

void CyberDiverEncoder_SetOffset(CyberDiverEncoder* encoder, float offset);
float CyberDiverEncoder_GetOffset(CyberDiverEncoder* encoder);

int32_t CyberDiverEncoder_GetCounts(CyberDiverEncoder* encoder);
float CyberDiverEncoder_GetPosition(CyberDiverEncoder* encoder);

uint32_t CyberDiverEncoder_GetHomed(CyberDiverEncoder* encoder);


#endif /* INC_CYBER_DIVER_ENCODER_H_ */
