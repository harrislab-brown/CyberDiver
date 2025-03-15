/*
 * power_board_config.h
 *
 *  Created on: Sep 11, 2024
 *      Author: johnt
 */

#ifndef INC_POWER_BOARD_CONFIG_H_
#define INC_POWER_BOARD_CONFIG_H_

#include "power_board_estimator.h"
#include "pid.h"

typedef struct
{
	PIDController_Config pid_config;

	float Kff_vel;  /* velocity feed-forward gain */
} PowerBoardConfig;

void PowerBoardConfig_Init(PowerBoardConfig* config);  /* initialize configuration with defaults */

#endif /* INC_POWER_BOARD_CONFIG_H_ */
