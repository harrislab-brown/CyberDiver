/*
 * power_board_config.c
 *
 *  Created on: Sep 11, 2024
 *      Author: johnt
 */

#include "power_board_config.h"


void PowerBoardConfig_Init(PowerBoardConfig* config)  /* initialize configuration with defaults */
{

	config->pid_config.Kp = 0.0f;
	config->pid_config.Ki = 0.0f;
	config->pid_config.Kd = 0.0f;
	config->pid_config.Kff = 1.0f;
	config->pid_config.lim_min = -1.0f;
	config->pid_config.lim_max = 1.0f;
	config->pid_config.T = 0.00002f;
	config->pid_config.tau = 0.0f;

	config->Kff_vel = 0.0f;
}
