/*
 * cyber_diver_power_board.c
 *
 *  Created on: Jul 10, 2024
 *      Author: johnt
 */

#include "cyber_diver_power_board.h"
#include "fast_spi.h"

void CyberDiverPowerBoard_Init(CyberDiverPowerBoard* power_board,
		CyberDiverPowerBoard_Config* config,
		SPI_HandleTypeDef* spi,
		GPIO_TypeDef* cs_port,
		uint16_t cs_pin,
		GPIO_TypeDef* int1_port,
		uint16_t int1_pin,
		GPIO_TypeDef* int2_port,
		uint16_t int2_pin)
{
	power_board->config = config;
	power_board->spi = spi;
	power_board->cs_port = cs_port;
	power_board->cs_pin = cs_pin;

	power_board->int1_port = int1_port;
	power_board->int1_pin = int1_pin;
	power_board->int2_port = int2_port;
	power_board->int2_pin = int2_pin;

	power_board->mode = POWER_BOARD_IDLE;
	power_board->mode_stable = 0;
	HAL_GPIO_WritePin(power_board->int1_port, power_board->int1_pin, POWER_BOARD_IDLE);
	HAL_Delay(1);
	power_board->mode_stable = 1;

}


uint32_t CyberDiverPowerBoard_Reset(CyberDiverPowerBoard* power_board)
{
	power_board->measured_current_amps = 0;
	power_board->measured_duty_cycle = 0;

	/* send the configuration to the power board */
	CyberDiverPowerBoard_SetMode(power_board, POWER_BOARD_IDLE);
	if (!CyberDiverPowerBoard_UpdateConfig(power_board))
		return 0;

	return 1;
}



void CyberDiverPowerBoard_SetMode(CyberDiverPowerBoard* power_board, CyberDiverPowerBoardMode mode)
{
	/* sets the mode of the power board object and updates the INT pin states accordingly */
	if (power_board->mode != mode)
	{
		power_board->mode_stable = 0;
		power_board->mode = mode;
		HAL_Delay(1);  /* make sure we don't try to talk to the power board too close to a mode change */
		HAL_GPIO_WritePin(power_board->int1_port, power_board->int1_pin, mode);
		HAL_Delay(1);
		power_board->mode_stable = 1;
	}

}


CyberDiverPowerBoardMode CyberDiverPowerBoard_GetMode(CyberDiverPowerBoard* power_board)
{
	return power_board->mode;
}


void CyberDiverPowerBoard_SetCurrent(CyberDiverPowerBoard* power_board, float setpoint_current_amps, float measured_velocity_mmps)
{
	if (power_board->mode == POWER_BOARD_RUNNING && power_board->mode_stable)
	{
		/* store the new command data in the power board object */
		if (setpoint_current_amps >= 5.5) setpoint_current_amps = 5.5;
		if (setpoint_current_amps <= -5.5) setpoint_current_amps = -5.5;
		power_board->tx_data.setpoint_current_scaled = (int16_t)(setpoint_current_amps * CD_PB_CURRENT_SCALE);
		power_board->tx_data.measured_velocity_scaled = (int16_t)(measured_velocity_mmps * CD_PB_VEL_SCALE);

		/* do the SPI communication with our fast implementation */
		SPI_TxRx_Fast((uint8_t*)&(power_board->tx_data), (uint8_t*)&(power_board->rx_data), 4,
				power_board->spi->Instance, power_board->cs_port, power_board->cs_pin);

		/* store the data we got back from the power board */
		power_board->measured_current_amps = (float)power_board->rx_data.measured_current_scaled / CD_PB_CURRENT_SCALE;
		power_board->measured_duty_cycle = (float)power_board->rx_data.measured_duty_cycle_scaled / CD_PB_DUTY_SCALE;
	}
}


float CyberDiverPowerBoard_GetCurrent(CyberDiverPowerBoard* power_board)
{
	return power_board->measured_current_amps;
}


float CyberDiverPowerBoard_GetDutyCycle(CyberDiverPowerBoard* power_board)
{
	return power_board->measured_duty_cycle;
}


uint32_t CyberDiverPowerBoard_SetGains(CyberDiverPowerBoard* power_board, float Kp, float Ki, float Kd, float Kff)
{

	float Kp_old = power_board->config->cur_pid_config.Kp;
	float Ki_old = power_board->config->cur_pid_config.Ki;
	float Kd_old = power_board->config->cur_pid_config.Kd;
	float Kff_old = power_board->config->cur_pid_config.Kff;

	power_board->config->cur_pid_config.Kp = Kp;
	power_board->config->cur_pid_config.Ki = Ki;
	power_board->config->cur_pid_config.Kd = Kd;
	power_board->config->cur_pid_config.Kff = Kff;

	if (CyberDiverPowerBoard_UpdateConfig(power_board))
	{
		return 1;
	}
	else
	{
		/* revert the settings */
		power_board->config->cur_pid_config.Kp = Kp_old;
		power_board->config->cur_pid_config.Ki = Ki_old;
		power_board->config->cur_pid_config.Kd = Kd_old;
		power_board->config->cur_pid_config.Kff = Kff_old;
	}

	return 0;
}


void CyberDiverPowerBoard_GetGains(CyberDiverPowerBoard* power_board, float* Kp, float* Ki, float* Kd, float* Kff)
{
	*Kp = power_board->config->cur_pid_config.Kp;
	*Ki = power_board->config->cur_pid_config.Ki;
	*Kd = power_board->config->cur_pid_config.Kd;
	*Kff = power_board->config->cur_pid_config.Kff;
}


uint32_t CyberDiverPowerBoard_SetPeriod(CyberDiverPowerBoard* power_board, float T)
{
	/* inner control loop update period */
	/* WARNING: this should only be updated with scope connected to verify that the CPU can keep up */

	float T_old = power_board->config->cur_pid_config.T;

	uint32_t arr = CD_PB_TIM_COUNTS_PER_SECOND / (1.0f / T) / 2;
	power_board->config->cur_pid_config.T = (float)(2.0f * arr) / (float)(CD_PB_TIM_COUNTS_PER_SECOND);  /* update the value in the configuration so that it divides evenly into the timer frequency */

	if (CyberDiverPowerBoard_UpdateConfig(power_board))
	{
		return 1;
	}
	else
	{
		/* revert the settings */
		power_board->config->cur_pid_config.T = T_old;
	}


	return 0;
}


float CyberDiverPowerBoard_GetPeriod(CyberDiverPowerBoard* power_board)
{
	return power_board->config->cur_pid_config.T;
}


uint32_t CyberDiverPowerBoard_SetTau(CyberDiverPowerBoard* power_board, float tau)
{
	if (tau > 0)
	{
		float tau_old = power_board->config->cur_pid_config.tau;
		power_board->config->cur_pid_config.tau = tau;
		if (CyberDiverPowerBoard_UpdateConfig(power_board))
		{
			return 1;
		}
		else
		{
			/* revert the settings */
			power_board->config->cur_pid_config.tau = tau_old;
		}
	}

	return 0;
}


float CyberDiverPowerBoard_GetTau(CyberDiverPowerBoard* power_board)
{
	return power_board->config->cur_pid_config.tau;
}


uint32_t CyberDiverPowerBoard_SetKffvel(CyberDiverPowerBoard* power_board, float Kff_vel)
{
	float Kff_vel_old = power_board->config->Kff_vel;

	power_board->config->Kff_vel = Kff_vel;

	if (CyberDiverPowerBoard_UpdateConfig(power_board))
	{
		return 1;
	}
	else
	{
		/* revert the settings */
		power_board->config->Kff_vel = Kff_vel_old;
	}

	return 0;
}


float CyberDiverPowerBoard_GetKffvel(CyberDiverPowerBoard* power_board)
{
	return power_board->config->Kff_vel;
}


uint32_t CyberDiverPowerBoard_UpdateConfig(CyberDiverPowerBoard* power_board)
{
	/* send the most recent configuration to the power board */

	if (power_board->mode == POWER_BOARD_IDLE && power_board->mode_stable)
	{
		uint8_t rx_buf[sizeof(CyberDiverPowerBoard_Config)] = {0};  /* empty rx buffer */
		HAL_GPIO_WritePin(power_board->cs_port, power_board->cs_pin, GPIO_PIN_RESET);
		uint32_t ret = (HAL_OK == HAL_SPI_TransmitReceive(power_board->spi, (uint8_t*)(power_board->config), rx_buf, sizeof(CyberDiverPowerBoard_Config), HAL_MAX_DELAY));  /* just send the whole configuration object */
		HAL_GPIO_WritePin(power_board->cs_port, power_board->cs_pin, GPIO_PIN_SET);

		if (ret && !strcmp((char*)rx_buf, CD_PB_CONFIG_VALIDATION))  /* check that we successfully talked to the power board, rx_buf should contain the validation string */
			return 1;
	}

	return 0;
}
