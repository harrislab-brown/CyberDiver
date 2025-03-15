/*
 * cyber_diver_power_board.h
 *
 *  Created on: Jul 10, 2024
 *      Author: johnt
 */

#ifndef INC_CYBER_DIVER_POWER_BOARD_H_
#define INC_CYBER_DIVER_POWER_BOARD_H_

#include "main.h"
#include "pid.h"
#include <string.h>

#define CD_PB_CONFIG_VALIDATION "successfully configured power board"
#define CD_PB_CURRENT_SCALE (32767.0f / 5.5f)  /* scale the quantities we send over SPI and send as int16_t for speed */
#define CD_PB_VEL_SCALE (32767.0f / 2500.0f)
#define CD_PB_DUTY_SCALE (32767.0f / 1.0f)
#define CD_PB_TIM_COUNTS_PER_SECOND 84000000  /* for validation of the control loop update period */

typedef enum
{
	POWER_BOARD_IDLE,  /* in idle mode, the power board expects to receive the configuration parameters over SPI */
	POWER_BOARD_RUNNING  /* when running, the power board expect a current set-point command and returns the measured current */
} CyberDiverPowerBoardMode;

typedef struct
{
	int16_t setpoint_current_scaled;
	int16_t measured_velocity_scaled;
} CyberDiverPowerBoardTXData;

typedef struct
{
	int16_t measured_current_scaled;
	int16_t measured_duty_cycle_scaled;
} CyberDiverPowerBoardRXData;

typedef struct
{
	PIDController_Config cur_pid_config;

	float Kff_vel;  /* feed-forward gain for velocity disturbance */

} CyberDiverPowerBoard_Config;

typedef struct
{
	/* SPI handle */
	SPI_HandleTypeDef* spi;

	/* chip select pin */
	GPIO_TypeDef* cs_port;
	uint16_t cs_pin;

	/* mode select pin */
	/* this is OUTPUTS on the control board side and INPUT on the power board */
	/* the state of this pin sets the power board mode */
	GPIO_TypeDef* int1_port;
	uint16_t int1_pin;

	/* currently unused power board GPIO connection */
	/* This is INPUT on the control board side and OUTPUT on the power board */
	GPIO_TypeDef* int2_port;
	uint16_t int2_pin;

	/* operating mode of the power board */
	volatile CyberDiverPowerBoardMode mode;
	volatile uint32_t mode_stable;  /* true when the power board can be talked to without risking errors */

	/* pointer to object that stores configuration parameters */
	CyberDiverPowerBoard_Config* config;

	/* objects to hold data sent/received from the power board */
	/* since the control board controls exactly when the communication happens, no need to double buffer here because we always know the communication is done when accessing values */
	CyberDiverPowerBoardTXData tx_data;
	CyberDiverPowerBoardRXData rx_data;

	/* power board measurements */
	volatile float measured_current_amps;
	volatile float measured_duty_cycle;

} CyberDiverPowerBoard;

void CyberDiverPowerBoard_Init(CyberDiverPowerBoard* power_board,
		CyberDiverPowerBoard_Config* config,
		SPI_HandleTypeDef* spi,
		GPIO_TypeDef* cs_port,
		uint16_t cs_pin,
		GPIO_TypeDef* int1_port,
		uint16_t int1_pin,
		GPIO_TypeDef* int2_port,
		uint16_t int2_pin);

uint32_t CyberDiverPowerBoard_Reset(CyberDiverPowerBoard* power_board);

/* sets the mode of the power board object and updates the INT pin states accordingly */
void CyberDiverPowerBoard_SetMode(CyberDiverPowerBoard* power_board, CyberDiverPowerBoardMode mode);
CyberDiverPowerBoardMode CyberDiverPowerBoard_GetMode(CyberDiverPowerBoard* power_board);

/* sends a current command if the power board is in running mode, also send the velocity for state estimation */
void CyberDiverPowerBoard_SetCurrent(CyberDiverPowerBoard* power_board, float setpoint_current_amps, float measured_velocity_mmps);
float CyberDiverPowerBoard_GetCurrent(CyberDiverPowerBoard* power_board);
float CyberDiverPowerBoard_GetDutyCycle(CyberDiverPowerBoard* power_board);


uint32_t CyberDiverPowerBoard_SetGains(CyberDiverPowerBoard* power_board, float Kp, float Ki, float Kd, float Kff);
void CyberDiverPowerBoard_GetGains(CyberDiverPowerBoard* power_board, float* Kp, float* Ki, float* Kd, float* Kff);
uint32_t CyberDiverPowerBoard_SetPeriod(CyberDiverPowerBoard* power_board, float T);  /* inner control loop update period */
float CyberDiverPowerBoard_GetPeriod(CyberDiverPowerBoard* power_board);
uint32_t CyberDiverPowerBoard_SetTau(CyberDiverPowerBoard* power_board, float tau);
float CyberDiverPowerBoard_GetTau(CyberDiverPowerBoard* power_board);
uint32_t CyberDiverPowerBoard_SetKffvel(CyberDiverPowerBoard* power_board, float Kff_vel);
float CyberDiverPowerBoard_GetKffvel(CyberDiverPowerBoard* power_board);


/* sends the configuration parameters to the power board if it is in idle mode */
uint32_t CyberDiverPowerBoard_UpdateConfig(CyberDiverPowerBoard* power_board);


#endif /* INC_CYBER_DIVER_POWER_BOARD_H_ */
