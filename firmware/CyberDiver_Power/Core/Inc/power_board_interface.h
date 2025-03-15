/*
 * power_board_interface.h
 *
 *  Created on: Sep 11, 2024
 *      Author: johnt
 */

#ifndef INC_POWER_BOARD_INTERFACE_H_
#define INC_POWER_BOARD_INTERFACE_H_


#include "stm32f4xx_hal.h"
#include "power_board_config.h"
#include <string.h>


#define PB_CONFIG_VALIDATION "successfully configured power board"  /* power board configuration is 36 bytes */
#define PB_CURRENT_SCALE (32767.0f / 5.5f)  /* scale the quantities we send over SPI and send as int16_t for speed */
#define PB_VEL_SCALE (32767.0f / 2500.0f)
#define PB_DUTY_SCALE (32767.0f / 1.0f)

/* SPI + GPIO interface with the control board */

typedef enum
{
	PB_IDLE,  /* coil disabled, waiting for configuration from control board */
	PB_RUNNING  /* coil enabled, running the control loop */
} PowerBoardInterfaceMode;

typedef struct
{
	int16_t current_scaled;
	int16_t duty_cycle_scaled;
} PowerBoardInterface_DataTX;

typedef struct
{
	int16_t current_scaled;
	int16_t velocity_scaled;
} PowerBoardInterface_DataRX;

typedef struct
{
	volatile PowerBoardInterfaceMode mode;
	volatile uint32_t mode_changed;

	SPI_HandleTypeDef* hspi;

	/* data handled by the interface */
	volatile float setpoint_current_amps;
	volatile float measured_current_amps;
	volatile float velocity_mmps;  /* comes from control board */
	volatile float duty_cycle;
	volatile PowerBoardConfig config;

	/* double buffer the data to be sent or received to avoid race conditions */
	volatile uint32_t spi_index;
	PowerBoardInterface_DataTX data_tx[2];
	PowerBoardInterface_DataRX data_rx[2];

	/* also can send or receive configuration data */
	PowerBoardConfig config_tx[2];  /* populate the config tx with some fixed values, so the control board can tell if it sent a configuration successfully */
	PowerBoardConfig config_rx[2];

} PowerBoardInterface;

void PowerBoardInterface_Init(PowerBoardInterface* interface, SPI_HandleTypeDef* hspi);

float PowerBoardInterface_GetSetpoint(PowerBoardInterface* interface);
float PowerBoardInterface_GetVelocity(PowerBoardInterface* interface);
PowerBoardConfig PowerBoardInterface_GetConfig(PowerBoardInterface* interface);
PowerBoardInterfaceMode PowerBoardInterface_GetMode(PowerBoardInterface* interface);
uint32_t PowerBoardInterface_ModeChanged(PowerBoardInterface* interface);  /* returns true when the interface mode was changed */
void PowerBoardInterface_SetData(PowerBoardInterface* interface, float measured_current, float duty_cycle);  /* sets the data to send back to the control board on the next SPI transaction */

void PowerBoardInterface_SetMode(PowerBoardInterface* interface, PowerBoardInterfaceMode mode);  /* called in EXTI callback based on control board GPIO command pin */
void PowerBoardInterface_ListenForSPI(PowerBoardInterface* interface);  /* re-initialize SPI depending on the mode to receive configuration or send/receive controller data */
void PowerBoardInterface_SPIComplete(PowerBoardInterface* interface);  /* call this in the SPI DMA half and full call-backs */

#endif /* INC_POWER_BOARD_INTERFACE_H_ */
