/*
 * power_board_interface.c
 *
 *  Created on: Sep 11, 2024
 *      Author: johnt
 */

#include "power_board_interface.h"


void PowerBoardInterface_Init(PowerBoardInterface* interface, SPI_HandleTypeDef* hspi)
{
	/* set up member variables */
	interface->mode = PB_IDLE;
	interface->mode_changed = 0;
	interface->hspi = hspi;

	/* put validation string in the config tx buffer */
	char validation_str[] = PB_CONFIG_VALIDATION;
	memcpy(&interface->config_tx[0], validation_str, sizeof(PowerBoardConfig));
	memcpy(&interface->config_tx[1], validation_str, sizeof(PowerBoardConfig));

	PowerBoardInterface_ListenForSPI(interface);
}

float PowerBoardInterface_GetSetpoint(PowerBoardInterface* interface)
{
	return interface->setpoint_current_amps;
}

float PowerBoardInterface_GetVelocity(PowerBoardInterface* interface)
{
	return interface->velocity_mmps;
}

PowerBoardConfig PowerBoardInterface_GetConfig(PowerBoardInterface* interface)
{
	return interface->config;
}

PowerBoardInterfaceMode PowerBoardInterface_GetMode(PowerBoardInterface* interface)
{
	return interface->mode;
}

uint32_t PowerBoardInterface_ModeChanged(PowerBoardInterface* interface)  /* returns true when the interface mode was changed */
{

	if (interface->mode_changed)
	{
		interface->mode_changed = 0;
		return 1;
	}

	return 0;
}

void PowerBoardInterface_SetData(PowerBoardInterface* interface, float measured_current, float duty_cycle)  /* sets the data to send back to the control board on the next SPI transaction */
{
	interface->measured_current_amps = measured_current;
	interface->duty_cycle = duty_cycle;
}

void PowerBoardInterface_SetMode(PowerBoardInterface* interface, PowerBoardInterfaceMode mode)  /* called in EXTI callback based on control board GPIO command pin */
{
	interface->mode = mode;
	interface->mode_changed = 1;
	PowerBoardInterface_ListenForSPI(interface);
}

void PowerBoardInterface_ListenForSPI(PowerBoardInterface* interface)  /* re-initialize SPI depending on the mode to receive configuration or send/receive controller data */
{
	/* reset the SPI to listen for the correct commands (this is a bit janky) */
	/* https://community.st.com/t5/stm32-mcus-products/restart-spi-dma-transmission/td-p/637909 */
	HAL_SPI_DMAStop(interface->hspi);
	__HAL_RCC_SPI2_FORCE_RESET();  /* !!! Which SPI to use is hard coded here !!! */
	__HAL_RCC_SPI2_RELEASE_RESET();
	HAL_SPI_Init(interface->hspi);

	interface->spi_index = 0;

	if (interface->mode == PB_IDLE)
	{
		HAL_SPI_TransmitReceive_DMA(interface->hspi, (uint8_t*)interface->config_tx, (uint8_t*)interface->config_rx, 2 * sizeof(PowerBoardConfig));
	}
	else if (interface->mode == PB_RUNNING)
	{
		/* Be careful to keep the tx and rx data buffers the same size in bytes */
		HAL_SPI_TransmitReceive_DMA(interface->hspi, (uint8_t*)interface->data_tx, (uint8_t*)interface->data_rx, 2 * sizeof(PowerBoardInterface_DataRX));
	}
}

void PowerBoardInterface_SPIComplete(PowerBoardInterface* interface)  /* call this in the SPI DMA half and full call-backs */
{
	/* copy data from the SPI buffer into local class variables to avoid race conditions */
	if (interface->mode == PB_IDLE)
	{
		interface->config = interface->config_rx[interface->spi_index];
	}
	else if (interface->mode == PB_RUNNING)
	{
		/* read the most recent data from the control board */
		interface->setpoint_current_amps = (float)interface->data_rx[interface->spi_index].current_scaled / PB_CURRENT_SCALE;
		interface->velocity_mmps = (float)interface->data_rx[interface->spi_index].velocity_scaled / PB_VEL_SCALE;

		/* update the outgoing SPI buffers with most recent data */
		interface->data_tx[interface->spi_index].current_scaled = (int16_t)(interface->measured_current_amps * PB_CURRENT_SCALE);
		interface->data_tx[interface->spi_index].duty_cycle_scaled = (int16_t)(interface->duty_cycle * PB_DUTY_SCALE);
	}

	interface->spi_index = !interface->spi_index;  /* keep track of which part in the double buffer is safe to read/write */
}


