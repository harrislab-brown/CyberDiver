/*
 * cyber_diver_logger.c
 *
 *  Created on: Jul 8, 2024
 *      Author: johnt
 */

#include "cyber_diver_logger.h"


void CyberDiverLogger_Init(CyberDiverLogger* logger, CyberDiverLogger_Config* config, uint32_t data_point_length)
{
	logger->config = config;
	logger->data_point_length = data_point_length;

	CyberDiverLogger_Reset(logger);
}


void CyberDiverLogger_Reset(CyberDiverLogger* logger)
{
	logger->is_logging = 0;

	if (logger->config->location == LOGGER_LOC_MCU)
	{
		logger->mem_start = logger->mcu_buf;
		logger->mem_size = CD_LOGGER_MCURAM_SIZE / 4;
	}
	else if (logger->config->location == LOGGER_LOC_SDRAM)
	{
		logger->mem_start = (uint32_t*)CD_LOGGER_SDRAM_ADDRESS_START;
		logger->mem_size = CD_LOGGER_SDRAM_SIZE / 4;
	}

	/* make sure the memory size is an integer number of packets */
	logger->mem_size -= logger->mem_size % (logger->data_point_length * logger->config->packet_size);
	logger->mem_end = logger->mem_start + logger->mem_size;

	logger->head = logger->mem_start;
	logger->tail = logger->mem_start;
	logger->count = 0;
	logger->count_per_packet = logger->data_point_length * logger->config->packet_size;

	logger->decimation_count = 0;

}


void CyberDiverLogger_StartLogging(CyberDiverLogger* logger)
{
	logger->is_logging = 1;
}


void CyberDiverLogger_StopLogging(CyberDiverLogger* logger)
{
	logger->is_logging = 0;
}


void CyberDiverLogger_AddData(CyberDiverLogger* logger, uint32_t* data)
{
	/* this will add a data point to the data buffer, the number of bytes it will take is set by the data point size */

	if (logger->is_logging && logger->decimation_count++ == logger->config->decimation)
	{
		logger->decimation_count = 0;

		for (uint32_t i = 0; i < logger->data_point_length; i++)
			*(volatile uint32_t*)(logger->head++) = *data++;  /* write to the memory */

		if (logger->head == logger->mem_end)  /* wrap the head */
			logger->head = logger->mem_start;

		if (logger->count < logger->mem_size)
			logger->count += logger->data_point_length;  /* update the count */
		else
			logger->tail = logger->head;  /* the buffer is full so we need to update the tail */

	}

}



uint32_t CyberDiverLogger_GetAvailable(CyberDiverLogger* logger)
{
	/* returns the number of complete packets available */
	return logger->count / logger->data_point_length / logger->config->packet_size;
}


uint32_t CyberDiverLogger_GetPacket(CyberDiverLogger* logger, volatile uint8_t** data, uint32_t* size)
{
	/* gives a pointer to oldest packet in the buffer and how bytes it contains, returns true if there is a packet available (even if incomplete) */

	/*
	 * this likely won't work correctly if called while the buffer is overflowing and new data is being added in an interrupt, but this isn't the intended use case
	 */
	if (CyberDiverLogger_GetCompletePacket(logger, data, size))
	{
		return 1;
	}
	else if (logger->count)
	{
		/* we have some data but it is not a complete packet - so return all of the remaining data */
		*data = (uint8_t*)logger->tail;
		*size = logger->count * 4;  /* size in bytes */

		logger->tail += logger->count;
		if (logger->tail == logger->mem_end)
			logger->tail = logger->mem_start;
		logger->count = 0;

		return 1;
	}

	return 0;
}



uint32_t CyberDiverLogger_GetCompletePacket(CyberDiverLogger* logger, volatile uint8_t** data, uint32_t* size)
{
	/* only returns true if we have a complete packet in the buffer */
	/* use this function when writing to the SD card during an experiment */

	if (logger->count >= logger->count_per_packet)
	{
		*data = (uint8_t*)logger->tail;
		*size = logger->count_per_packet * 4;  /* size in bytes */

		logger->count -= logger->count_per_packet;
		logger->tail += logger->count_per_packet;
		if (logger->tail == logger->mem_end)
			logger->tail = logger->mem_start;

		return 1;
	}

	return 0;
}


void CyberDiverLogger_SetDecimation(CyberDiverLogger* logger, uint32_t decimation)
{
	logger->config->decimation = decimation;
	CyberDiverLogger_Reset(logger);
}

uint32_t CyberDiverLogger_GetDecimation(CyberDiverLogger* logger)
{
	return logger->config->decimation;
}

void CyberDiverLogger_SetLocation(CyberDiverLogger* logger, CyberDiverLogger_MemLoc loc)
{
	logger->config->location = (uint32_t)loc;
	CyberDiverLogger_Reset(logger);
}

CyberDiverLogger_MemLoc CyberDiverLogger_GetLocation(CyberDiverLogger* logger)
{
	return logger->config->location;
}

void CyberDiverLogger_SetPacketSize(CyberDiverLogger* logger, uint32_t size)
{
	/* this is the packet size in data points */
	logger->config->packet_size = size;
	CyberDiverLogger_Reset(logger);
}

uint32_t CyberDiverLogger_GetPacketSize(CyberDiverLogger* logger)
{
	return logger->config->packet_size;
}



void CyberDiverLogger_TestSDRAM(CyberDiverLogger* logger)
{
	/*
	 * This function isn't called anywhere, but can be used to
	 * test the SDRAM speed and check that we can successfully
	 * write data using the memory monitor
	 */

	uint32_t fmc_test_start, fmc_test_time;
	(void)fmc_test_time;

	/* erase the SDRAM memory using 8 bit writes */
	fmc_test_start = HAL_GetTick();

	for (uint32_t i = 0; i < CD_LOGGER_SDRAM_SIZE; i++)
		*(volatile uint8_t*)(CD_LOGGER_SDRAM_ADDRESS_START + i) = (uint8_t)0;

	fmc_test_time = HAL_GetTick() - fmc_test_start;

	HAL_Delay(50);


	/* write to the memory using 16 bit writes */
	fmc_test_start = HAL_GetTick();

	for (uint32_t i = 0; i < CD_LOGGER_SDRAM_SIZE / 2; i++)
		*(volatile uint16_t*)(CD_LOGGER_SDRAM_ADDRESS_START + 2 * i) = (uint16_t)i;

	fmc_test_time = HAL_GetTick() - fmc_test_start;

	HAL_Delay(50);


	/* write to the memory using 32 bit writes */
	fmc_test_start = HAL_GetTick();

	for (uint32_t i = 0; i < CD_LOGGER_SDRAM_SIZE / 4; i++)
		*(volatile uint32_t*)(CD_LOGGER_SDRAM_ADDRESS_START + 4 * i) = (uint32_t)i;

	fmc_test_time = HAL_GetTick() - fmc_test_start;

	HAL_Delay(50);
}



