/*
 * cyber_diver_logger.h
 *
 *  Created on: Jul 8, 2024
 *      Author: johnt
 */

#ifndef INC_CYBER_DIVER_LOGGER_H_
#define INC_CYBER_DIVER_LOGGER_H_

/* double-buffered data logger for the CyberDiver, stores the data buffer for writing to SD card */

#include <stdint.h>
#include "main.h"


#define CD_LOGGER_MCURAM_SIZE 0x10000  /* how many bytes we can use for the data buffer in the MCU memory */
#define CD_LOGGER_SDRAM_ADDRESS_START 0xC0000000  /* STM32H723 memory map */
#define CD_LOGGER_SDRAM_SIZE 0x1000000  /* number of bytes in the external ram chip */


typedef enum
{
	LOGGER_LOC_MCU,
	LOGGER_LOC_SDRAM
} CyberDiverLogger_MemLoc;

typedef struct
{
	uint32_t decimation;  /* skip logging samples at interval to reduce data transfer rate requirement */
	uint32_t location;  /* whether to put the data buffer in local memory or on the external SDRAM */
	uint32_t packet_size;  /* size of each packet in data points */
} CyberDiverLogger_Config;

typedef struct
{
	CyberDiverLogger_Config* config;

	/* memory buffer in local RAM */
	volatile uint32_t mcu_buf[CD_LOGGER_MCURAM_SIZE / 4];  /* we are assuming all of our data has 4 bytes size */
	volatile uint32_t* head;
	volatile uint32_t* tail;
	volatile uint32_t count;  /* amount of data in the buffer */
	volatile uint32_t* mem_start;
	volatile uint32_t* mem_end;
	uint32_t mem_size;  /* size of the currently used memory */

	uint32_t data_point_length;  /* number of values in a data point */
	uint32_t count_per_packet;  /* number of values in a packet */

	uint32_t is_logging;
	volatile uint32_t decimation_count;

} CyberDiverLogger;

void CyberDiverLogger_Init(CyberDiverLogger* logger, CyberDiverLogger_Config* config, uint32_t data_point_length);

void CyberDiverLogger_Reset(CyberDiverLogger* logger);

void CyberDiverLogger_StartLogging(CyberDiverLogger* logger);  /* start logging data to the buffer in memory */
void CyberDiverLogger_StopLogging(CyberDiverLogger* logger);

void CyberDiverLogger_AddData(CyberDiverLogger* logger, uint32_t* data);  /* this will add data to the data buffer, the number of bytes it will take is set by the data point size */


uint32_t CyberDiverLogger_GetAvailable(CyberDiverLogger* logger);  /* returns the number of packets available */
uint32_t CyberDiverLogger_GetPacket(CyberDiverLogger* logger, volatile uint8_t** data, uint32_t* size);  /* gives a pointer to the memory and a size in bytes, returns true if there is a packet available */
uint32_t CyberDiverLogger_GetCompletePacket(CyberDiverLogger* logger, volatile uint8_t** data, uint32_t* size);


void CyberDiverLogger_SetDecimation(CyberDiverLogger* logger, uint32_t decimation);
uint32_t CyberDiverLogger_GetDecimation(CyberDiverLogger* logger);
void CyberDiverLogger_SetLocation(CyberDiverLogger* logger, CyberDiverLogger_MemLoc loc);
CyberDiverLogger_MemLoc CyberDiverLogger_GetLocation(CyberDiverLogger* logger);
void CyberDiverLogger_SetPacketSize(CyberDiverLogger* logger, uint32_t size);  /* this is the packet size in data points */
uint32_t CyberDiverLogger_GetPacketSize(CyberDiverLogger* logger);



void CyberDiverLogger_TestSDRAM(CyberDiverLogger* logger);



#endif /* INC_CYBER_DIVER_LOGGER_H_ */
