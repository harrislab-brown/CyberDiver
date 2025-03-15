/*
 * cyber_diver_logger_handler.c
 *
 *  Created on: Oct 30, 2024
 *      Author: johnt
 */



/*
 * Commands to implement for the logger:
 *
 *
 *
 * >>logger start
 *
 * >>logger stop
 *
 * >>logger stream start
 *
 * >>logger stream stop
 *
 * >>logger get packet
 *
 * >>logger get available
 *
 * >>logger set decimation [decimation:int]
 *
 * >>logger get decimation
 *
 * >>logger set location [location:str]
 *
 * >>logger get location
 *
 * >>logger set packetsize [size:int]
 *
 * >>logger get packetsize
 *
 *
 */


#include "shell_handlers/cyber_diver_logger_handler.h"


static uint32_t is_streaming = 0;


uint32_t CyberDiverLoggerCMD_Execute(void* obj, CyberDiverShell* shell)
{
	CyberDiverLogger* logger = (CyberDiverLogger*)obj;

	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "start"))
		{
			CyberDiverLogger_StartLogging(logger);
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "stop"))
		{
			CyberDiverLogger_StopLogging(logger);
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "stream"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "start"))
				{
					is_streaming = 1;
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "stop"))
				{
					is_streaming = 0;
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "set"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "decimation"))
				{
					int32_t decimation;
					if (CyberDiverShell_GetNextInt(shell, &decimation))
					{
						CyberDiverLogger_SetDecimation(logger, decimation);
						CyberDiverShell_PutOutputString(shell, "ack");
						CyberDiverShell_PutOutputDelimiter(shell);
						return 1;
					}
				}
				else if (!strcmp(str, "location"))
				{
					if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
					{
						CyberDiverLogger_MemLoc loc;
						if (!strcmp(str, "mcu"))
							loc = LOGGER_LOC_MCU;
						else if (!strcmp(str, "sdram"))
							loc = LOGGER_LOC_SDRAM;
						else
							return 0;

						CyberDiverLogger_SetLocation(logger, loc);
						CyberDiverShell_PutOutputString(shell, "ack");
						CyberDiverShell_PutOutputDelimiter(shell);
						return 1;
					}
				}
				else if (!strcmp(str, "packetsize"))
				{
					int32_t size;
					if (CyberDiverShell_GetNextInt(shell, &size))
					{
						CyberDiverLogger_SetPacketSize(logger, size);
						CyberDiverShell_PutOutputString(shell, "ack");
						CyberDiverShell_PutOutputDelimiter(shell);
						return 1;
					}
				}
			}
		}
		else if (!strcmp(str, "get"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "decimation"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, CyberDiverLogger_GetDecimation(logger));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "location"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);

					CyberDiverLogger_MemLoc loc = CyberDiverLogger_GetLocation(logger);
					switch (loc)
					{
					case LOGGER_LOC_MCU:
						CyberDiverShell_PutOutputString(shell, "mcu");
						break;
					case LOGGER_LOC_SDRAM:
						CyberDiverShell_PutOutputString(shell, "sdram");
						break;
					}

					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "packetsize"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, CyberDiverLogger_GetPacketSize(logger));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "packet"))
				{
					volatile uint8_t* data;
					uint32_t size;
					if (CyberDiverLogger_GetPacket(logger, &data, &size))
					{
						uint32_t sz = sizeof(CyberDiverControllerDataPoint);
						uint32_t num_data_points = size / sz;

						CyberDiverShell_PutOutputString(shell, "data");
						CyberDiverShell_PutOutputSeparator(shell);
						CyberDiverShell_PutOutputUint(shell, num_data_points);

						CyberDiverControllerDataPoint dp;
						for (uint32_t i = 0; i < num_data_points; i++)
						{
							/* encode the data points */
							memcpy(&dp, (uint8_t*)data, sz);
							data += sz;
							CyberDiverControllerHandler_SendData(&dp, shell);
						}

						CyberDiverShell_PutOutputDelimiter(shell);
						return 1;
					}
				}
				else if (!strcmp(str, "available"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, CyberDiverLogger_GetAvailable(logger));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
	}

	return 0;
}




uint32_t CyberDiverLoggerSender_Execute(void* obj, CyberDiverShell* shell)
{
	CyberDiverLogger* logger = (CyberDiverLogger*)obj;

	volatile uint8_t* data;
	uint32_t size;
	if (is_streaming && CyberDiverLogger_GetCompletePacket(logger, &data, &size))
	{
		uint32_t sz = sizeof(CyberDiverControllerDataPoint);
		uint32_t num_data_points = size / sz;

		CyberDiverShell_PutOutputString(shell, "data");
		CyberDiverShell_PutOutputSeparator(shell);
		CyberDiverShell_PutOutputUint(shell, num_data_points);

		CyberDiverControllerDataPoint dp;
		for (uint32_t i = 0; i < num_data_points; i++)
		{
			/* encode the data points */
			memcpy(&dp, (uint8_t*)data, sz);
			data += sz;
			CyberDiverControllerHandler_SendData(&dp, shell);
		}

		CyberDiverShell_PutOutputDelimiter(shell);
		return 1;
	}

	return 0;
}
