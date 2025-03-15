/*
 * cyber_diver_accel_handler.c
 *
 *  Created on: Oct 28, 2024
 *      Author: johnt
 */

/*
 * Commands to implement for the accelerometer:
 *
 *
 * >>accel set range [range:int]
 *
 * >>accel get range
 *
 * >>accel set lpf [lpf:int]
 *
 * >>accel get lpf
 *
 * >>accel set offsets [x:float] [y:float] [z:float]
 *
 * >>accel get offsets
 *
 * >>accel get data
 *
 */

#include "shell_handlers/cyber_diver_accel_handler.h"

uint32_t CyberDiverAccelCMD_Execute(void* obj, CyberDiverShell* shell)
{

	IIS3DWB* ac = (IIS3DWB*)obj;

	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "set"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "range"))
				{
					int32_t range;
					if (CyberDiverShell_GetNextInt(shell, &range))
					{
						if (IIS3DWB_SetRange(ac, range))
						{
							CyberDiverShell_PutOutputString(shell, "ack");
							CyberDiverShell_PutOutputDelimiter(shell);
							return 1;
						}
					}
				}
				else if (!strcmp(str, "lpf"))
				{
					int32_t lpf;
					if (CyberDiverShell_GetNextInt(shell, &lpf))
					{
						if (IIS3DWB_SetLPF(ac, lpf))
						{
							CyberDiverShell_PutOutputString(shell, "ack");
							CyberDiverShell_PutOutputDelimiter(shell);
							return 1;
						}
					}
				}
				else if (!strcmp(str, "offsets"))
				{
					float x, y, z;
					if (CyberDiverShell_GetNextFloat(shell, &x) &&
							CyberDiverShell_GetNextFloat(shell, &y) &&
							CyberDiverShell_GetNextFloat(shell, &z))
					{
						if (IIS3DWB_SetOffsets(ac, x, y, z))
						{
							CyberDiverShell_PutOutputString(shell, "ack");
							CyberDiverShell_PutOutputDelimiter(shell);
							return 1;
						}
					}
				}
			}
		}
		else if (!strcmp(str, "get"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "range"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, IIS3DWB_GetRange(ac));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "lpf"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, IIS3DWB_GetLPF(ac));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "offsets"))
				{
					float x, y, z;
					IIS3DWB_GetOffsets(ac, &x, &y, &z);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, x);
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, y);
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, z);
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "data"))
				{
					float x, y, z;
					IIS3DWB_GetAcceleration(ac, &x, &y, &z);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, x);
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, y);
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, z);
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
	}

	return 0;
}
