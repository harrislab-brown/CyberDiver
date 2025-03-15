/*
 * cyber_diver_power_board_handler.c
 *
 *  Created on: Oct 28, 2024
 *      Author: johnt
 */

/*
 * Commands to implement for the power board:
 *
 * >>powerboard set mode [idle, running]
 *
 * >>powerboard get mode
 *
 * >>powerboard set current [current:float]
 *
 * >>powerboard get current
 *
 * >>powerboard get duty
 *
 * >>powerboard config set gains [P:float] [I:float] [D:float] [FF:float]
 *
 * >>powerboard config get gains
 *
 * >>powerboard config set period [T:float]
 *
 * >>powerboard config get period
 *
 * >>powerboard config set tau [tau:float]
 *
 * >>powerboard config get tau
 *
 * >>powerboard config set kffvel [FF:float]
 *
 * >>powerboard config get kffvel
 *
 */


#include "shell_handlers/cyber_diver_power_board_handler.h"



static uint32_t CyberDiverPowerBoardCMD_ConfigSet(CyberDiverPowerBoard* pb, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "gains"))
		{
			float val[4];  /* expecting to get 4 numeric values for the gains */
			uint32_t i;
			for (i = 0; i < 4; i++)
			{
				if (!CyberDiverShell_GetNextFloat(shell, &val[i]))
					break;
			}

			if (i == 4)
			{
				/* we got all our values */
				if (CyberDiverPowerBoard_SetGains(pb, val[0], val[1], val[2], val[3]))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "period"))
		{
			float T;
			if (CyberDiverShell_GetNextFloat(shell, &T))
			{
				if (CyberDiverPowerBoard_SetPeriod(pb, T))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "tau"))
		{
			float tau;
			if (CyberDiverShell_GetNextFloat(shell, &tau))
			{
				if (CyberDiverPowerBoard_SetTau(pb, tau))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "kffvel"))
		{
			float kff;
			if (CyberDiverShell_GetNextFloat(shell, &kff))
			{
				if (CyberDiverPowerBoard_SetKffvel(pb, kff))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
	}


	return 0;
}


static uint32_t CyberDiverPowerBoardCMD_ConfigGet(CyberDiverPowerBoard* pb, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "gains"))
		{
			float Kp, Ki, Kd, Kff;
			CyberDiverPowerBoard_GetGains(pb, &Kp, &Ki, &Kd, &Kff);

			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, Kp);
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, Ki);
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, Kd);
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, Kff);
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "period"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverPowerBoard_GetPeriod(pb));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "tau"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverPowerBoard_GetTau(pb));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "kffvel"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverPowerBoard_GetKffvel(pb));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
	}

	return 0;
}


uint32_t CyberDiverPowerBoardCMD_Execute(void* obj, CyberDiverShell* shell)
{
	CyberDiverPowerBoard* pb = (CyberDiverPowerBoard*)obj;

	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "set"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "mode"))
				{
					if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
					{
						if (!strcmp(str, "idle"))
						{
							CyberDiverPowerBoard_SetMode(pb, POWER_BOARD_IDLE);
							CyberDiverShell_PutOutputString(shell, "ack");
							CyberDiverShell_PutOutputDelimiter(shell);
							return 1;
						}
						else if (!strcmp(str, "running"))
						{
							CyberDiverPowerBoard_SetMode(pb, POWER_BOARD_RUNNING);
							CyberDiverShell_PutOutputString(shell, "ack");
							CyberDiverShell_PutOutputDelimiter(shell);
							return 1;
						}
					}
				}
				else if (!strcmp(str, "current"))
				{
					float current;
					if (CyberDiverShell_GetNextFloat(shell, &current))
					{
						CyberDiverPowerBoard_SetCurrent(pb, current, 0);  /* assume zero velocity here so it won't use feed-forward back EMF rejection */
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
				if (!strcmp(str, "mode"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);

					switch (CyberDiverPowerBoard_GetMode(pb))
					{
					case POWER_BOARD_IDLE:
						CyberDiverShell_PutOutputString(shell, "idle");
						break;
					case POWER_BOARD_RUNNING:
						CyberDiverShell_PutOutputString(shell, "running");
						break;
					}

					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "current"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, CyberDiverPowerBoard_GetCurrent(pb));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "duty"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, CyberDiverPowerBoard_GetDutyCycle(pb));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "config"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "set"))
				{
					if (CyberDiverPowerBoardCMD_ConfigSet(pb, shell))
						return 1;
				}
				else if (!strcmp(str, "get"))
				{
					if (CyberDiverPowerBoardCMD_ConfigGet(pb, shell))
						return 1;
				}
			}
		}
	}

	return 0;
}
