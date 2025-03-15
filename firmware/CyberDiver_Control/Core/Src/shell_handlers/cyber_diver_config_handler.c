/*
 * cyber_diver_config_handler.c
 *
 *  Created on: Oct 30, 2024
 *      Author: johnt
 */


/*
 * Commands dealing with the configuration file:
 *
 * >>config save
 *
 * >>config load
 *
 * >>config get version
 *
 * >>config set state idle [mode:str] [setpoint:float] [led:int]
 *
 * >>config get state idle
 *
 * >>config set state armed [mode:str] [setpoint:float] [led:int]
 *
 * >>config get state armed
 *
 * >>config set time staging [time:int]
 *
 * >>config get time staging
 *
 * >>config set time running [time:int]
 *
 * >>config get time running
 *
 * >>config set trigger [accel:float]
 *
 * >>config get trigger
 *
 *
 */


#include "shell_handlers/cyber_diver_config_handler.h"

uint32_t CyberDiverConfigCMD_Set(CyberDiverConfig* config, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "state"))
		{
			char state_str[CD_SHELL_MAX_TOKEN_LEN];
			char mode_str[CD_SHELL_MAX_TOKEN_LEN];
			if (CyberDiverShell_GetNextString(shell, state_str, CD_SHELL_MAX_TOKEN_LEN)
					&& CyberDiverShell_GetNextString(shell, mode_str, CD_SHELL_MAX_TOKEN_LEN))
			{
				CyberDiverControllerState* state;
				if (!strcmp(state_str, "idle"))
					state = &config->idle_controller_state;
				else if (!strcmp(state_str, "armed"))
					state = &config->armed_controller_state;
				else
					return 0;

				CyberDiverControllerMode mode;
				if (!strcmp(mode_str, "idle"))
					mode = CONTROLLER_IDLE;
				else if (!strcmp(mode_str, "current_control"))
					mode = CONTROLLER_CURRENT_CONTROL;
				else if (!strcmp(mode_str, "force_control"))
					mode = CONTROLLER_FORCE_CONTROL;
				else if (!strcmp(mode_str, "position_control"))
					mode = CONTROLLER_POSITION_CONTROL;
				else if (!strcmp(mode_str, "simulated_structure"))
					mode = CONTROLLER_SIMULATED_STRUCTURE;
				else
					return 0;

				float setpoint;
				int32_t led;
				if (CyberDiverShell_GetNextFloat(shell, &setpoint) && CyberDiverShell_GetNextInt(shell, &led))
				{
					led = led ? 1 : 0;
					state->setpoint = setpoint;
					state->status = mode | (led << 31);  /* not good */
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "time"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				uint32_t* time;
				if (!strcmp(str, "staging"))
					time = &config->staging_time_us;
				else if (!strcmp(str, "running"))
					time = &config->running_time_us;
				else
					return 0;

				if (CyberDiverShell_GetNextInt(shell, (int32_t*)time))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "trigger"))
		{
			float trigger;
			if (CyberDiverShell_GetNextFloat(shell, &trigger))
			{
				config->trigger_accel_threshold = trigger;
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
	}

	return 0;
}

uint32_t CyberDiverConfigCMD_Get(CyberDiverConfig* config, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "version"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputUint(shell, config->firmware_version);
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "state"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				CyberDiverControllerState* state;
				if (!strcmp(str, "idle"))
					state = &config->idle_controller_state;
				else if (!strcmp(str, "armed"))
					state = &config->armed_controller_state;
				else
					return 0;

				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputSeparator(shell);

				switch (state->status & (~(1 << 31)))  /* this is crazy */
				{
				case CONTROLLER_IDLE:
					CyberDiverShell_PutOutputString(shell, "idle");
					break;
				case CONTROLLER_CURRENT_CONTROL:
					CyberDiverShell_PutOutputString(shell, "current_control");
					break;
				case CONTROLLER_FORCE_CONTROL:
					CyberDiverShell_PutOutputString(shell, "force_control");
					break;
				case CONTROLLER_POSITION_CONTROL:
					CyberDiverShell_PutOutputString(shell, "position_control");
					break;
				case CONTROLLER_SIMULATED_STRUCTURE:
					CyberDiverShell_PutOutputString(shell, "simulated_structure");
					break;
				}

				CyberDiverShell_PutOutputSeparator(shell);
				CyberDiverShell_PutOutputFloat(shell, state->setpoint);
				CyberDiverShell_PutOutputSeparator(shell);
				CyberDiverShell_PutOutputUint(shell, state->status >> 31);
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "time"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "staging"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, config->staging_time_us);
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "running"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, config->running_time_us);
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "trigger"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, config->trigger_accel_threshold);
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
	}

	return 0;
}
