/*
 * cyber_diver_controller_handler.c
 *
 *  Created on: Oct 29, 2024
 *      Author: johnt
 */


/*
 * Commands to implement for the CyberDiver controller:
 *
 *
 * >>controller set mode [mode:str]
 *
 * >>controller get mode
 *
 * >>controller set setpoint [setpoint:float]
 *
 * >>controller get setpoint
 *
 * >>controller set led [led:int]
 *
 * >>controller get led
 *
 * >>controller set state [mode:str] [setpoint:float] [led:int]
 *
 * >>controller get state
 *
 * >>controller get position
 *
 * >>controller get velocity
 *
 * >>controller get current
 *
 * >>controller get force
 *
 * >>controller get duty
 *
 * >>controller get accel
 *
 * >>controller get output
 *
 * >>controller get data
 *
 * >>controller resettime
 *
 * >>controller sequence start
 *
 * >>controller sequence stop
 *
 * >>controller sequence clear
 *
 * >>controller sequence set length [length:int]
 *
 * >>controller sequence get length
 *
 * >>controller sequence set looping [looping:int]
 *
 * >>controller sequence get looping
 *
 * >>controller sequence set step [index:int] [time:int] [mode:str] [setpoint:float] [led:int]
 *
 * >>controller sequence get step [index:int]
 *
 * >>controller config set gains [P:float] [I:float] [D:float] [FF:float]
 *
 * >>controller config get gains
 *
 * >>controller config set tau [tau:float]
 *
 * >>controller config get tau
 *
 * >>controller config set period [T:float]
 *
 * >>controller config get period
 *
 * >>controller config set limits [min:float] [max:float]
 *
 * >>controller config get limits
 *
 * >>controller config set calibration coil [order:int] [min:float] [max:float] [fastcompute:int] [coeff0:float] [coeff1:float] …
 *
 * >>controller config get calibration coil
 *
 * >>controller config set calibration passive [order:int] [min:float] [max:float] [fastcompute:int] [coeff0:float] [coeff1:float] …
 *
 * >>controller config get calibration passive
 *
 * >>controller config set calibration structure [order:int] [min:float] [max:float] [fastcompute:int] [coeff0:float] [coeff1:float] …
 *
 * >>controller config get calibration structure
 *
 *>>controller config set damping [damping:float]
 *
 *>>controller config get damping
 *
 */


#include "shell_handlers/cyber_diver_controller_handler.h"



static uint32_t CyberDiverControllerCMD_Set(CyberDiverController* controller, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "state"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{

				CyberDiverControllerMode mode;
				if (!strcmp(str, "idle"))
					mode = CONTROLLER_IDLE;
				else if (!strcmp(str, "current_control"))
					mode = CONTROLLER_CURRENT_CONTROL;
				else if (!strcmp(str, "force_control"))
					mode = CONTROLLER_FORCE_CONTROL;
				else if (!strcmp(str, "position_control"))
					mode = CONTROLLER_POSITION_CONTROL;
				else if (!strcmp(str, "simulated_structure"))
					mode = CONTROLLER_SIMULATED_STRUCTURE;
				else
					return 0;

				float setpoint;
				int32_t led;
				if (CyberDiverShell_GetNextFloat(shell, &setpoint) && CyberDiverShell_GetNextInt(shell, &led))
				{
					CyberDiverController_SetMode(controller, mode);
					CyberDiverController_SetSetpoint(controller, setpoint);
					CyberDiverController_SetLED(controller, led);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}

			}
		}
		else if (!strcmp(str, "mode"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "idle"))
				{
					CyberDiverController_SetMode(controller, CONTROLLER_IDLE);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "current_control"))
				{
					CyberDiverController_SetMode(controller, CONTROLLER_CURRENT_CONTROL);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "force_control"))
				{
					CyberDiverController_SetMode(controller, CONTROLLER_FORCE_CONTROL);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "position_control"))
				{
					CyberDiverController_SetMode(controller, CONTROLLER_POSITION_CONTROL);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "simulated_structure"))
				{
					CyberDiverController_SetMode(controller, CONTROLLER_SIMULATED_STRUCTURE);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "setpoint"))
		{
			float setpoint;
			if (CyberDiverShell_GetNextFloat(shell, &setpoint))
			{
				CyberDiverController_SetSetpoint(controller, setpoint);
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "led"))
		{
			int32_t led;
			if (CyberDiverShell_GetNextInt(shell, &led))
			{
				CyberDiverController_SetLED(controller, led);
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
	}


	return 0;
}


static uint32_t CyberDiverControllerCMD_Get(CyberDiverController* controller, CyberDiverShell* shell)
{

	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "state"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);

			switch (CyberDiverController_GetMode(controller))
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
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetSetpoint(controller));
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputUint(shell, CyberDiverController_GetLED(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "mode"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);

			switch (CyberDiverController_GetMode(controller))
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


			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "setpoint"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetSetpoint(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "led"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputUint(shell, CyberDiverController_GetLED(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "position"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetPosition(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "velocity"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetVelocity(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "current"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetCurrent(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "force"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetForce(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "duty"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetDutyCycle(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "accel"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetAcceleration(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "output"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetOutput(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "data"))
		{
			CyberDiverControllerDataPoint data = CyberDiverController_GetData(controller);

			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverControllerHandler_SendData(&data, shell);
			CyberDiverShell_PutOutputDelimiter(shell);

			return 1;
		}
	}

	return 0;
}


static uint32_t CyberDiverControllerCMD_Sequence(CyberDiverController* controller, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "start"))
		{
			CyberDiverController_StartSequence(controller);
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "stop"))
		{
			CyberDiverController_StopSequence(controller);
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "clear"))
		{
			if (CyberDiverController_ClearSequence(controller))
			{
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "set"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "length"))
				{
					int32_t length;
					if (CyberDiverShell_GetNextInt(shell, &length))
					{
						if (CyberDiverController_SetSequenceLength(controller, length))
						{
							CyberDiverShell_PutOutputString(shell, "ack");
							CyberDiverShell_PutOutputDelimiter(shell);
							return 1;
						}
					}
				}
				else if (!strcmp(str, "looping"))
				{
					int32_t looping;
					if (CyberDiverShell_GetNextInt(shell, &looping))
					{
						if (CyberDiverController_SetSequenceLooping(controller, looping))
						{
							CyberDiverShell_PutOutputString(shell, "ack");
							CyberDiverShell_PutOutputDelimiter(shell);
							return 1;
						}
					}
				}
				else if (!strcmp(str, "step"))
				{
					int32_t index;
					int32_t time;
					char mode_str[CD_SHELL_MAX_TOKEN_LEN];
					float setpoint;
					int32_t led;

					if (CyberDiverShell_GetNextInt(shell, &index) &&
							CyberDiverShell_GetNextInt(shell, &time) &&
							CyberDiverShell_GetNextString(shell, mode_str, CD_SHELL_MAX_TOKEN_LEN) &&
							CyberDiverShell_GetNextFloat(shell, &setpoint) &&
							CyberDiverShell_GetNextInt(shell, &led))
					{

						uint32_t mode_str_valid = 1;
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
							mode_str_valid = 0;


						if (mode_str_valid && CyberDiverController_SetSequenceStep(controller, index, time, mode, setpoint, led))
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
				if (!strcmp(str, "length"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, CyberDiverController_GetSequenceLength(controller));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "looping"))
				{
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputUint(shell, CyberDiverController_GetSequenceLooping(controller));
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
				else if (!strcmp(str, "step"))
				{
					int32_t index;
					if (CyberDiverShell_GetNextInt(shell, &index))
					{
						uint32_t time;
						CyberDiverControllerMode mode;
						float setpoint;
						uint32_t led;
						CyberDiverController_GetSequenceStep(controller, index, &time, &mode, &setpoint, &led);

						CyberDiverShell_PutOutputString(shell, "ack");
						CyberDiverShell_PutOutputSeparator(shell);
						CyberDiverShell_PutOutputUint(shell, time);
						CyberDiverShell_PutOutputSeparator(shell);

						switch (mode)
						{
						case CONTROLLER_IDLE:
							CyberDiverShell_PutOutputString(shell, "idle");
							break;
						case CONTROLLER_CURRENT_CONTROL:
							CyberDiverShell_PutOutputString(shell, "current control");
							break;
						case CONTROLLER_FORCE_CONTROL:
							CyberDiverShell_PutOutputString(shell, "force control");
							break;
						case CONTROLLER_POSITION_CONTROL:
							CyberDiverShell_PutOutputString(shell, "position control");
							break;
						case CONTROLLER_SIMULATED_STRUCTURE:
							CyberDiverShell_PutOutputString(shell, "simulated structure");
							break;
						}

						CyberDiverShell_PutOutputSeparator(shell);
						CyberDiverShell_PutOutputFloat(shell, setpoint);
						CyberDiverShell_PutOutputSeparator(shell);
						CyberDiverShell_PutOutputUint(shell, led);
						CyberDiverShell_PutOutputDelimiter(shell);
						return 1;
					}
				}
			}
		}
	}

	return 0;
}


static uint32_t CyberDiverControllerCMD_ConfigSet(CyberDiverController* controller, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "gains"))
		{
			float Kp, Ki, Kd, Kff;
			if (CyberDiverShell_GetNextFloat(shell, &Kp) &&
					CyberDiverShell_GetNextFloat(shell, &Ki) &&
					CyberDiverShell_GetNextFloat(shell, &Kd) &&
					CyberDiverShell_GetNextFloat(shell, &Kff))
			{
				CyberDiverController_SetGains(controller, Kp, Ki, Kd, Kff);
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "tau"))
		{
			float tau;
			if (CyberDiverShell_GetNextFloat(shell, &tau))
			{
				CyberDiverController_SetTau(controller, tau);
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "period"))
		{
			float T;
			if (CyberDiverShell_GetNextFloat(shell, &T))
			{
				CyberDiverController_SetPeriod(controller, T);
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "limits"))
		{
			float lim_min, lim_max;
			if (CyberDiverShell_GetNextFloat(shell, &lim_min) && CyberDiverShell_GetNextFloat(shell, &lim_max))
			{
				CyberDiverController_SetLimits(controller, lim_min, lim_max);
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "calibration"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{

				void (*setter)(CyberDiverController*, uint32_t, float*, float, float, uint32_t);

				if (!strcmp(str, "coil"))
				{
					setter = CyberDiverController_SetCoilCalibration;
				}
				else if (!strcmp(str, "passive"))
				{
					setter = CyberDiverController_SetPassiveCalibration;
				}
				else if (!strcmp(str, "structure"))
				{
					setter = CyberDiverController_SetStructureCalibration;
				}
				else
				{
					return 0;
				}

				/* by this point we know which calibration curve we are setting */
				int32_t order;
				float input_min, input_max;
				int32_t fast_compute;

				if (CyberDiverShell_GetNextInt(shell, &order) &&
						CyberDiverShell_GetNextFloat(shell, &input_min) &&
						CyberDiverShell_GetNextFloat(shell, &input_max) &&
						CyberDiverShell_GetNextInt(shell, &fast_compute))
				{
					/* now get the coefficients */
					if ((uint32_t)order > CALIB_POLY_MAX_ORDER) order = CALIB_POLY_MAX_ORDER;
					float coeffs[order + 1];
					for (uint32_t i = 0; i <= order; i++)
						if (!CyberDiverShell_GetNextFloat(shell, &coeffs[i]))
							return 0;

					/* by this point we have successfully parsed all the coefficients so call our setter function */
					setter(controller, order, coeffs, input_min, input_max, fast_compute);
					CyberDiverShell_PutOutputString(shell, "ack");
					CyberDiverShell_PutOutputDelimiter(shell);
					return 1;
				}
			}
		}
		else if (!strcmp(str, "damping"))
		{
			float damping;
			if (CyberDiverShell_GetNextFloat(shell, &damping))
			{
				CyberDiverController_SetDamping(controller, damping);
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
	}

	return 0;
}


static uint32_t CyberDiverControllerCMD_ConfigGet(CyberDiverController* controller, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "gains"))
		{
			float Kp, Ki, Kd, Kff;
			CyberDiverController_GetGains(controller, &Kp, &Ki, &Kd, &Kff);

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
		else if (!strcmp(str, "tau"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetTau(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "period"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetPeriod(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "limits"))
		{
			float lim_min, lim_max;
			CyberDiverController_GetLimits(controller, &lim_min, &lim_max);

			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, lim_min);
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, lim_max);
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "calibration"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{

				uint32_t order;
				float* coeffs;
				float input_min, input_max;
				uint32_t fast_compute;

				if (!strcmp(str, "coil"))
				{
					CyberDiverController_GetCoilCalibration(controller, &order, &coeffs, &input_min, &input_max, &fast_compute);
				}
				else if (!strcmp(str, "passive"))
				{
					CyberDiverController_GetPassiveCalibration(controller, &order, &coeffs, &input_min, &input_max, &fast_compute);
				}
				else if (!strcmp(str, "structure"))
				{
					CyberDiverController_GetStructureCalibration(controller, &order, &coeffs, &input_min, &input_max, &fast_compute);
				}
				else
				{
					return 0;
				}

				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputSeparator(shell);

				CyberDiverShell_PutOutputUint(shell, order);
				CyberDiverShell_PutOutputSeparator(shell);
				CyberDiverShell_PutOutputFloat(shell, input_min);
				CyberDiverShell_PutOutputSeparator(shell);
				CyberDiverShell_PutOutputFloat(shell, input_max);
				CyberDiverShell_PutOutputSeparator(shell);
				CyberDiverShell_PutOutputUint(shell, fast_compute);

				for (uint32_t i = 0; i <= order; i++)
				{
					CyberDiverShell_PutOutputSeparator(shell);
					CyberDiverShell_PutOutputFloat(shell, coeffs[i]);
				}


				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;

			}
		}
		else if (!strcmp(str, "damping"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverController_GetDamping(controller));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
	}

	return 0;
}


uint32_t CyberDiverControllerCMD_Execute(void* obj, CyberDiverShell* shell)
{
	CyberDiverController* controller = (CyberDiverController*)obj;

	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "resettime"))
		{
			CyberDiverController_ResetTime(controller);
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "set"))
		{
			if (CyberDiverControllerCMD_Set(controller, shell))
				return 1;
		}
		else if (!strcmp(str, "get"))
		{
			if (CyberDiverControllerCMD_Get(controller, shell))
				return 1;
		}
		else if (!strcmp(str, "sequence"))
		{
			if (CyberDiverControllerCMD_Sequence(controller, shell))
				return 1;
		}
		else if (!strcmp(str, "config"))
		{
			if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
			{
				if (!strcmp(str, "set"))
				{
					if (CyberDiverControllerCMD_ConfigSet(controller, shell))
						return 1;
				}
				else if (!strcmp(str, "get"))
				{
					if (CyberDiverControllerCMD_ConfigGet(controller, shell))
						return 1;
				}
			}
		}
	}

	return 0;
}



void CyberDiverControllerHandler_SendData(CyberDiverControllerDataPoint* data, CyberDiverShell* shell)
{

	char str[1024];

	sprintf(str, " %lu %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f ",
			data->time_micros,
			data->measured_current_amps,
			data->measured_force_n,
			data->measured_position_mm,
			data->measured_velocity_mmps,
			data->measured_duty_cycle,
			data->measured_accel_x_g,
			data->measured_accel_y_g,
			data->measured_accel_z_g,
			data->setpoint);

	CyberDiverShell_PutOutputString(shell, str);

	switch (CyberDiverController_GetModeFromStatus(data->status))
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
	CyberDiverShell_PutOutputUint(shell, CyberDiverController_GetLEDFromStatus(data->status));
}
