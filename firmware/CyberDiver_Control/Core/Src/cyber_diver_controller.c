/*
 * cyber_diver_controller.c
 *
 *	This object runs the control loops for the active impactor
 *
 *  Created on: Jul 5, 2024
 *      Author: johnt
 */

#include "cyber_diver_controller.h"



static void CyberDiverController_ConfigureTimer(CyberDiverController* controller)
{
	controller->htim_loop->Instance->PSC = CD_CONTROLLER_TIM_PSC - 1;
	uint32_t arr = CD_CONTROLLER_TIM_COUNTS_PER_SECOND * controller->config->pos_pid_config.T - 1;
	controller->config->pos_pid_config.T = ((float)arr + 1.0f) / CD_CONTROLLER_TIM_COUNTS_PER_SECOND;  /* makes sure the period is accurate based on discrete timer count */
	controller->htim_loop->Instance->ARR = arr;
}



void CyberDiverController_Init(CyberDiverController* controller, CyberDiverController_Config* config, volatile uint32_t* time_micros_ptr, TIM_HandleTypeDef* htim_loop)
{
	/* Initialize the impactor controller object */

	/* assign the configuration */
	controller->config = config;

	/* set the pointers to sensor data, power board data and time-keeping variable */
	controller->time_micros_ptr = time_micros_ptr;
	controller->htim_loop = htim_loop;
	CyberDiverController_ConfigureTimer(controller);

	/* set to idle mode */
	controller->state.status = CONTROLLER_IDLE;
	controller->state.setpoint = 0.0f;

	/* initialize the calibration polynomials */
	CalibrationPolynomial_Init(&controller->poly_coil, &config->poly_coil_config);
	CalibrationPolynomial_Init(&controller->poly_passive, &config->poly_passive_config);
	CalibrationPolynomial_Init(&controller->poly_structure, &config->poly_structure_config);

	/* initialize the derivative estimator */
	DerivativeEstimator_Init(&controller->deriv);

	/* initialize the PID controller */
	PIDController_Init(&controller->pid_pos, &config->pos_pid_config);

	/* initialize the sequencer */
	Sequencer_Init(&controller->sequencer, time_micros_ptr);
}

void CyberDiverController_Update(CyberDiverController* controller, float position, float current, float duty_cycle, float accel_x, float accel_y, float accel_z)
{
	/* store the experimental data */

	/* update the time-keeping variable */
	controller->time_micros = *controller->time_micros_ptr - controller->start_time_micros;

	/* update the position */
	controller->measured_position_mm = position;

	/* Get the measured current and duty cycle from the power board interface */
	controller->measured_current_amps = current;
	controller->measured_duty_cycle = duty_cycle;

	/* store the measured acceleration from the accelerometer chip */
	controller->measured_accel_x_g = -accel_x;  /* invert sign to match our convention */
	controller->measured_accel_y_g = -accel_y;
	controller->measured_accel_z_g = -accel_z;

	/* estimate the relative velocity between nose and body */
	controller->measured_velocity_mmps = DerivativeEstimator_Update(&controller->deriv, controller->measured_position_mm, controller->config->pos_pid_config.T);

	/* calculate the force (TOTAL coupling force on the BODY) based on the measured current and the displacement */
	controller->measured_force_n = -CalibrationPolynomial_Eval(&controller->poly_passive, controller->measured_position_mm) + controller->measured_current_amps * CalibrationPolynomial_Eval(&controller->poly_coil, controller->measured_position_mm);

	/* update the sequencer */
	if (Sequencer_IsRunning(&controller->sequencer))
	{
		uint32_t index;
		if (Sequencer_Update(&controller->sequencer, &index))
			controller->state = controller->config->experimental_sequence.sequence[index];
	}


	/* update the control loops based on the current controller state */
	if (CyberDiverController_GetMode(controller) == CONTROLLER_IDLE)
	{
		controller->output_current_amps = 0.0f;
	}
	else if (CyberDiverController_GetMode(controller) == CONTROLLER_CURRENT_CONTROL)
	{
		controller->output_current_amps = controller->state.setpoint;
	}
	else if (CyberDiverController_GetMode(controller) == CONTROLLER_FORCE_CONTROL)  /* TOTAL coupling force on the BODY */
	{
		controller->output_current_amps = (controller->state.setpoint + CalibrationPolynomial_Eval(&controller->poly_passive, controller->measured_position_mm)) / CalibrationPolynomial_Eval(&controller->poly_coil, controller->measured_position_mm);
	}
	else if (CyberDiverController_GetMode(controller) == CONTROLLER_POSITION_CONTROL)  /* treat the PID controller output as a force for equal response across the travel range */
	{
		PIDController_Update(&controller->pid_pos, controller->state.setpoint, controller->measured_position_mm);
		controller->output_current_amps = (controller->pid_pos.out + CalibrationPolynomial_Eval(&controller->poly_passive, controller->measured_position_mm)) / CalibrationPolynomial_Eval(&controller->poly_coil, controller->measured_position_mm);
	}
	else if (CyberDiverController_GetMode(controller) == CONTROLLER_SIMULATED_STRUCTURE)
	{
		/* apply the structural model to determine the desired output force */
		/* then convert desired force to set-point current using the passive coupling force and the coil calibration curve */

		/*
		 * We could combine all these polynomials beforehand and only have to do a single evaluation here for more speed.
		 * However I tested this briefly and it doesn't seem to make much different for the overall performance
		 */

		float force_n = -CalibrationPolynomial_Eval(&controller->poly_structure, controller->measured_position_mm) - controller->config->damping_npmmps * controller->measured_velocity_mmps;
		controller->output_current_amps = (force_n + CalibrationPolynomial_Eval(&controller->poly_passive, controller->measured_position_mm)) / CalibrationPolynomial_Eval(&controller->poly_coil, controller->measured_position_mm);
	}
}


void CyberDiverController_Reset(CyberDiverController* controller)
{
	/* stop the controller */
	CyberDiverController_StopSequence(controller);
	CyberDiverController_SetMode(controller, CONTROLLER_IDLE);
	CyberDiverController_SetSetpoint(controller, 0);
	CyberDiverController_SetLED(controller, 0);

	/* re-compute the lookup tables */
	CalibrationPolynomial_ComputeLUT(&controller->poly_coil);
	CalibrationPolynomial_ComputeLUT(&controller->poly_passive);
	CalibrationPolynomial_ComputeLUT(&controller->poly_structure);

	/* update the control loop timer register */
	CyberDiverController_ConfigureTimer(controller);

	/* reset the PID controllers */
	PIDController_Reset(&controller->pid_pos);
}


void CyberDiverController_StartTimer(CyberDiverController* controller)
{
	/* set t = 0 */
	CyberDiverController_ResetTime(controller);

	/* start the timer that triggers the control loop update */
	HAL_TIM_Base_Start_IT(controller->htim_loop);
}



void CyberDiverController_SetState(CyberDiverController* controller, CyberDiverControllerState state)
{
	controller->state = state;
}

CyberDiverControllerState CyberDiverController_GetState(CyberDiverController* controller)
{
	return controller->state;
}


void CyberDiverController_SetMode(CyberDiverController* controller, CyberDiverControllerMode mode)
{
	/* don't change the top bit of the mode variable since this holds the led status */
	controller->state.status = (controller->state.status & (1 << 31)) | (mode & ~(1 << 31));
}


CyberDiverControllerMode CyberDiverController_GetMode(CyberDiverController* controller)
{
	/* get rid of the top bit which represents the led status */
	return CyberDiverController_GetModeFromStatus(controller->state.status);
}


void CyberDiverController_SetSetpoint(CyberDiverController* controller, float setpoint)
{
	controller->state.setpoint = setpoint;
}


float CyberDiverController_GetSetpoint(CyberDiverController* controller)
{
	return controller->state.setpoint;
}


void CyberDiverController_SetLED(CyberDiverController* controller, uint32_t on)
{
	/* clear or set the top bit of the mode variable */
	if (on) on = 1;
	controller->state.status &= ~(1 << 31);
	controller->state.status |= (on << 31);
}


uint32_t CyberDiverController_GetLED(CyberDiverController* controller)
{
	return CyberDiverController_GetLEDFromStatus(controller->state.status);
}


float CyberDiverController_GetPosition(CyberDiverController* controller)
{
	return controller->measured_position_mm;
}
float CyberDiverController_GetVelocity(CyberDiverController* controller)
{
	return controller->measured_velocity_mmps;
}
float CyberDiverController_GetCurrent(CyberDiverController* controller)
{
	return controller->measured_current_amps;
}
float CyberDiverController_GetForce(CyberDiverController* controller)
{
	return controller->measured_force_n;
}
float CyberDiverController_GetDutyCycle(CyberDiverController* controller)
{
	return controller->measured_duty_cycle;
}
float CyberDiverController_GetAcceleration(CyberDiverController* controller)
{
	return controller->measured_accel_y_g;  /* this is the axial acceleration */
}
float CyberDiverController_GetOutput(CyberDiverController* controller)
{
	/* this is the commanded current that is sent to the power board */
	return controller->output_current_amps;
}





void CyberDiverController_SetGains(CyberDiverController* controller, float Kp, float Ki, float Kd, float Kff)
{
	controller->config->pos_pid_config.Kp = Kp;
	controller->config->pos_pid_config.Ki = Ki;
	controller->config->pos_pid_config.Kd = Kd;
	controller->config->pos_pid_config.Kff = Kff;

	PIDController_Reset(&controller->pid_pos);  /* be sure to reset the controller after updating the gains */
}

void CyberDiverController_GetGains(CyberDiverController* controller, float* Kp, float* Ki, float* Kd, float* Kff)
{
	*Kp = controller->config->pos_pid_config.Kp;
	*Ki = controller->config->pos_pid_config.Ki;
	*Kd = controller->config->pos_pid_config.Kd;
	*Kff = controller->config->pos_pid_config.Kff;
}

void CyberDiverController_SetTau(CyberDiverController* controller, float tau)
{
	if (tau < 0) tau = 0;
	controller->config->pos_pid_config.tau = tau;
	PIDController_Reset(&controller->pid_pos);  /* be sure to reset the controller after updating the gains */
}

float CyberDiverController_GetTau(CyberDiverController* controller)
{
	return controller->config->pos_pid_config.tau;
}

void CyberDiverController_SetPeriod(CyberDiverController* controller, float T)
{
	/* inner control loop update period */
	/* WARNING: this should only be updated with scope connected to verify that the CPU can keep up */

	if (T < CD_CONTROLLER_MIN_UPDATE_PERIOD) T = CD_CONTROLLER_MIN_UPDATE_PERIOD;
	controller->config->pos_pid_config.T = T;

	CyberDiverController_ConfigureTimer(controller);  /* update the timer register to change the loop rate */
	PIDController_Reset(&controller->pid_pos);
}

float CyberDiverController_GetPeriod(CyberDiverController* controller)
{
	return controller->config->pos_pid_config.T;
}

void CyberDiverController_SetLimits(CyberDiverController* controller, float lim_min, float lim_max)
{
	if (lim_min < lim_max)
	{
		controller->config->pos_pid_config.lim_min = lim_min;
		controller->config->pos_pid_config.lim_max = lim_max;
		PIDController_Reset(&controller->pid_pos);  /* be sure to reset the controller after updating the gains */
	}
}

void CyberDiverController_GetLimits(CyberDiverController* controller, float* lim_min, float* lim_max)
{
	*lim_min = controller->config->pos_pid_config.lim_min;
	*lim_max = controller->config->pos_pid_config.lim_max;
}




CyberDiverControllerDataPoint CyberDiverController_GetData(CyberDiverController* controller)
{
	CyberDiverControllerDataPoint data;

	data.time_micros = controller->time_micros;
	data.measured_current_amps = controller->measured_current_amps;
	data.measured_force_n = controller->measured_force_n;
	data.measured_position_mm = controller->measured_position_mm;
	data.measured_velocity_mmps = controller->measured_velocity_mmps;
	data.measured_duty_cycle = controller->measured_duty_cycle;
	data.measured_accel_x_g = controller->measured_accel_x_g;
	data.measured_accel_y_g = controller->measured_accel_y_g;
	data.measured_accel_z_g = controller->measured_accel_z_g;
	data.status = controller->state.status;
	data.setpoint = controller->state.setpoint;

	return data;
}


void CyberDiverController_ResetTime(CyberDiverController* controller)
{
	controller->start_time_micros = *controller->time_micros_ptr;
}




void CyberDiverController_StartSequence(CyberDiverController* controller)
{
	CyberDiverController_ResetTime(controller);
	Sequencer_SetSequence(&controller->sequencer, controller->config->experimental_sequence.time_array, controller->config->experimental_sequence.len, controller->config->experimental_sequence.is_looping);
	Sequencer_Start(&controller->sequencer);
}


void CyberDiverController_StopSequence(CyberDiverController* controller)
{
	/* stop the sequence in progress and set the diver to idle mode */
	Sequencer_Stop(&controller->sequencer);
	controller->state.status = CONTROLLER_IDLE;
	controller->state.setpoint = 0.0f;
}

uint32_t CyberDiverController_ClearSequence(CyberDiverController* controller)
{
	/* make sure not to clear a sequence that is in progress */
	if (!Sequencer_IsRunning(&controller->sequencer))
	{
		for (uint32_t i = 0; i < controller->config->experimental_sequence.len; i++)
		{
			controller->config->experimental_sequence.time_array[i] = 0;
			controller->config->experimental_sequence.sequence[i].status = CONTROLLER_IDLE;
			controller->config->experimental_sequence.sequence[i].setpoint = 0.0f;
		}

		controller->config->experimental_sequence.len = 0;
		controller->config->experimental_sequence.is_looping = 0;
		return 1;
	}

	return 0;
}

uint32_t CyberDiverController_SetSequenceLength(CyberDiverController* controller, uint32_t length)
{
	/* only do this when the sequence is stopped */
	if (!Sequencer_IsRunning(&controller->sequencer))
	{
		if (length < 1) length = 1;
		if (length > CD_CONTROLLER_SEQUENCE_MAX_LEN) length = CD_CONTROLLER_SEQUENCE_MAX_LEN;

		controller->config->experimental_sequence.len = length;

		return 1;
	}

	return 0;
}

uint32_t CyberDiverController_GetSequenceLength(CyberDiverController* controller)
{
	return controller->config->experimental_sequence.len;
}

uint32_t CyberDiverController_SetSequenceLooping(CyberDiverController* controller, uint32_t looping)
{
	/* only do this when the sequence is stopped */

	if (!Sequencer_IsRunning(&controller->sequencer))
	{
		if (looping)
			controller->config->experimental_sequence.is_looping = 1;
		else
			controller->config->experimental_sequence.is_looping = 0;

		return 1;
	}

	return 0;
}

uint32_t CyberDiverController_GetSequenceLooping(CyberDiverController* controller)
{
	return controller->config->experimental_sequence.is_looping;
}

uint32_t CyberDiverController_SetSequenceStep(CyberDiverController* controller, uint32_t index, uint32_t time, CyberDiverControllerMode mode, float setpoint, uint32_t led)
{
	/* only do this when the sequence is stopped */
	if (!Sequencer_IsRunning(&controller->sequencer))
	{
		if (index < 0) index = 0;
		if (index >= controller->config->experimental_sequence.len) index = controller->config->experimental_sequence.len - 1;

		controller->config->experimental_sequence.time_array[index] = time;
		controller->config->experimental_sequence.sequence[index].setpoint = setpoint;
		if (led) led = 1;
		controller->config->experimental_sequence.sequence[index].status = CyberDiverController_GetStatusFromModeAndLED(mode, led);

		return 1;
	}


	return 0;
}

void CyberDiverController_GetSequenceStep(CyberDiverController* controller, uint32_t index, uint32_t* time, CyberDiverControllerMode* mode, float* setpoint, uint32_t* led)
{
	if (index < 0) index = 0;
	if (index >= controller->config->experimental_sequence.len) index = controller->config->experimental_sequence.len - 1;


	*time = controller->config->experimental_sequence.time_array[index];
	*mode = CyberDiverController_GetModeFromStatus(controller->config->experimental_sequence.sequence[index].status);
	*setpoint = controller->config->experimental_sequence.sequence[index].setpoint;
	*led = CyberDiverController_GetLEDFromStatus(controller->config->experimental_sequence.sequence[index].status);
}





/*
 * Functions for interfacing with the calibration curves
 */

void CyberDiverController_SetCoilCalibration(CyberDiverController* controller, uint32_t order, float* coeffs, float input_min, float input_max, uint32_t fast_compute)
{
	CalibrationPolynomial_Set(&controller->poly_coil, order, coeffs, input_min, input_max, fast_compute);
}

void CyberDiverController_GetCoilCalibration(CyberDiverController* controller, uint32_t* order, float** coeffs, float* input_min, float* input_max, uint32_t* fast_compute)
{
	CalibrationPolynomial_Get(&controller->poly_coil, order, coeffs, input_min, input_max, fast_compute);
}

void CyberDiverController_SetPassiveCalibration(CyberDiverController* controller, uint32_t order, float* coeffs, float input_min, float input_max, uint32_t fast_compute)
{
	CalibrationPolynomial_Set(&controller->poly_passive, order, coeffs, input_min, input_max, fast_compute);
}

void CyberDiverController_GetPassiveCalibration(CyberDiverController* controller, uint32_t* order, float** coeffs, float* input_min, float* input_max, uint32_t* fast_compute)
{
	CalibrationPolynomial_Get(&controller->poly_passive, order, coeffs, input_min, input_max, fast_compute);
}

void CyberDiverController_SetStructureCalibration(CyberDiverController* controller, uint32_t order, float* coeffs, float input_min, float input_max, uint32_t fast_compute)
{
	CalibrationPolynomial_Set(&controller->poly_structure, order, coeffs, input_min, input_max, fast_compute);
}

void CyberDiverController_GetStructureCalibration(CyberDiverController* controller, uint32_t* order, float** coeffs, float* input_min, float* input_max, uint32_t* fast_compute)
{
	CalibrationPolynomial_Get(&controller->poly_structure, order, coeffs, input_min, input_max, fast_compute);
}

void CyberDiverController_SetDamping(CyberDiverController* controller, float damping)
{
	controller->config->damping_npmmps = damping;
}

float CyberDiverController_GetDamping(CyberDiverController* controller)
{
	return controller->config->damping_npmmps;
}







CyberDiverControllerMode CyberDiverController_GetModeFromStatus(uint32_t status)
{
	/* get rid of the top bit which represents the led status */
	return status & (~(1 << 31));
}

uint32_t CyberDiverController_GetLEDFromStatus(uint32_t status)
{
	return status >> 31;
}

uint32_t CyberDiverController_GetStatusFromModeAndLED(CyberDiverControllerMode mode, uint32_t led)
{
	return (led << 31) | mode;
}
