/*
 * pid.h
 *
 *  Created on: Jan 9, 2024
 *      Author: johnt
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;
	float Kff;  /* feed forward gain */

	/* Derivative low pass time constant */
	float tau;

	/* Output limits */
	float lim_min;
	float lim_max;

	/* Controller sample time (in seconds) */
	float T;

} PIDController_Config;

typedef struct
{
	PIDController_Config* config;

	/* Controller state */
	float integrator;
	float differentiator;
	float prev_measurement;
	float prev_error;

	/* Controller output */
	float out;

} PIDController;


void PIDController_Init(PIDController* pid, PIDController_Config* config);
void PIDController_Reset(PIDController* pid);
float PIDController_Update(PIDController* pid, float setpoint, float measurement);

#endif /* INC_PID_H_ */
