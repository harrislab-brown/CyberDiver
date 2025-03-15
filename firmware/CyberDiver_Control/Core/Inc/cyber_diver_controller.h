/*
 * cyber_diver_controller.h
 *
 *	This object runs the control loops for the active impactor
 *
 *  Created on: Jul 5, 2024
 *      Author: johnt
 */

#ifndef INC_CYBER_DIVER_CONTROLLER_H_
#define INC_CYBER_DIVER_CONTROLLER_H_

#define CD_CONTROLLER_SEQUENCE_MAX_LEN 1024
#define CD_CONTROLLER_MIN_UPDATE_PERIOD 0.00002f

#define CD_CONTROLLER_TIM_BASE_HZ 275000000
#define CD_CONTROLLER_TIM_PSC 275
#define CD_CONTROLLER_TIM_COUNTS_PER_SECOND (CD_CONTROLLER_TIM_BASE_HZ / CD_CONTROLLER_TIM_PSC)


#include "pid.h"
#include "calib_poly.h"
#include "sequencer.h"
#include "deriv.h"
#include "main.h"


/* operating modes of the diver */
typedef enum
{
	CONTROLLER_IDLE,  /* This is somewhat redundant since it's the same as current control mode with a set point of zero, but nice to have an explicit idle state. Also can switch to idle without removing the current set point */
	CONTROLLER_CURRENT_CONTROL,
	CONTROLLER_FORCE_CONTROL,
	CONTROLLER_POSITION_CONTROL,
	CONTROLLER_SIMULATED_STRUCTURE
} CyberDiverControllerMode;

/* state of the controller, including mode, controller set point, and whether the indicator LED is on (to verify timing with high-speed camera) */
typedef struct
{
	uint32_t status;  /* this variable combines the impactor mode and the led status - the bottom bits are the mode and the top bit is the led on/off flag */
	float setpoint;
} CyberDiverControllerState;

typedef struct
{
	/* during the experiment, we run the cyber diver through a prescribed sequence of states */
	CyberDiverControllerState sequence[CD_CONTROLLER_SEQUENCE_MAX_LEN];  /* array for the sequence of states */
	uint32_t len;
	uint32_t time_array[CD_CONTROLLER_SEQUENCE_MAX_LEN];  /* times to transition between states */
	uint32_t is_looping;  /* whether we want the experimental sequence to loop */

} CyberDiverControllerExperimentalSequence;


typedef struct
{
	CyberDiverControllerExperimentalSequence experimental_sequence;

	PIDController_Config pos_pid_config;

	CalibrationPolynomial_Config poly_coil_config;  /* coil force coeff (N/A) from displacement (mm)  */
	CalibrationPolynomial_Config poly_passive_config;  /* passive restoring force (N) from displacement (mm) */
	CalibrationPolynomial_Config poly_structure_config;  /* desired output force (N) from displacement (mm) */

	float damping_npmmps;  /* linear damping coefficient in N/(mm/s) */

} CyberDiverController_Config;

typedef struct
{
	/* configuration parameters */
	CyberDiverController_Config* config;

	/* operating mode of the diver controller */
	CyberDiverControllerState state;
	Sequencer sequencer;
	uint32_t start_time_micros;

	volatile uint32_t* time_micros_ptr;  /* pointer to a variable which holds the current time in microseconds */
	TIM_HandleTypeDef* htim_loop;  /* handle to the timer that runs the real time control loop */

	/* state variables updated each cycle */
	uint32_t time_micros;  /* stores the time in microseconds, captured at update from the microsecond pointer */
	float measured_current_amps;  /* sensor readings converted to physical units */
	float measured_force_n;
	float measured_position_mm;
	float measured_duty_cycle;  /* coil duty cycle */
	float measured_accel_x_g, measured_accel_y_g, measured_accel_z_g;  /* capture the acceleration from one of the on-board sensors */

	/* finite difference to estimate velocity */
	DerivativeEstimator deriv;
	float measured_velocity_mmps;

	/* output of the controller is the current that the coil should be set to */
	float output_current_amps;

	/* PID controller for position */
	PIDController pid_pos;

	/* calibration polynomials */
	CalibrationPolynomial poly_coil;  /* coil force coeff (N/A) from displacement (mm)  */
	CalibrationPolynomial poly_passive;  /* passive restoring force (N) from displacement (mm) */
	CalibrationPolynomial poly_structure;  /* desired output force (N) from displacement (mm) */

} CyberDiverController;


void CyberDiverController_Init(CyberDiverController* controller, CyberDiverController_Config* config, volatile uint32_t* time_micros_ptr, TIM_HandleTypeDef* htim_loop);

/* main update function, called in real time loop */
void CyberDiverController_Update(CyberDiverController* controller, float position, float current, float duty_cycle, float accel_x, float accel_y, float accel_z);

/* call this whenever we load a new configuration file */
void CyberDiverController_Reset(CyberDiverController* controller);

/* starts the timer that runs the controller update */
void CyberDiverController_StartTimer(CyberDiverController* controller);


void CyberDiverController_SetState(CyberDiverController* controller, CyberDiverControllerState state);  /* general interface functions */
CyberDiverControllerState CyberDiverController_GetState(CyberDiverController* controller);

void CyberDiverController_SetMode(CyberDiverController* controller, CyberDiverControllerMode mode);
CyberDiverControllerMode CyberDiverController_GetMode(CyberDiverController* controller);

void CyberDiverController_SetSetpoint(CyberDiverController* controller, float setpoint);
float CyberDiverController_GetSetpoint(CyberDiverController* controller);

void CyberDiverController_SetLED(CyberDiverController* controller, uint32_t on);
uint32_t CyberDiverController_GetLED(CyberDiverController* controller);


float CyberDiverController_GetPosition(CyberDiverController* controller);  /* get any of the measured variables */
float CyberDiverController_GetVelocity(CyberDiverController* controller);
float CyberDiverController_GetCurrent(CyberDiverController* controller);
float CyberDiverController_GetForce(CyberDiverController* controller);
float CyberDiverController_GetDutyCycle(CyberDiverController* controller);
float CyberDiverController_GetAcceleration(CyberDiverController* controller);
float CyberDiverController_GetOutput(CyberDiverController* controller);


void CyberDiverController_SetGains(CyberDiverController* controller, float Kp, float Ki, float Kd, float Kff);  /* interface functions relating to the controller configuration parameters */
void CyberDiverController_GetGains(CyberDiverController* controller, float* Kp, float* Ki, float* Kd, float* Kff);

void CyberDiverController_SetTau(CyberDiverController* controller, float tau);
float CyberDiverController_GetTau(CyberDiverController* controller);

void CyberDiverController_SetPeriod(CyberDiverController* controller, float T);
float CyberDiverController_GetPeriod(CyberDiverController* controller);

void CyberDiverController_SetLimits(CyberDiverController* controller, float lim_min, float lim_max);
void CyberDiverController_GetLimits(CyberDiverController* controller, float* lim_min, float* lim_max);



/* provide a convenient method for returning all of the controller data at a given time */

/*
 * The idea is to decouple the data output from the internals of the controller class
 * However, seems like some duplicated code so there might be a cleaner way to do this
 */

typedef struct
{
	uint32_t time_micros;
	float measured_current_amps;
	float measured_force_n;
	float measured_position_mm;
	float measured_velocity_mmps;
	float measured_duty_cycle;
	float measured_accel_x_g;
	float measured_accel_y_g;
	float measured_accel_z_g;

	float setpoint;
	uint32_t status;  /* operating mode of the controller and the LED status bit */
} CyberDiverControllerDataPoint;

CyberDiverControllerDataPoint CyberDiverController_GetData(CyberDiverController* controller);
void CyberDiverController_ResetTime(CyberDiverController* controller);  /* sets t = 0 */



/* sequence interface functions to start or stop the operation of the cyber diver.
 * starting will start the experimental sequence contained in the config object
 * For testing with a constant set point, can make the sequence a single step
 */
void CyberDiverController_StartSequence(CyberDiverController* controller);
void CyberDiverController_StopSequence(CyberDiverController* controller);
uint32_t CyberDiverController_ClearSequence(CyberDiverController* controller);  /* only works if the sequence is not currently running */

uint32_t CyberDiverController_SetSequenceLength(CyberDiverController* controller, uint32_t length);
uint32_t CyberDiverController_GetSequenceLength(CyberDiverController* controller);
uint32_t CyberDiverController_SetSequenceLooping(CyberDiverController* controller, uint32_t looping);
uint32_t CyberDiverController_GetSequenceLooping(CyberDiverController* controller);
uint32_t CyberDiverController_SetSequenceStep(CyberDiverController* controller, uint32_t index, uint32_t time, CyberDiverControllerMode mode, float setpoint, uint32_t led);
void CyberDiverController_GetSequenceStep(CyberDiverController* controller, uint32_t index, uint32_t* time, CyberDiverControllerMode* mode, float* setpoint, uint32_t* led);


/*
 * Functions for interfacing with the calibration curves
 */

void CyberDiverController_SetCoilCalibration(CyberDiverController* controller, uint32_t order, float* coeffs, float input_min, float input_max, uint32_t fast_compute);
void CyberDiverController_GetCoilCalibration(CyberDiverController* controller, uint32_t* order, float** coeffs, float* input_min, float* input_max, uint32_t* fast_compute);
void CyberDiverController_SetPassiveCalibration(CyberDiverController* controller, uint32_t order, float* coeffs, float input_min, float input_max, uint32_t fast_compute);
void CyberDiverController_GetPassiveCalibration(CyberDiverController* controller, uint32_t* order, float** coeffs, float* input_min, float* input_max, uint32_t* fast_compute);
void CyberDiverController_SetStructureCalibration(CyberDiverController* controller, uint32_t order, float* coeffs, float input_min, float input_max, uint32_t fast_compute);
void CyberDiverController_GetStructureCalibration(CyberDiverController* controller, uint32_t* order, float** coeffs, float* input_min, float* input_max, uint32_t* fast_compute);

void CyberDiverController_SetDamping(CyberDiverController* controller, float damping);
float CyberDiverController_GetDamping(CyberDiverController* controller);


/*
 * static methods
 */

CyberDiverControllerMode CyberDiverController_GetModeFromStatus(uint32_t status);
uint32_t CyberDiverController_GetLEDFromStatus(uint32_t status);
uint32_t CyberDiverController_GetStatusFromModeAndLED(CyberDiverControllerMode mode, uint32_t led);

#endif /* INC_CYBER_DIVER_CONTROLLER_H_ */
