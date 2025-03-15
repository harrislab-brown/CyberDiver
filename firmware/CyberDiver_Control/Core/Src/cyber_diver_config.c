/*
 * cyber_diver_config.c
 *
 *  Created on: Jul 10, 2024
 *      Author: johnt
 */

#include "cyber_diver_config.h"


uint8_t CyberDiverConfig_Init(CyberDiverConfig* global_config)
{

	/* returns true if we are able to successfully parse the settings file */
	uint8_t ret = 0;
	if (CyberDiverConfig_GetConfigFromSD(global_config))
	{
		ret = 1;
	}
	else
	{
		/* assign default values */

		global_config->firmware_version = CD_CONFIG_FIRMWARE_VERSION;
		strcpy(global_config->experiment_description, "default parameters");

		/* controller state at startup) */
		global_config->idle_controller_state.status = CONTROLLER_IDLE;
		global_config->idle_controller_state.setpoint = 0.0f;

		/* controller state while armed */
		global_config->armed_controller_state.status = CONTROLLER_IDLE;
		global_config->armed_controller_state.setpoint = 0.0f;

		/* how to handle transitions between states */
		global_config->staging_time_us = 5000000;
		global_config->running_time_us = 5000000;
		global_config->trigger_accel_threshold = 0.5f;

		/* controller neutral position offset */
		global_config->encoder_config.neutral_position_offset_mm = -0.203f;

		/* controller sequence configuration */
		global_config->controller_config.experimental_sequence.sequence[0].status = CONTROLLER_IDLE;
		global_config->controller_config.experimental_sequence.sequence[0].setpoint = 0.0f;
		global_config->controller_config.experimental_sequence.len = 1;
		global_config->controller_config.experimental_sequence.time_array[0] = 0;
		global_config->controller_config.experimental_sequence.is_looping = 0;

		/* position PID controller configuration */
		global_config->controller_config.pos_pid_config.Kff = 0.0f;
		global_config->controller_config.pos_pid_config.Kp = 50.0f;
		global_config->controller_config.pos_pid_config.Ki = 100.0f;
		global_config->controller_config.pos_pid_config.Kd = 0.025f;
		global_config->controller_config.pos_pid_config.lim_min = -30.0f;
		global_config->controller_config.pos_pid_config.lim_max = 30.0f;
		global_config->controller_config.pos_pid_config.T = 0.00002f;  /* outer control loop runs at 50 kHz */
		global_config->controller_config.pos_pid_config.tau = 0.0001f;

		/* SD card logger configuration */
		global_config->logger_config.decimation = 1;  /* only log every other sample */
		global_config->logger_config.location = LOGGER_LOC_MCU;
		global_config->logger_config.packet_size = 100;

		/* power board current PID controller configuration */
		global_config->power_board_config.cur_pid_config.Kff = 0.2f;
		global_config->power_board_config.cur_pid_config.Kp = 0.75f;
		global_config->power_board_config.cur_pid_config.Ki = 7500.0f;
		global_config->power_board_config.cur_pid_config.Kd = 0.0f;
		global_config->power_board_config.cur_pid_config.lim_min = -1.0f;
		global_config->power_board_config.cur_pid_config.lim_max = 1.0f;
		global_config->power_board_config.cur_pid_config.T = 0.00002f;  /* current control loop runs at 50 kHz */
		global_config->power_board_config.cur_pid_config.tau = 0.0f;

		global_config->power_board_config.Kff_vel = 0.0f;  /* feed-forward gain for velocity disturbance rejection */

		/* polynomials for diver calibration */
		float poly_coil_coeffs[] = {5.617, 0.1784, -0.04036, -0.001735, 0.0001108};
		global_config->controller_config.poly_coil_config.order = 4;
		global_config->controller_config.poly_coil_config.input_min = -6.5;
		global_config->controller_config.poly_coil_config.input_max = 6.5;
		global_config->controller_config.poly_coil_config.fast_compute = 1;
		for (uint32_t i = 0; i <= global_config->controller_config.poly_coil_config.order; i++)
			global_config->controller_config.poly_coil_config.coeffs[i] = poly_coil_coeffs[i];


		float poly_passive_coeffs[] = {0, 4.009, 0, 0.009074};
		global_config->controller_config.poly_passive_config.order = 3;
		global_config->controller_config.poly_passive_config.input_min = -6.5;
		global_config->controller_config.poly_passive_config.input_max = 6.5;
		global_config->controller_config.poly_passive_config.fast_compute = 1;
		for (uint32_t i = 0; i <= global_config->controller_config.poly_passive_config.order; i++)
			global_config->controller_config.poly_passive_config.coeffs[i] = poly_passive_coeffs[i];


		float poly_structure_coeffs[] = {0, 2.0, 0, 0};
		global_config->controller_config.poly_structure_config.order = 3;
		global_config->controller_config.poly_structure_config.input_min = -6.5;
		global_config->controller_config.poly_structure_config.input_max = 6.5;
		global_config->controller_config.poly_structure_config.fast_compute = 1;
		for (uint32_t i = 0; i <= global_config->controller_config.poly_structure_config.order; i++)
			global_config->controller_config.poly_structure_config.coeffs[i] = poly_structure_coeffs[i];

		global_config->controller_config.damping_npmmps = 0.0f;


		/* accelerometer */
		global_config->accelerometer_config.g_range = 16;
		global_config->accelerometer_config.lpf = 20;  /* sets bandwidth to ODR/20 = 1.333 kHz */
		global_config->accelerometer_config.usr_offset_x = 0.0f;
		global_config->accelerometer_config.usr_offset_y = 0.0f;
		global_config->accelerometer_config.usr_offset_z = 0.0f;
	}

	return ret;
}

uint8_t CyberDiverConfig_GetConfigFromSD(CyberDiverConfig* global_config)
{
	FATFS fs;
	FIL fil;
	UINT read_count;

	if (f_mount(&fs, "/", 1) != FR_OK)  /* mount the file system */
	{
		return 0;
	}

	if (f_open(&fil, CD_CONFIG_FILE_NAME, FA_READ) != FR_OK)  /* open the config file to read */
	{
		f_mount(NULL, "/", 1);  /* dismount the file system */
		return 0;
	}

	if (f_read(&fil, global_config, sizeof(CyberDiverConfig), &read_count) == FR_OK)  /* read the file right into the config struct */
	{
		if (read_count < sizeof(CyberDiverConfig) || global_config->firmware_version != CD_CONFIG_FIRMWARE_VERSION)  /* the barest minimum of input validation: make sure we read enough bytes and that the firmware versions match */
		{
			f_close(&fil);
			f_mount(NULL, "/", 1);  /* dismount the file system */
			return 0;
		}
	}
	else
	{
		f_close(&fil);
		f_mount(NULL, "/", 1);  /* dismount the file system */
		return 0;
	}

	f_close(&fil);
	f_mount(NULL, "/", 1);  /* dismount the file system */

	return 1;
}

uint8_t CyberDiverConfig_WriteConfigToSD(CyberDiverConfig* global_config)
{
	FATFS fs;
	FIL fil;
	UINT write_count;

	if (f_mount(&fs, "/", 1) != FR_OK)  /* mount the file system */
	{
		return 0;
	}

	if (f_open(&fil, CD_CONFIG_FILE_NAME, FA_CREATE_ALWAYS|FA_WRITE) != FR_OK)  /* open the config file to write */
	{
		f_mount(NULL, "/", 1);  /* dismount the file system */
		return 0;
	}

	f_write(&fil, global_config, sizeof(CyberDiverConfig), &write_count);
	f_close(&fil);
	f_mount(NULL, "/", 1);  /* dismount the file system */

	return 1;
}

uint8_t CyberDiverConfig_WriteConfigToFile(CyberDiverConfig* global_config, FIL* file)
{
	/* write configuration to a file that is already open (for adding the configuration parameters to a data file for an experiment) */
	UINT write_count;
	return (f_write(file, global_config, sizeof(CyberDiverConfig), &write_count) == FR_OK);
}



