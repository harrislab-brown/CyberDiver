/*
 * cyber_diver_settings.h
 *
 *  Created on: Jul 6, 2024
 *      Author: johnt
 */

#ifndef INC_CYBER_DIVER_CONFIG_H_
#define INC_CYBER_DIVER_CONFIG_H_

#include "cyber_diver_encoder.h"
#include "cyber_diver_controller.h"
#include "cyber_diver_power_board.h"
#include "cyber_diver_logger.h"
#include "IIS3DWB.h"
#include "fatfs.h"

/* struct to store all of the user settings for the cyber diver */

#define CD_CONFIG_FIRMWARE_VERSION 2
#define CD_CONFIG_FILE_NAME "CONFIG.DAT"

typedef struct
{
	uint32_t firmware_version;  /* keep track of firmware version for parsing data */
	char experiment_description[1024];  /* string that describes the experiment that we are running */

	CyberDiverControllerState idle_controller_state;
	CyberDiverControllerState armed_controller_state;

	uint32_t staging_time_us;  /* how long to allow for mounting the diver to the dropper before triggering the experiment based on acceleration */
	uint32_t running_time_us;  /* how long to run an experiment before returning to the idle state */
	float trigger_accel_threshold;  /* go from armed to running if the absolute value of the axial acceleration is below this value */

	CyberDiverEncoder_Config encoder_config;

	CyberDiverController_Config controller_config;

	CyberDiverLogger_Config logger_config;

	CyberDiverPowerBoard_Config power_board_config;

	IIS3DWB_Config accelerometer_config;

} CyberDiverConfig;

uint8_t CyberDiverConfig_Init(CyberDiverConfig* global_config);

/* get the configuration from the SD card or write it (returns true if successful) */
uint8_t CyberDiverConfig_GetConfigFromSD(CyberDiverConfig* global_config);
uint8_t CyberDiverConfig_WriteConfigToSD(CyberDiverConfig* global_config);
uint8_t CyberDiverConfig_WriteConfigToFile(CyberDiverConfig* global_config, FIL* file);  /* write configuration to a file that is already open */


#endif /* INC_CYBER_DIVER_CONFIG_H_ */
