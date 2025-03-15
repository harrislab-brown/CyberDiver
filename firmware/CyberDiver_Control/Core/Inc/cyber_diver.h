/*
 * cyber_diver.h
 *
 *  Created on: Jul 6, 2024
 *      Author: johnt
 */

#ifndef INC_CYBER_DIVER_H_
#define INC_CYBER_DIVER_H_

#include "cyber_diver_config.h"
#include "cyber_diver_encoder.h"
#include "cyber_diver_controller.h"
#include "cyber_diver_power_board.h"
#include "cyber_diver_logger.h"
#include "cyber_diver_shell.h"
#include "cyber_diver_sd_card.h"
#include "IIS3DWB.h"
#include "led.h"
#include "button.h"
#include "shell_handlers/cyber_diver_encoder_handler.h"
#include "shell_handlers/cyber_diver_power_board_handler.h"
#include "shell_handlers/cyber_diver_accel_handler.h"
#include "shell_handlers/cyber_diver_controller_handler.h"
#include "shell_handlers/cyber_diver_config_handler.h"
#include "shell_handlers/cyber_diver_logger_handler.h"
#include "usbd_cdc_if.h"

/*
 * Flags for debugging
 */

//#define CD_PROFILE
//#define CD_IGNORE_ENCODER
//#define CD_IGNORE_POWER_BOARD



/* state control */
typedef enum
{
	DIVER_IDLE,
	DIVER_HOMING,  /* waiting for the encoder to be homed */
	DIVER_TUNE,  /* connected to the USB interface, listening for serial commands */
	DIVER_STAGING,  /* give time for the user to mount the diver to the dropper */
	DIVER_ARMED,  /* waiting for acceleration trigger to run experiment */
	DIVER_RUNNING,
	DIVER_ERROR
} CyberDiverState;


typedef struct
{
	CyberDiverState state;
	uint8_t entering_state;  /* true when first entering a state */
	uint32_t time_state_entered;

	volatile uint32_t* time_micros_ptr;

	CyberDiverConfig config;  /* stores the configuration settings */

	CyberDiverEncoder encoder;
	CyberDiverController controller;  /* runs the control loops and cyber-phsyical calculations */
	CyberDiverPowerBoard power_board;  /* sends commands and gets data from the power board */
	CyberDiverShell shell;  /* handles USB communication when tuning the impactor */
	CyberDiverLogger logger;  /* maintains large data buffer for logging */

	IIS3DWB accelerometer;  /* accelerometer chip */

	LEDSequence led;  /* controls the indicator LED */
	ButtonDebounced button;  /* user input button */

} CyberDiver;


void CyberDiver_Init(CyberDiver* cd,
		volatile uint32_t* time_micros_ptr,
		TIM_HandleTypeDef* htim_loop,
		TIM_HandleTypeDef* htim_encoder,

		SPI_HandleTypeDef* power_board_spi,
		GPIO_TypeDef* power_board_cs_port,
		uint16_t power_board_cs_pin,
		GPIO_TypeDef* power_board_int1_port,
		uint16_t power_board_int1_pin,
		GPIO_TypeDef* power_board_int2_port,
		uint16_t power_board_int2_pin,

		SPI_HandleTypeDef* accelerometer_spi,
		GPIO_TypeDef* accelerometer_cs_port,
		uint16_t accelerometer_cs_pin,
		GPIO_TypeDef* accelerometer_int1_port,
		uint16_t accelerometer_int1_pin,
		GPIO_TypeDef* accelerometer_int2_port,
		uint16_t accelerometer_int2_pin,

		GPIO_TypeDef* led_port,
		uint16_t led_pin,
		GPIO_TypeDef* button_port,
		uint16_t button_pin);

void CyberDiver_Loop(CyberDiver* cd);


/* run-time critical functions in a timer interrupt */
void CyberDiver_TimerUpdate(CyberDiver* cd);

#endif /* INC_CYBER_DIVER_H_ */
