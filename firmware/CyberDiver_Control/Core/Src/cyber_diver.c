/*
 * cyber_diver.c
 *
 *  Created on: Jul 6, 2024
 *      Author: johnt
 */

#include "cyber_diver.h"

#define LEN(x) sizeof(x)/sizeof(x[0])
#define ARR_LEN(x) (x),LEN(x)
#define LED_ARR(x) &(cd->led),ARR_LEN(x)

static const uint32_t led_idle_sequence[] = {1000000, 100000};
#define LED_IDLE LED_ARR(led_idle_sequence)
static const uint32_t led_homing_sequence[] = {100000, 100000};
#define LED_HOMING LED_ARR(led_homing_sequence)
static const uint32_t led_staging_sequence[] = {1000000, 100000, 100000, 100000};
#define LED_STAGING LED_ARR(led_staging_sequence)
static const uint32_t led_armed_sequence[] = {1000000, 100000, 100000, 100000, 100000, 100000};
#define LED_ARMED LED_ARR(led_armed_sequence)
static const uint32_t led_success_sequence[] = {100000, 100000, 100000, 100000};
#define LED_SUCCESS LED_ARR(led_success_sequence)
static const uint32_t led_error_sequence[] = {500000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};
#define LED_ERROR LED_ARR(led_error_sequence)
static const uint32_t led_default_config_sequence[] = {100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};
#define LED_DEFAULT_CONFIG LED_ARR(led_default_config_sequence)


static uint32_t shell_connected = 0;


static void CyberDiver_GoToState(CyberDiver* cd, CyberDiverState state)
{
	cd->state = state;
	cd->entering_state = 1;
}


static uint32_t CyberDiver_Reset(CyberDiver* cd)
{
	/* This should be called at startup or when a new configuration is loaded from a file */

	/* reset the logger */
	CyberDiverLogger_Reset(&cd->logger);

	/* reset the controller (recompute LUTs and set the control loop timer registers) */
	CyberDiverController_Reset(&cd->controller);

	/* accelerometer */
	if (!IIS3DWB_Configure(&cd->accelerometer))
		return 0;

	/* power board */
	if (!CyberDiverPowerBoard_Reset(&cd->power_board))
	{
#ifndef CD_IGNORE_POWER_BOARD
		return 0;
#endif
	}


	return 1;
}



static uint32_t CyberDiverConnectCMD_Execute(void* obj, CyberDiverShell* shell)
{
	CyberDiverShell_PutOutputString(shell, "ack");
	CyberDiverShell_PutOutputDelimiter(shell);
	shell_connected = 1;
	return 1;
}

static uint32_t CyberDiverDisconnectCMD_Execute(void* obj, CyberDiverShell* shell)
{
	CyberDiverShell_PutOutputString(shell, "ack");
	CyberDiverShell_PutOutputDelimiter(shell);
	shell_connected = 0;
	return 1;
}

static uint32_t CyberDiverConfigCMD_Execute(void* obj, CyberDiverShell* shell)
{
	CyberDiver* cd = (CyberDiver*)obj;

	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "save"))
		{
			if (CyberDiverConfig_WriteConfigToSD(&cd->config))
			{
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "load"))
		{
			if (CyberDiverConfig_GetConfigFromSD(&cd->config) && CyberDiver_Reset(cd))
			{
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
		else if (!strcmp(str, "set"))  /* call the subsystem specific handlers */
		{
			if (CyberDiverConfigCMD_Set(&cd->config, shell))
				return 1;
		}
		else if (!strcmp(str, "get"))
		{
			if (CyberDiverConfigCMD_Get(&cd->config, shell))
				return 1;
		}
	}

	return 0;
}

static uint32_t CyberDiverEncoderCMD_Execute(void* obj, CyberDiverShell* shell)
{
	CyberDiver* cd = (CyberDiver*)obj;

	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "home"))
		{
			CyberDiver_GoToState(cd, DIVER_HOMING);
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "set"))  /* call the subsystem specific handlers if we are not homing */
		{
			if (CyberDiverEncoderCMD_Set(&cd->encoder, shell))
				return 1;
		}
		else if (!strcmp(str, "get"))
		{
			if (CyberDiverEncoderCMD_Get(&cd->encoder, shell))
				return 1;
		}
	}

	return 0;
}



static void CyberDiver_RegisterIdleShell(CyberDiver* cd)
{
	CyberDiverShell_ClearInputHandlers(&cd->shell);

	CyberDiverShell_InputHandler connect_cmd = {
		.name = "connect",
		.execute = CyberDiverConnectCMD_Execute,
		.obj = NULL
	};

	CyberDiverShell_RegisterInputHandler(&cd->shell, connect_cmd);
}


static void CyberDiver_RegisterTuneShell(CyberDiver* cd)
{
	CyberDiverShell_ClearInputHandlers(&cd->shell);

	CyberDiverShell_InputHandler disconnect_cmd = {
		.name = "disconnect",
		.execute = CyberDiverDisconnectCMD_Execute,
		.obj = NULL
	};

	CyberDiverShell_InputHandler config_cmd = {
		.name = "config",
		.execute = CyberDiverConfigCMD_Execute,
		.obj = cd
	};

	CyberDiverShell_InputHandler encoder_cmd = {
		.name = "encoder",
		.execute = CyberDiverEncoderCMD_Execute,
		.obj = cd
	};

	CyberDiverShell_InputHandler powerboard_cmd = {
		.name = "powerboard",
		.execute = CyberDiverPowerBoardCMD_Execute,
		.obj = &cd->power_board
	};

	CyberDiverShell_InputHandler accel_cmd = {
		.name = "accel",
		.execute = CyberDiverAccelCMD_Execute,
		.obj = &cd->accelerometer
	};

	CyberDiverShell_InputHandler controller_cmd = {
		.name = "controller",
		.execute = CyberDiverControllerCMD_Execute,
		.obj = &cd->controller
	};

	CyberDiverShell_InputHandler logger_cmd = {
		.name = "logger",
		.execute = CyberDiverLoggerCMD_Execute,
		.obj = &cd->logger
	};


	CyberDiverShell_RegisterInputHandler(&cd->shell, disconnect_cmd);
	CyberDiverShell_RegisterInputHandler(&cd->shell, config_cmd);
	CyberDiverShell_RegisterInputHandler(&cd->shell, encoder_cmd);
	CyberDiverShell_RegisterInputHandler(&cd->shell, powerboard_cmd);
	CyberDiverShell_RegisterInputHandler(&cd->shell, accel_cmd);
	CyberDiverShell_RegisterInputHandler(&cd->shell, controller_cmd);
	CyberDiverShell_RegisterInputHandler(&cd->shell, logger_cmd);


	CyberDiverShell_OutputHandler logger_sender = {
		.execute = CyberDiverLoggerSender_Execute,
		.obj = &cd->logger
	};


	CyberDiverShell_RegisterOutputHandler(&cd->shell, logger_sender);

}


static void CyberDiver_Idle(CyberDiver* cd)
{
	if (cd->entering_state)
	{
		CyberDiverPowerBoard_SetMode(&cd->power_board, POWER_BOARD_IDLE);  /* disable the H-bridge */
		LEDSequence_SetBlinkSequence(LED_IDLE);
		CyberDiver_RegisterIdleShell(cd);  /* set up the shell commands for the idle state */
		CyberDiverController_SetState(&cd->controller, cd->config.idle_controller_state);  /* set the controller state */
		cd->entering_state = 0;
	}

	/* check if we want to connect over serial to tune the impactor parameters */
	CyberDiverShell_Status shell_status = CyberDiverShell_Update(&cd->shell);
	(void)shell_status;
	if (shell_connected)
	{
		CyberDiver_GoToState(cd, DIVER_TUNE);
	}

	/* press the button to enter the experiment staging state */
	if (ButtonDebounced_GetPressed(&(cd->button)))
	{
		CyberDiver_GoToState(cd, DIVER_STAGING);
	}

}


static void CyberDiver_Homing(CyberDiver* cd)
{
	if (cd->entering_state)
	{
		CyberDiverPowerBoard_SetMode(&cd->power_board, POWER_BOARD_IDLE);  /* disable the H-bridge */
		CyberDiverController_SetMode(&cd->controller, CONTROLLER_IDLE);  /* make sure we're not driving the coil while homing */
		LEDSequence_SetBlinkSequence(LED_HOMING);
		CyberDiverEncoder_Home(&cd->encoder);  /* tell the encoder to start looking for the index */
		cd->entering_state = 0;
	}

	/* home the encoder */
	CyberDiverEncoder_Update(&cd->encoder);


	if (CyberDiverEncoder_GetHomed(&cd->encoder) || ButtonDebounced_GetPressed(&cd->button))  /* use the button to escape if we get stuck homing */
	{
		if (shell_connected)
			CyberDiver_GoToState(cd, DIVER_TUNE);  /* go back to tune state if we homed while tuning */
		else
			CyberDiver_GoToState(cd, DIVER_IDLE);
	}
}


static void CyberDiver_Tune(CyberDiver* cd)
{

	if (cd->entering_state)
	{
		LEDSequence_Stop(&(cd->led));
		LEDSequence_SetBurstSequence(LED_SUCCESS);
		CyberDiver_RegisterTuneShell(cd);  /* set up the shell commands for the tuning state */
		cd->entering_state = 0;
	}


	/* update the shell */
	/* (data enters the shell in usbd_cdc_if.c in the function CDC_Receive_HS) */
	CyberDiverShell_Status shell_status = CyberDiverShell_Update(&cd->shell);
	if (shell_status.ihandl_status == CD_SHELL_INPUT_PROCESSED)
		LEDSequence_SetBurstSequence(LED_SUCCESS);  /* blink when we process a command successfully */


	/* send any available data over USB */
	/* (the shell input and output handlers are responsible for populating the data to send in the shell output buffer) */
	char* usb_tx;
	uint32_t usb_tx_len;
	if (CyberDiverShell_GetOutput(&cd->shell, &usb_tx, &usb_tx_len))
		if (CDC_Transmit_HS((uint8_t*)usb_tx, usb_tx_len) == USBD_OK)
			CyberDiverShell_UpdateOutputBuffer(&cd->shell, usb_tx_len);


	if (!shell_connected)  /* would be nice to detect disconnection also when the USB is physically unplugged, but not sure if there is an easy way */
	{
		CyberDiver_GoToState(cd, DIVER_IDLE);
	}
}


static void CyberDiver_Staging(CyberDiver* cd)
{
	/* give time for the user to mount the diver to the dropper */

	if (cd->entering_state)
	{
		LEDSequence_SetBlinkSequence(LED_STAGING);
		cd->time_state_entered = *cd->time_micros_ptr;  /* start the count down to the experiment */
		cd->entering_state = 0;
	}

	if(*cd->time_micros_ptr - cd->time_state_entered > cd->config.staging_time_us)
	{
		CyberDiver_GoToState(cd, DIVER_ARMED);
	}

	/* to terminate an experiment, press the button to go back to the idle state */
	if (ButtonDebounced_GetPressed(&cd->button))
	{
		CyberDiver_GoToState(cd, DIVER_IDLE);
	}
}


static void CyberDiver_Armed(CyberDiver* cd)
{
	/* waiting for acceleration trigger to run experiment */

	if (cd->entering_state)
	{
		LEDSequence_SetBlinkSequence(LED_ARMED);
		CyberDiverPowerBoard_SetMode(&cd->power_board, POWER_BOARD_RUNNING);  /* enable the H-bridge */
		CyberDiverController_SetState(&cd->controller, cd->config.armed_controller_state);  /* set the controller state */


		if (!CyberDiverSD_CreateDataFile()) /* create and open the data file */
		{
			CyberDiver_GoToState(cd, DIVER_ERROR);
			return;
		}


		if (!CyberDiverConfig_WriteConfigToFile(&cd->config, CyberDiverSD_GetFile()))  /* write the configuration to the data file before writing the data for this experiment */
		{
			CyberDiver_GoToState(cd, DIVER_ERROR);
			return;
		}

		cd->entering_state = 0;
	}

	/* trigger the next state based on accelerometer free-fall threshold */
	if (fabs(CyberDiverController_GetAcceleration(&cd->controller)) < cd->config.trigger_accel_threshold)
	{
		CyberDiver_GoToState(cd, DIVER_RUNNING);
	}

}


static void CyberDiver_Running(CyberDiver* cd)
{
	if (cd->entering_state)
	{
		LEDSequence_Stop(&cd->led);  /* disable the led blink sequence so that it can be used to verify impact timing */
		CyberDiverLogger_Reset(&cd->logger);
		CyberDiverLogger_StartLogging(&cd->logger);  /* start the logger */
		CyberDiverController_StartSequence(&cd->controller);  /* start the cyber diver controller experimental sequence */
		cd->time_state_entered = *cd->time_micros_ptr;
		cd->entering_state = 0;
	}

	/* record the experimental data */
	volatile uint8_t* data;
	uint32_t size;
	if (CyberDiverLogger_GetCompletePacket(&cd->logger, &data, &size))
	{
		if (!CyberDiverSD_WriteData((uint8_t*)data, size))
		{
			CyberDiver_GoToState(cd, DIVER_ERROR);
			return;
		}
	}

	/* stop the experiment on button press or after a certain amount of time */
	if (ButtonDebounced_GetPressed(&cd->button) || *cd->time_micros_ptr - cd->time_state_entered > cd->config.running_time_us)
	{
		CyberDiverController_StopSequence(&cd->controller);
		CyberDiverLogger_StopLogging(&cd->logger);

		/* save any remaining data */
		volatile uint8_t* data;
		uint32_t size;
		while (CyberDiverLogger_GetPacket(&cd->logger, &data, &size))
		{
			if (!CyberDiverSD_WriteData((uint8_t*)data, size))
			{
				CyberDiver_GoToState(cd, DIVER_ERROR);
				return;
			}
		}

		if (!CyberDiverSD_SaveDataFile())
		{
			CyberDiver_GoToState(cd, DIVER_ERROR);
			return;
		}

		CyberDiver_GoToState(cd, DIVER_IDLE);
	}
}


static void CyberDiver_Error(CyberDiver* cd)
{
	if (cd->entering_state)
	{
		CyberDiverPowerBoard_SetMode(&cd->power_board, POWER_BOARD_IDLE);  /* disable the H-bridge */
		CyberDiverController_StopSequence(&cd->controller);
		CyberDiverController_SetMode(&cd->controller, CONTROLLER_IDLE);
		CyberDiverLogger_StopLogging(&cd->logger);
		LEDSequence_SetBlinkSequence(LED_ERROR);
		cd->entering_state = 0;
	}
}


/*
 * What should happen with the states of everybody at startup and while running?
 *
 * Encoder: the quadrature hardware is always counting ticks in the background so
 * this one is easy enough. We just have to wait for it to be homed when we first
 * start up, and whenever the user requests for it to be homed over the shell. We
 * should also re-home the encoder after every experimental trial to make sure
 * the position doesn't get lost. There should be some logic that blocks the output
 * of the controller (i.e. don't do anything with the coil) if the encoder is not zeroed.
 *
 * Accelerometer: should always be running, so configure its registers at startup
 * and then just let it run. If we load a new configuration file or change an
 * accelerometer parameter through the shell we should also configure the registers
 * of the sensor again. While doing the configuration, we need to stop the EXTI
 * interrupts that are linked to the sensor data ready pin.
 *
 * Power board: also can always be in run mode, except when being configured at
 * startup, when loading a new configuration file, or when its parameters are
 * being tuned through the shell. The user can enable/disable the power stage by
 * pressing the physical button on the power board. Ideally it might be nice for
 * the control board to be able to enable/disable the power stage (or to know its state)
 * but this would require extra data on the inter-board SPI bus. I've carefully
 * minimized the amount of data to be transfered between the boards to reduce
 * the latency of the overall control system.
 *
 * Controller: after initialization, the main controller can just continuously run in
 * the timer interrupt. However, there should be guards in place so that only reasonable
 * values are sent to the power board. The power board class itself guards against sending
 * commands to the power board when it is in the idle state, so all we really need to
 * worry about is whether the encoder data can be trusted. I could have the encoder class
 * output a default position (i.e. zero) when it is being homed, which should be safe enough.
 * But it might produce a large transient velocity estimate. So maybe just not mess with the
 * encoder value at all. Instead just be sure to put the controller in idle mode whenever
 * we are homing the encoder.
 *
 */


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
		uint16_t button_pin)
{

	HAL_NVIC_DisableIRQ(IIS3DWB_INTERRUPT);  /* disable the IIS3DWB accelerometer pin interrupt */

	HAL_Delay(500);  /* give the power board ample time to initialize */

	cd->time_micros_ptr = time_micros_ptr;

	uint8_t config_from_file = CyberDiverConfig_Init(&cd->config);

	CyberDiverEncoder_Init(&cd->encoder, &cd->config.encoder_config, htim_encoder);

	CyberDiverController_Init(&cd->controller, &cd->config.controller_config, time_micros_ptr, htim_loop);

	CyberDiverPowerBoard_Init(&cd->power_board,
			&cd->config.power_board_config,
			power_board_spi,
			power_board_cs_port,
			power_board_cs_pin,
			power_board_int1_port,
			power_board_int1_pin,
			power_board_int2_port,
			power_board_int2_pin);

	CyberDiverShell_Init(&cd->shell);

	CyberDiverLogger_Init(&cd->logger, &cd->config.logger_config, sizeof(CyberDiverControllerDataPoint) / 4);

	LEDSequence_Init(&cd->led, led_port, led_pin, time_micros_ptr);

	ButtonDebounced_Init(&cd->button, button_port, button_pin, time_micros_ptr, 250000);

	IIS3DWB_Init(&cd->accelerometer, &cd->config.accelerometer_config, accelerometer_spi,
			accelerometer_cs_port, accelerometer_cs_pin,
			accelerometer_int1_port, accelerometer_int1_pin,
			accelerometer_int2_port, accelerometer_int2_pin);

	if (CyberDiver_Reset(cd))
	{
		if (config_from_file)
		{
			/* indicate successful initialization with the LED */
			LEDSequence_SetBurstSequence(LED_SUCCESS);
		}
		else
		{
			/* indicate that we are using the default configuration parameters */
			LEDSequence_SetBurstSequence(LED_DEFAULT_CONFIG);
			CyberDiverConfig_WriteConfigToSD(&cd->config);  /* write the default configuration to the SD card so we have a starting point */
		}


		CyberDiverController_StartTimer(&cd->controller);


#ifdef CD_IGNORE_ENCODER
		CyberDiver_GoToState(cd, DIVER_IDLE);
#else
		CyberDiver_GoToState(cd, DIVER_HOMING);
#endif

	}
	else
	{
		/* go to error state */
		CyberDiver_GoToState(cd, DIVER_ERROR);
	}

}


void CyberDiver_Loop(CyberDiver* cd)
{
	switch (cd->state)
	{
	case DIVER_IDLE:
		CyberDiver_Idle(cd);
		break;

	case DIVER_HOMING:
		CyberDiver_Homing(cd);
		break;

	case DIVER_TUNE:
		CyberDiver_Tune(cd);
		break;

	case DIVER_STAGING:
		CyberDiver_Staging(cd);
		break;

	case DIVER_ARMED:
		CyberDiver_Armed(cd);
		break;

	case DIVER_RUNNING:
		CyberDiver_Running(cd);
		break;

	case DIVER_ERROR:
		CyberDiver_Error(cd);
		break;
	}

	/* Put anything low priority that we want to always be running here */
	LEDSequence_Update(&(cd->led));  /* update the indicator LED */

}


void CyberDiver_TimerUpdate(CyberDiver* cd)
{
	/* time-critical update function */

/*
 * Blip the INT1 pin in order to profile the control loop with a scope.
 * When the full system is running, the INT1 pin is used to set the command mode of the
 * power board so make sure not to blip the pin when the power board is connected
 */


#ifdef CD_PROFILE
	volatile GPIO_TypeDef* port = PWR_INT1_GPIO_Port;
	port->BSRR = PWR_INT1_Pin;
#endif

	/* get the latest acceleration measurements */
	float accel_x, accel_y, accel_z;
	IIS3DWB_GetAcceleration(&cd->accelerometer, &accel_x, &accel_y, &accel_z);

	/* update the cyber-diver controller */
	CyberDiverController_Update(&cd->controller,
			CyberDiverEncoder_GetPosition(&cd->encoder),
			CyberDiverPowerBoard_GetCurrent(&cd->power_board),
			CyberDiverPowerBoard_GetDutyCycle(&cd->power_board),
			accel_x, accel_y, accel_z);


	/* send the controller output to the power board and get the most recent coil data */
	CyberDiverPowerBoard_SetCurrent(&cd->power_board, CyberDiverController_GetOutput(&cd->controller), CyberDiverController_GetVelocity(&cd->controller));

	/* set the state of the LED according to the controller if the LED is not currently running a sequence */
	if (!LEDSequence_GetIsRunning(&cd->led))
		LEDSequence_SetState(&cd->led, CyberDiverController_GetLED(&cd->controller));

	/* log the data */
	CyberDiverControllerDataPoint data = CyberDiverController_GetData(&(cd->controller));
	CyberDiverLogger_AddData(&cd->logger, (uint32_t*)&data);


#ifdef CD_PROFILE
	port->BSRR = (uint32_t)PWR_INT1_Pin << 16U;
#endif
}
