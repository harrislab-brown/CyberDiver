/*
 * IIS3DWB.c
 *
 *  Created on: Jul 16, 2024
 *      Author: johnt
 */

#include "IIS3DWB.h"
#include "fast_spi.h"


void IIS3DWB_Init(IIS3DWB* sensor, IIS3DWB_Config* config, SPI_HandleTypeDef* spi,
		GPIO_TypeDef* cs_port, uint16_t cs_pin,
		GPIO_TypeDef* int1_port, uint16_t int1_pin,
		GPIO_TypeDef* int2_port, uint16_t int2_pin)
{

	sensor->config = config;
	sensor->spi = spi;
	sensor->cs_port = cs_port;
	sensor->cs_pin = cs_pin;
	sensor->int1_port = int1_port;
	sensor->int1_pin = int1_pin;
	sensor->int2_port = int2_port;
	sensor->int2_pin = int2_pin;

	sensor->accel_x_g = 0.0f;
	sensor->accel_y_g = 0.0f;
	sensor->accel_z_g = 0.0f;

	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

}


uint32_t IIS3DWB_Configure(IIS3DWB* sensor)
{

	HAL_NVIC_DisableIRQ(IIS3DWB_INTERRUPT);  /* disable the accelerometer pin interrupts while initializing */

	/* verify that we can communicate with the sensor */
	uint8_t reg_data;
	HAL_StatusTypeDef status = IIS3DWB_ReadRegister(sensor, IIS3DWB_REG_WHO_AM_I, &reg_data);
	if (status != HAL_OK || reg_data != IIS3DWB_DEVICE_ID)
		return 0;  /* return with error if we receive unexpected device ID */


	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_CTRL1_XL, IIS3DWB_ODR_DISABLE);  	/* power down the device (p. 32) */
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_CTRL4_C, 0x04);  					/* disable the I2C interface (p. 34) */
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_CTRL6_C, 0x00);						/* select three axis sampling mode and set weight of user offsets to 2^(-10) g/LSB (p. 35) */
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_COUNTER_BDR_REG1, 0x80);			/* set the data-ready interrupt to pulse mode (p. 30) */
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_INT1_CTRL, 0x01);					/* enable the data-ready interrupt on INT1 pin (p. 31) */
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_CTRL7_C, 0x02);						/* enable the user offset correction block (p. 35) */



	/* enable the low pass filter with configured cutoff (p. 36) */
	uint8_t lpf_data;
	switch(sensor->config->lpf)
	{
	case 4:
		lpf_data = IIS3DWB_LPF_4;
		break;
	case 10:
		lpf_data = IIS3DWB_LPF_10;
		break;
	case 20:
		lpf_data = IIS3DWB_LPF_20;
		break;
	case 45:
		lpf_data = IIS3DWB_LPF_45;
		break;
	case 100:
		lpf_data = IIS3DWB_LPF_100;
		break;
	case 200:
		lpf_data = IIS3DWB_LPF_200;
		break;
	case 400:
		lpf_data = IIS3DWB_LPF_400;
		break;
	case 800:
		lpf_data = IIS3DWB_LPF_800;
		break;
	default:
		lpf_data = IIS3DWB_LPF_20;
		break;
	}

	reg_data = (IIS3DWB_LPF_ENABLE | lpf_data);
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_CTRL8_XL, reg_data);


	/* x, y, z are the DC offsets of the sensor in g */
	/* we assume the weight of the user offsets is 2^(-10) g/LSB */
	int8_t x_b = (int8_t)(sensor->config->usr_offset_x / IIS3DWB_OFFSET_WEIGHT);
	int8_t y_b = (int8_t)(sensor->config->usr_offset_y / IIS3DWB_OFFSET_WEIGHT);
	int8_t z_b = (int8_t)(sensor->config->usr_offset_z / IIS3DWB_OFFSET_WEIGHT);
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_X_OFS_USR, x_b);
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_Y_OFS_USR, y_b);
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_Z_OFS_USR, z_b);


	/* enable the sensor with the configured range (p. 32) */
	uint8_t range_data;
	switch(sensor->config->g_range)
	{
	case 2:
		range_data = IIS3DWB_G_RANGE_2;
		break;
	case 4:
		range_data = IIS3DWB_G_RANGE_4;
		break;
	case 8:
		range_data = IIS3DWB_G_RANGE_8;
		break;
	case 16:
		range_data = IIS3DWB_G_RANGE_16;
		break;
	default:
		range_data = IIS3DWB_G_RANGE_2;
		break;
	}

	reg_data = (IIS3DWB_ODR_26667HZ | range_data);
	(void)IIS3DWB_WriteRegister(sensor, IIS3DWB_REG_CTRL1_XL, reg_data);  /* enables the accelerometer with the selected range and also enables the LPF2 filter path (p. 32) */

	HAL_NVIC_EnableIRQ(IIS3DWB_INTERRUPT);

	return 1;
}


void IIS3DWB_GetAcceleration(IIS3DWB* sensor, float* x, float* y, float* z)
{
	*x = sensor->accel_x_g;
	*y = sensor->accel_y_g;
	*z = sensor->accel_z_g;
}


uint32_t IIS3DWB_SetRange(IIS3DWB* sensor, uint32_t g_range)
{
	uint32_t len = sizeof(IIS3DWB_ALLOWED_G_RANGE) / sizeof(IIS3DWB_ALLOWED_G_RANGE[0]);
	uint32_t i;
	for (i = 0; i < len; i++)
		if (g_range == IIS3DWB_ALLOWED_G_RANGE[i])  /* make sure the requested setting is allowed */
			break;

	if (i == len)
		return 0;

	uint32_t g_range_old = sensor->config->g_range;
	sensor->config->g_range = g_range;

	/* update the hardware */
	if (IIS3DWB_Configure(sensor))
	{
		return 1;
	}
	else
	{
		/* revert the settings */
		sensor->config->g_range = g_range_old;
	}

	return 0;
}


uint32_t IIS3DWB_GetRange(IIS3DWB* sensor)
{
	return sensor->config->g_range;
}


uint32_t IIS3DWB_SetLPF(IIS3DWB* sensor, uint32_t lpf)
{
	uint32_t len = sizeof(IIS3DWB_ALLOWED_LPF) / sizeof(IIS3DWB_ALLOWED_LPF[0]);
	uint32_t i;
	for (i = 0; i < len; i++)
		if (lpf == IIS3DWB_ALLOWED_LPF[i])  /* make sure the requested setting is allowed */
			break;

	if (i == len)
		return 0;

	uint32_t lpf_old = sensor->config->lpf;
	sensor->config->lpf = lpf;

	/* update the hardware */
	if (IIS3DWB_Configure(sensor))
	{
		return 1;
	}
	else
	{
		/* revert the settings */
		sensor->config->lpf = lpf_old;
	}

	return 0;
}


uint32_t IIS3DWB_GetLPF(IIS3DWB* sensor)
{
	return sensor->config->lpf;
}


uint32_t IIS3DWB_SetOffsets(IIS3DWB* sensor, float x, float y, float z)
{
	/* clamp the offsets to the max value that can fit in the register */
	if (x > IIS3DWB_MAX_OFFSET) x = IIS3DWB_MAX_OFFSET;
	if (x < -IIS3DWB_MAX_OFFSET) x = -IIS3DWB_MAX_OFFSET;
	if (y > IIS3DWB_MAX_OFFSET) y = IIS3DWB_MAX_OFFSET;
	if (y < -IIS3DWB_MAX_OFFSET) y = -IIS3DWB_MAX_OFFSET;
	if (z > IIS3DWB_MAX_OFFSET) z = IIS3DWB_MAX_OFFSET;
	if (z < -IIS3DWB_MAX_OFFSET) z = -IIS3DWB_MAX_OFFSET;

	/* calculate the closest binary values */
	int8_t x_b = (int8_t)(x / IIS3DWB_OFFSET_WEIGHT);
	int8_t y_b = (int8_t)(y / IIS3DWB_OFFSET_WEIGHT);
	int8_t z_b = (int8_t)(z / IIS3DWB_OFFSET_WEIGHT);

	/* keep the old values */
	float x_old = sensor->config->usr_offset_x;
	float y_old = sensor->config->usr_offset_y;
	float z_old = sensor->config->usr_offset_z;

	/* convert back to floating point  */
	sensor->config->usr_offset_x = (float)x_b * IIS3DWB_OFFSET_WEIGHT;
	sensor->config->usr_offset_y = (float)y_b * IIS3DWB_OFFSET_WEIGHT;
	sensor->config->usr_offset_z = (float)z_b * IIS3DWB_OFFSET_WEIGHT;

	/* update the hardware */
	if (IIS3DWB_Configure(sensor))
	{
		return 1;
	}
	else
	{
		/* revert the settings */
		sensor->config->usr_offset_x = x_old;
		sensor->config->usr_offset_y = y_old;
		sensor->config->usr_offset_z = z_old;
	}

	return 0;
}


void IIS3DWB_GetOffsets(IIS3DWB* sensor, float* x, float* y, float* z)
{
	*x = sensor->config->usr_offset_x;
	*y = sensor->config->usr_offset_y;
	*z = sensor->config->usr_offset_z;
}


HAL_StatusTypeDef IIS3DWB_ReadRegister(IIS3DWB* sensor, uint8_t reg, uint8_t* data)
{
	uint8_t tx_buf[2] = {(reg | 0x80), 0x00};  // set the first bit to indicate a read communication
	uint8_t rx_buf[2];

	HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(sensor->spi, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

	*data = rx_buf[1];

	return status;
}


HAL_StatusTypeDef IIS3DWB_WriteRegister(IIS3DWB* sensor, uint8_t reg, uint8_t data)
{
	uint8_t tx_buf[2] = {reg, data};

	HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(sensor->spi, tx_buf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

	return status;
}


uint8_t IIS3DWB_GetInt1State(IIS3DWB* sensor)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(sensor->int1_port, sensor->int1_pin);
}


uint8_t IIS3DWB_GetInt2State(IIS3DWB* sensor)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(sensor->int2_port, sensor->int2_pin);
}


void IIS3DWB_ReadAcceleration(IIS3DWB* sensor)
{
	/* read multiple bytes corresponding to the raw accelerometer data */
	uint8_t tx_buf[7] = {(IIS3DWB_REG_OUTX_L_XL | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx_buf[7];

	/* use our fast SPI implementation instead of the HAL call */
	SPI_TxRx_Fast(tx_buf, rx_buf, 7, sensor->spi->Instance, sensor->cs_port, sensor->cs_pin);

	/* convert the raw readings to physical units */
	int16_t raw_data_x = ((int16_t)(rx_buf[1])) | (((int16_t)(rx_buf[2])) << 8);
	int16_t raw_data_y = ((int16_t)(rx_buf[3])) | (((int16_t)(rx_buf[4])) << 8);
	int16_t raw_data_z = ((int16_t)(rx_buf[5])) | (((int16_t)(rx_buf[6])) << 8);

	sensor->accel_x_g = sensor->config->g_range * (float)raw_data_x / (float)(1 << (IIS3DWB_RESOLUTION - 1));
	sensor->accel_y_g = sensor->config->g_range * (float)raw_data_y / (float)(1 << (IIS3DWB_RESOLUTION - 1));
	sensor->accel_z_g = sensor->config->g_range * (float)raw_data_z / (float)(1 << (IIS3DWB_RESOLUTION - 1));

}
