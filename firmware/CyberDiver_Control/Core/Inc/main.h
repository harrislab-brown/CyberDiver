/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EXP_INT_Pin GPIO_PIN_3
#define EXP_INT_GPIO_Port GPIOE
#define LED_STATUS_Pin GPIO_PIN_14
#define LED_STATUS_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_15
#define BUTTON_GPIO_Port GPIOC
#define ENC_IW_Pin GPIO_PIN_2
#define ENC_IW_GPIO_Port GPIOA
#define SPI6_NSS_Pin GPIO_PIN_4
#define SPI6_NSS_GPIO_Port GPIOA
#define SD_DET_Pin GPIO_PIN_11
#define SD_DET_GPIO_Port GPIOD
#define PWR_INT2_Pin GPIO_PIN_4
#define PWR_INT2_GPIO_Port GPIOD
#define PWR_INT1_Pin GPIO_PIN_5
#define PWR_INT1_GPIO_Port GPIOD
#define SPI1_NSS_Pin GPIO_PIN_10
#define SPI1_NSS_GPIO_Port GPIOG
#define IIS3DWB_INT1_Pin GPIO_PIN_4
#define IIS3DWB_INT1_GPIO_Port GPIOB
#define IIS3DWB_INT1_EXTI_IRQn EXTI4_IRQn
#define IIS3DWB_INT2_Pin GPIO_PIN_5
#define IIS3DWB_INT2_GPIO_Port GPIOB
#define ADXL372_INT1_Pin GPIO_PIN_6
#define ADXL372_INT1_GPIO_Port GPIOB
#define ADXL372_INT2_Pin GPIO_PIN_7
#define ADXL372_INT2_GPIO_Port GPIOB
#define LSM6DS3_INT1_Pin GPIO_PIN_8
#define LSM6DS3_INT1_GPIO_Port GPIOB
#define LSM6DS3_INT2_Pin GPIO_PIN_9
#define LSM6DS3_INT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
