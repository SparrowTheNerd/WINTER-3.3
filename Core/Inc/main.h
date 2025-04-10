/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define LIPO_VOLT_Pin GPIO_PIN_0
#define LIPO_VOLT_GPIO_Port GPIOC
#define SENS6_Pin GPIO_PIN_0
#define SENS6_GPIO_Port GPIOA
#define PY6_Pin GPIO_PIN_1
#define PY6_GPIO_Port GPIOA
#define PY5_Pin GPIO_PIN_2
#define PY5_GPIO_Port GPIOA
#define SENS5_Pin GPIO_PIN_3
#define SENS5_GPIO_Port GPIOA
#define SENS4_Pin GPIO_PIN_4
#define SENS4_GPIO_Port GPIOA
#define PY4_Pin GPIO_PIN_5
#define PY4_GPIO_Port GPIOA
#define PY3_Pin GPIO_PIN_6
#define PY3_GPIO_Port GPIOA
#define SENS3_Pin GPIO_PIN_7
#define SENS3_GPIO_Port GPIOA
#define SENS2_Pin GPIO_PIN_4
#define SENS2_GPIO_Port GPIOC
#define PY2_Pin GPIO_PIN_5
#define PY2_GPIO_Port GPIOC
#define PY1_Pin GPIO_PIN_0
#define PY1_GPIO_Port GPIOB
#define SENS1_Pin GPIO_PIN_1
#define SENS1_GPIO_Port GPIOB
#define LoRa_BUSY_Pin GPIO_PIN_10
#define LoRa_BUSY_GPIO_Port GPIOB
#define LoRa_SCK_Pin GPIO_PIN_13
#define LoRa_SCK_GPIO_Port GPIOB
#define LoRa_MISO_Pin GPIO_PIN_14
#define LoRa_MISO_GPIO_Port GPIOB
#define LoRa_MOSI_Pin GPIO_PIN_15
#define LoRa_MOSI_GPIO_Port GPIOB
#define LoRa_CS_Pin GPIO_PIN_6
#define LoRa_CS_GPIO_Port GPIOC
#define LoRa_RST_Pin GPIO_PIN_7
#define LoRa_RST_GPIO_Port GPIOC
#define LoRa_PA_EN_Pin GPIO_PIN_10
#define LoRa_PA_EN_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_10
#define IMU_INT_GPIO_Port GPIOC
#define HIGHG_INT_Pin GPIO_PIN_11
#define HIGHG_INT_GPIO_Port GPIOC
#define MAG_INT_Pin GPIO_PIN_12
#define MAG_INT_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_2
#define SD_CS_GPIO_Port GPIOD
#define SD_SCK_Pin GPIO_PIN_3
#define SD_SCK_GPIO_Port GPIOB
#define SD_MISO_Pin GPIO_PIN_4
#define SD_MISO_GPIO_Port GPIOB
#define SD_MOSI_Pin GPIO_PIN_5
#define SD_MOSI_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_6
#define SERVO4_GPIO_Port GPIOB
#define SERVO3_Pin GPIO_PIN_7
#define SERVO3_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_8
#define SERVO2_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_9
#define SERVO1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
