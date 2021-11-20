/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define SERVO06_Pin GPIO_PIN_2
#define SERVO06_GPIO_Port GPIOE
#define SERVO07_Pin GPIO_PIN_3
#define SERVO07_GPIO_Port GPIOE
#define SERVO08_Pin GPIO_PIN_4
#define SERVO08_GPIO_Port GPIOE
#define SERVO15_Pin GPIO_PIN_5
#define SERVO15_GPIO_Port GPIOE
#define SERVO10_Pin GPIO_PIN_2
#define SERVO10_GPIO_Port GPIOB
#define SERVO09_Pin GPIO_PIN_7
#define SERVO09_GPIO_Port GPIOE
#define SERVO05_Pin GPIO_PIN_8
#define SERVO05_GPIO_Port GPIOE
#define SERVO04_Pin GPIO_PIN_9
#define SERVO04_GPIO_Port GPIOE
#define SERVO03_Pin GPIO_PIN_10
#define SERVO03_GPIO_Port GPIOE
#define SERVO14_Pin GPIO_PIN_12
#define SERVO14_GPIO_Port GPIOB
#define SERVO13_Pin GPIO_PIN_13
#define SERVO13_GPIO_Port GPIOB
#define SERVO12_Pin GPIO_PIN_14
#define SERVO12_GPIO_Port GPIOB
#define SERVO11_Pin GPIO_PIN_15
#define SERVO11_GPIO_Port GPIOA
#define SERVO16_Pin GPIO_PIN_10
#define SERVO16_GPIO_Port GPIOC
#define SERVO17_Pin GPIO_PIN_11
#define SERVO17_GPIO_Port GPIOC
#define SERVO00_Pin GPIO_PIN_12
#define SERVO00_GPIO_Port GPIOC
#define SERVO01_Pin GPIO_PIN_0
#define SERVO01_GPIO_Port GPIOD
#define SERVO02_Pin GPIO_PIN_1
#define SERVO02_GPIO_Port GPIOD
#define INOUT_Pin GPIO_PIN_8
#define INOUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
