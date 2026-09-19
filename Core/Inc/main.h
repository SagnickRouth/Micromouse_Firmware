/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define MOTOR_STBY_Pin GPIO_PIN_10
#define MOTOR_STBY_GPIO_Port GPIOB
#define MOTOR_AIN1_Pin GPIO_PIN_12
#define MOTOR_AIN1_GPIO_Port GPIOB
#define MOTOR_AIN2_Pin GPIO_PIN_13
#define MOTOR_AIN2_GPIO_Port GPIOB
#define MOTOR_BIN1_Pin GPIO_PIN_14
#define MOTOR_BIN1_GPIO_Port GPIOB
#define MOTOR_BIN2_Pin GPIO_PIN_15
#define MOTOR_BIN2_GPIO_Port GPIOB
#define VL53_LFWD_XSHUT_Pin GPIO_PIN_0
#define VL53_LFWD_XSHUT_GPIO_Port GPIOB
#define VL53_LDIAG_XSHUT_Pin GPIO_PIN_1
#define VL53_LDIAG_XSHUT_GPIO_Port GPIOB
#define VL53_RDIAG_XSHUT_Pin GPIO_PIN_1
#define VL53_RDIAG_XSHUT_GPIO_Port GPIOA
#define VL53_RFWD_XSHUT_Pin GPIO_PIN_2
#define VL53_RFWD_XSHUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
