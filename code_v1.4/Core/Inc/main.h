/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define ADE2_SS_Pin GPIO_PIN_3
#define ADE2_SS_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define ADE_SCK_Pin GPIO_PIN_5
#define ADE_SCK_GPIO_Port GPIOA
#define ADE_MISO_Pin GPIO_PIN_6
#define ADE_MISO_GPIO_Port GPIOA
#define ADE_MOSI_Pin GPIO_PIN_7
#define ADE_MOSI_GPIO_Port GPIOA
#define stop_discharging_Pin GPIO_PIN_0
#define stop_discharging_GPIO_Port GPIOB
#define stop_charging_Pin GPIO_PIN_1
#define stop_charging_GPIO_Port GPIOB
#define ADE2_SS2_Pin GPIO_PIN_12
#define ADE2_SS2_GPIO_Port GPIOB
#define ADE2_SCK_Pin GPIO_PIN_13
#define ADE2_SCK_GPIO_Port GPIOB
#define ADE2_MISO_Pin GPIO_PIN_14
#define ADE2_MISO_GPIO_Port GPIOB
#define ADE2_MOSI_Pin GPIO_PIN_15
#define ADE2_MOSI_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_11
#define OUT1_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_12
#define OUT2_GPIO_Port GPIOA
#define OUT3_Pin GPIO_PIN_15
#define OUT3_GPIO_Port GPIOA
#define OUT4_Pin GPIO_PIN_3
#define OUT4_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_6
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_7
#define LCD_SDA_GPIO_Port GPIOB
#define button_mode_Pin GPIO_PIN_8
#define button_mode_GPIO_Port GPIOB
#define button_mode_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
