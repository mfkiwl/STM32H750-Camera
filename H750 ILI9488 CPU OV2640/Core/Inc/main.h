/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define SDMMC1_CD_Pin GPIO_PIN_13
#define SDMMC1_CD_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOC
#define CAMERA_RESET_Pin GPIO_PIN_11
#define CAMERA_RESET_GPIO_Port GPIOB
#define LCD_RESET_Pin GPIO_PIN_9
#define LCD_RESET_GPIO_Port GPIOA
#define SW_DOWN_Pin GPIO_PIN_6
#define SW_DOWN_GPIO_Port GPIOD
#define SW_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define SW_RIGHT_Pin GPIO_PIN_7
#define SW_RIGHT_GPIO_Port GPIOD
#define SW_RIGHT_EXTI_IRQn EXTI9_5_IRQn
#define SW_UP_Pin GPIO_PIN_3
#define SW_UP_GPIO_Port GPIOB
#define SW_UP_EXTI_IRQn EXTI3_IRQn
#define SW_LEFT_Pin GPIO_PIN_4
#define SW_LEFT_GPIO_Port GPIOB
#define SW_LEFT_EXTI_IRQn EXTI4_IRQn
#define SW_CENTER_Pin GPIO_PIN_5
#define SW_CENTER_GPIO_Port GPIOB
#define SW_CENTER_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
