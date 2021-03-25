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
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOA
#define LEDR_Pin GPIO_PIN_6
#define LEDR_GPIO_Port GPIOA
#define LEDG_Pin GPIO_PIN_7
#define LEDG_GPIO_Port GPIOA
#define LEDB_Pin GPIO_PIN_0
#define LEDB_GPIO_Port GPIOB
#define DONE_Pin GPIO_PIN_8
#define DONE_GPIO_Port GPIOE
#define KICKMODE_Pin GPIO_PIN_9
#define KICKMODE_GPIO_Port GPIOE
#define KICK_Pin GPIO_PIN_10
#define KICK_GPIO_Port GPIOE
#define BOOST_Pin GPIO_PIN_11
#define BOOST_GPIO_Port GPIOE
#define DIPSW_1_Pin GPIO_PIN_14
#define DIPSW_1_GPIO_Port GPIOB
#define DIPSW_2_Pin GPIO_PIN_15
#define DIPSW_2_GPIO_Port GPIOB
#define DIPSW_3_Pin GPIO_PIN_10
#define DIPSW_3_GPIO_Port GPIOD
#define DIPSW_4_Pin GPIO_PIN_11
#define DIPSW_4_GPIO_Port GPIOD
#define EMO_Pin GPIO_PIN_12
#define EMO_GPIO_Port GPIOD
#define ALIVE_Pin GPIO_PIN_13
#define ALIVE_GPIO_Port GPIOD
#define POWER_Pin GPIO_PIN_14
#define POWER_GPIO_Port GPIOD
#define USER_SW0_Pin GPIO_PIN_0
#define USER_SW0_GPIO_Port GPIOD
#define USER_SW1_Pin GPIO_PIN_1
#define USER_SW1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
