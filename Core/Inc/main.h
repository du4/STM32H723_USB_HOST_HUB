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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


#define I2C_ADDRESS        6
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void packPressurePacket(uint32_t tick);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define uSecondTimPrescaller 274
#define UserBtn_Pin GPIO_PIN_13
#define UserBtn_GPIO_Port GPIOC
#define ULPI_RES_Pin GPIO_PIN_2
#define ULPI_RES_GPIO_Port GPIOB
#define RedLed_Pin GPIO_PIN_14
#define RedLed_GPIO_Port GPIOB
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define SYNC1_Pin GPIO_PIN_14
#define SYNC1_GPIO_Port GPIOD
#define SYNC2_Pin GPIO_PIN_15
#define SYNC2_GPIO_Port GPIOD
#define CUT_EVENT_Pin GPIO_PIN_5
#define CUT_EVENT_GPIO_Port GPIOB
#define YellowLed_Pin GPIO_PIN_1
#define YellowLed_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
