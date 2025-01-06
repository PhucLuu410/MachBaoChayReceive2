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
#define TINHIEUNHIETDO_Pin GPIO_PIN_4
#define TINHIEUNHIETDO_GPIO_Port GPIOA
#define TINHIEUKHOI_Pin GPIO_PIN_5
#define TINHIEUKHOI_GPIO_Port GPIOA
#define TINHIEUKHIGAS_Pin GPIO_PIN_6
#define TINHIEUKHIGAS_GPIO_Port GPIOA
#define TINHIEULUA_Pin GPIO_PIN_7
#define TINHIEULUA_GPIO_Port GPIOA
#define TINHIEUGUI_Pin GPIO_PIN_0
#define TINHIEUGUI_GPIO_Port GPIOB
#define COICHIP_Pin GPIO_PIN_12
#define COICHIP_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_13
#define RELAY1_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_14
#define RELAY2_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_15
#define OUT1_GPIO_Port GPIOB
#define TINHIEUKHIGAS2_Pin GPIO_PIN_3
#define TINHIEUKHIGAS2_GPIO_Port GPIOB
#define TINHIEUKHOI2_Pin GPIO_PIN_4
#define TINHIEUKHOI2_GPIO_Port GPIOB
#define TINHIEUNHIETDO2_Pin GPIO_PIN_5
#define TINHIEUNHIETDO2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
