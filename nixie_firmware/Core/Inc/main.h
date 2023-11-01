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
#include <stdbool.h>
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
#define IR_REC_Pin GPIO_PIN_3
#define IR_REC_GPIO_Port GPIOA
#define IR_REC_EXTI_IRQn EXTI3_IRQn
#define DIGIT_1_Pin GPIO_PIN_12
#define DIGIT_1_GPIO_Port GPIOB
#define DIGIT_2_Pin GPIO_PIN_13
#define DIGIT_2_GPIO_Port GPIOB
#define DIGIT_3_Pin GPIO_PIN_14
#define DIGIT_3_GPIO_Port GPIOB
#define DIGIT_4_Pin GPIO_PIN_15
#define DIGIT_4_GPIO_Port GPIOB
#define SIGN_0_Pin GPIO_PIN_6
#define SIGN_0_GPIO_Port GPIOC
#define SIGN_9_Pin GPIO_PIN_7
#define SIGN_9_GPIO_Port GPIOC
#define SIGN_8_Pin GPIO_PIN_8
#define SIGN_8_GPIO_Port GPIOC
#define SIGN_7_Pin GPIO_PIN_9
#define SIGN_7_GPIO_Port GPIOC
#define SIGN_6_Pin GPIO_PIN_8
#define SIGN_6_GPIO_Port GPIOA
#define SIGN_5_Pin GPIO_PIN_9
#define SIGN_5_GPIO_Port GPIOA
#define SIGN_4_Pin GPIO_PIN_10
#define SIGN_4_GPIO_Port GPIOA
#define SIGN_3_Pin GPIO_PIN_11
#define SIGN_3_GPIO_Port GPIOA
#define SIGN_2_Pin GPIO_PIN_12
#define SIGN_2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
