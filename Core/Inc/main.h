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
#define BEEP_Pin GPIO_PIN_13
#define BEEP_GPIO_Port GPIOC
#define LORA_M0_Pin GPIO_PIN_0
#define LORA_M0_GPIO_Port GPIOC
#define LORA_M1_Pin GPIO_PIN_1
#define LORA_M1_GPIO_Port GPIOC
#define LORA_AUX_Pin GPIO_PIN_2
#define LORA_AUX_GPIO_Port GPIOC
#define OLED_CS_Pin GPIO_PIN_3
#define OLED_CS_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_4
#define OLED_DC_GPIO_Port GPIOA
#define OLED_RES_Pin GPIO_PIN_6
#define OLED_RES_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_4
#define LED_RED_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOB
#define CCD_SI_Pin GPIO_PIN_10
#define CCD_SI_GPIO_Port GPIOB
#define CCD_CLK_Pin GPIO_PIN_11
#define CCD_CLK_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOA
#define SERVO_PWM_Pin GPIO_PIN_7
#define SERVO_PWM_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
