/* USER CODE BEGIN Header */
/**
  * 中文说明：本段为工程生成代码说明。
  */
/* USER CODE END Header */

/* 防止重复包含 --------------------------------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含文件 ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 私有包含文件 --------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* 导出类型 ------------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* 导出常量 ------------------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* 导出宏 --------------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* 导出函数声明 --------------------------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* 私有定义 ------------------------------------------------------------------*/
#define BEEP_Pin GPIO_PIN_13
#define BEEP_GPIO_Port GPIOC
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
