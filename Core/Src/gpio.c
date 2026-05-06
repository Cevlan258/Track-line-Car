/* USER CODE BEGIN Header */
/**
  * 中文说明：本段为工程生成代码说明。
  */
/* USER CODE END Header */

/* 包含文件 ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* 中文分区说明 ------------------------------------------------------------*/
/* 配置 GPIO                                                                 */
/* 中文分区说明 ------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * 中文说明：本段为工程生成代码说明。
  */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO 端口时钟使能 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 配置 GPIO 初始输出电平 */
  HAL_GPIO_WritePin(GPIOC, BEEP_Pin|OLED_CS_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /* 配置 GPIO 引脚输出电平 */
  HAL_GPIO_WritePin(GPIOA, OLED_DC_Pin|OLED_RES_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /* 配置 GPIO 引脚输出电平 */
  HAL_GPIO_WritePin(GPIOB, CCD_SI_Pin|CCD_CLK_Pin, GPIO_PIN_RESET);

  /* 配置 GPIO 引脚：BEEP_Pin OLED_CS_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin|OLED_CS_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 配置 GPIO 引脚：LORA_AUX_Pin */
  GPIO_InitStruct.Pin = LORA_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &GPIO_InitStruct);

  /* 配置 GPIO 引脚：OLED_DC_Pin、OLED_RES_Pin、LED_GREEN_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RES_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 配置 GPIO 引脚：KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /* 配置 GPIO 引脚：CCD_SI_Pin、CCD_CLK_Pin */
  GPIO_InitStruct.Pin = CCD_SI_Pin|CCD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
