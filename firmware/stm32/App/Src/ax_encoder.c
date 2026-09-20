#include "ax_encoder.h"
#include "tim.h"

void AX_ENCODER_A_Init(void)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

uint16_t AX_ENCODER_A_GetCounter(void)
{
  return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

void AX_ENCODER_A_SetCounter(uint16_t count)
{
  __HAL_TIM_SET_COUNTER(&htim2, count);
}

void AX_ENCODER_B_Init(void)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

uint16_t AX_ENCODER_B_GetCounter(void)
{
  return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void AX_ENCODER_B_SetCounter(uint16_t count)
{
  __HAL_TIM_SET_COUNTER(&htim3, count);
}
