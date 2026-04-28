#include "ax_motor.h"
#include "tim.h"

#define MOTOR_PWM_MAX 3599

static int16_t motor_limit(int16_t speed)
{
  if (speed > MOTOR_PWM_MAX)
  {
    return MOTOR_PWM_MAX;
  }
  if (speed < -MOTOR_PWM_MAX)
  {
    return -MOTOR_PWM_MAX;
  }
  return speed;
}

void AX_MOTOR_Init(void)
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  AX_MOTOR_A_SetSpeed(0);
  AX_MOTOR_B_SetSpeed(0);
}

void AX_MOTOR_A_SetSpeed(int16_t speed)
{
  const int16_t temp = motor_limit(speed);

  if (temp == 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  }
  else if (temp > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_PWM_MAX - temp);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_PWM_MAX + temp);
  }
}

void AX_MOTOR_B_SetSpeed(int16_t speed)
{
  const int16_t temp = motor_limit(speed);

  if (temp == 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  }
  else if (temp > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_PWM_MAX - temp);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_PWM_MAX + temp);
  }
}
