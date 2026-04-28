#include "ax_servo.h"
#include "tim.h"

static uint16_t servo_limit_pulse(uint16_t pulse_us)
{
  if (pulse_us < AX_SERVO_MIN_US)
  {
    return AX_SERVO_MIN_US;
  }
  if (pulse_us > AX_SERVO_MAX_US)
  {
    return AX_SERVO_MAX_US;
  }
  return pulse_us;
}

void AX_SERVO_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  AX_SERVO_Center();
}

void AX_SERVO_SetPulseUs(uint16_t pulse_us)
{
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, servo_limit_pulse(pulse_us));
}

void AX_SERVO_SetSteering(int16_t steering)
{
  int32_t pulse;

  if (steering > AX_SERVO_STEER_MAX)
  {
    steering = AX_SERVO_STEER_MAX;
  }
  if (steering < -AX_SERVO_STEER_MAX)
  {
    steering = -AX_SERVO_STEER_MAX;
  }

  pulse = AX_SERVO_CENTER_US + ((int32_t)steering * AX_SERVO_RANGE_US) / AX_SERVO_STEER_MAX;
  AX_SERVO_SetPulseUs((uint16_t)pulse);
}

void AX_SERVO_Center(void)
{
  AX_SERVO_SetPulseUs(AX_SERVO_CENTER_US);
}
