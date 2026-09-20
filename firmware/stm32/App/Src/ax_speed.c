#include "ax_speed.h"
#include "ax_robot.h"

static int16_t motor_pwm_out_a;
static int16_t motor_pwm_out_b;
static float bias_last_a;
static float bias_last_b;

int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current)
{
  const float bias = spd_target - spd_current;

  motor_pwm_out_a += (int16_t)(ax_motor_kp * bias + ax_motor_kd * (bias - bias_last_a));
  bias_last_a = bias;

  if (motor_pwm_out_a > 4200)
  {
    motor_pwm_out_a = 4200;
  }
  if (motor_pwm_out_a < -4200)
  {
    motor_pwm_out_a = -4200;
  }

  return motor_pwm_out_a;
}

int16_t AX_SPEED_PidCtlB(float spd_target, float spd_current)
{
  const float bias = spd_target - spd_current;

  motor_pwm_out_b += (int16_t)(ax_motor_kp * bias + ax_motor_kd * (bias - bias_last_b));
  bias_last_b = bias;

  if (motor_pwm_out_b > 4200)
  {
    motor_pwm_out_b = 4200;
  }
  if (motor_pwm_out_b < -4200)
  {
    motor_pwm_out_b = -4200;
  }

  return motor_pwm_out_b;
}

void AX_SPEED_Reset(void)
{
  motor_pwm_out_a = 0;
  motor_pwm_out_b = 0;
  bias_last_a = 0.0f;
  bias_last_b = 0.0f;
}
