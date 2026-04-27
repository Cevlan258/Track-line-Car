#include "ax_speed.h"
#include "ax_robot.h"

int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current)
{
  static int16_t motor_pwm_out;
  static float bias_last;
  const float bias = spd_target - spd_current;

  motor_pwm_out += (int16_t)(ax_motor_kp * bias + ax_motor_kd * (bias - bias_last));
  bias_last = bias;

  if (motor_pwm_out > 4200)
  {
    motor_pwm_out = 4200;
  }
  if (motor_pwm_out < -4200)
  {
    motor_pwm_out = -4200;
  }

  return motor_pwm_out;
}

int16_t AX_SPEED_PidCtlB(float spd_target, float spd_current)
{
  static int16_t motor_pwm_out;
  static float bias_last;
  const float bias = spd_target - spd_current;

  motor_pwm_out += (int16_t)(ax_motor_kp * bias + ax_motor_kd * (bias - bias_last));
  bias_last = bias;

  if (motor_pwm_out > 4200)
  {
    motor_pwm_out = 4200;
  }
  if (motor_pwm_out < -4200)
  {
    motor_pwm_out = -4200;
  }

  return motor_pwm_out;
}
