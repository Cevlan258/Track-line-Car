#include "ax_kinematics.h"
#include "ax_encoder.h"
#include "ax_motor.h"
#include "ax_robot.h"
#include "ax_servo.h"
#include "ax_speed.h"
#include "app_config.h"

static int16_t abs_i16(int16_t value)
{
  return (value < 0) ? (int16_t)(0 - value) : value;
}

static float clamp_f32(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static void apply_inner_wheel_slowdown(void)
{
  const float turn_ratio = clamp_f32((float)abs_i16(R_Vel.TG_IW) / (float)APP_MAIX_MAX_YAW,
                                     0.0f,
                                     1.0f);
  float inner_scale;

  if ((R_Vel.TG_IW == 0) || (R_Vel.TG_IX == 0))
  {
    return;
  }

  inner_scale = 1.0f - (turn_ratio * ((float)APP_TURN_INNER_SLOWDOWN_PCT / 100.0f));
  inner_scale = clamp_f32(inner_scale,
                          (float)APP_TURN_INNER_MIN_SCALE_PCT / 100.0f,
                          1.0f);

  if (R_Vel.TG_IW > 0)
  {
#if APP_TURN_POSITIVE_YAW_SLOWS_MOTOR_A
    R_Wheel_A.TG *= inner_scale;
#else
    R_Wheel_B.TG *= inner_scale;
#endif
  }
  else
  {
#if APP_TURN_POSITIVE_YAW_SLOWS_MOTOR_A
    R_Wheel_B.TG *= inner_scale;
#else
    R_Wheel_A.TG *= inner_scale;
#endif
  }
}

void AX_ROBOT_Kinematics(void)
{
  if (ax_robot_move_enable == 0U)
  {
    R_Vel.TG_IX = 0;
    R_Vel.TG_IY = 0;
    R_Vel.TG_IW = 0;
    R_Vel.RT_IX = 0;
    R_Vel.RT_IY = 0;
    R_Vel.RT_IW = 0;
    R_Wheel_A.RT = 0.0f;
    R_Wheel_B.RT = 0.0f;
    R_Wheel_A.TG = 0.0f;
    R_Wheel_B.TG = 0.0f;
    R_Wheel_A.PWM = 0;
    R_Wheel_B.PWM = 0;
    AX_ENCODER_A_SetCounter(0);
    AX_ENCODER_B_SetCounter(0);
    AX_SPEED_Reset();
    AX_MOTOR_A_SetSpeed(0);
    AX_MOTOR_B_SetSpeed(0);
    AX_SERVO_Center();
    return;
  }

  R_Wheel_A.RT = (float)((int16_t)AX_ENCODER_A_GetCounter() * TWD_WHEEL_SCALE);
  AX_ENCODER_A_SetCounter(0);
  R_Wheel_B.RT = (float)-((int16_t)AX_ENCODER_B_GetCounter() * TWD_WHEEL_SCALE);
  AX_ENCODER_B_SetCounter(0);

  R_Vel.RT_IX = (int16_t)(((R_Wheel_A.RT + R_Wheel_B.RT) / 2.0f) * 1000.0f);
  R_Vel.RT_IY = 0;
  R_Vel.RT_IW = 0;
  ax_robot_distance_mm += (int32_t)(((R_Wheel_A.RT + R_Wheel_B.RT) / 2.0f) * 20.0f);

  if (R_Vel.TG_IX > R_VX_LIMIT) R_Vel.TG_IX = R_VX_LIMIT;
  if (R_Vel.TG_IX < -R_VX_LIMIT) R_Vel.TG_IX = -R_VX_LIMIT;
  if (R_Vel.TG_IY > R_VY_LIMIT) R_Vel.TG_IY = R_VY_LIMIT;
  if (R_Vel.TG_IY < -R_VY_LIMIT) R_Vel.TG_IY = -R_VY_LIMIT;
  if (R_Vel.TG_IW > R_VW_LIMIT) R_Vel.TG_IW = R_VW_LIMIT;
  if (R_Vel.TG_IW < -R_VW_LIMIT) R_Vel.TG_IW = -R_VW_LIMIT;

  R_Vel.TG_FX = R_Vel.TG_IX / 1000.0f;
  R_Vel.TG_FY = 0.0f;
  R_Vel.TG_FW = R_Vel.TG_IW / 1000.0f;

  R_Wheel_A.TG = R_Vel.TG_FX;
  R_Wheel_B.TG = R_Vel.TG_FX;
  apply_inner_wheel_slowdown();

  R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, (float)R_Wheel_A.RT);
  R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, (float)R_Wheel_B.RT);

  AX_SERVO_SetSteering(R_Vel.TG_IW);
  AX_MOTOR_A_SetSpeed(R_Wheel_A.PWM);
  AX_MOTOR_B_SetSpeed(R_Wheel_B.PWM);
}

void AX_ROBOT_Stop(void)
{
  AX_MOTOR_A_SetSpeed(0);
  AX_MOTOR_B_SetSpeed(0);
  AX_SERVO_Center();
}

void AX_ROBOT_ResetDistance(void)
{
  ax_robot_distance_mm = 0;
}

int32_t AX_ROBOT_GetDistanceMm(void)
{
  return ax_robot_distance_mm;
}
