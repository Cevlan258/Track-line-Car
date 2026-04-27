#include "ax_kinematics.h"
#include "ax_encoder.h"
#include "ax_motor.h"
#include "ax_robot.h"
#include "ax_speed.h"

void AX_ROBOT_Kinematics(void)
{
  if (ax_robot_move_enable == 0U)
  {
    R_Vel.TG_IX = 0;
    R_Vel.TG_IY = 0;
    R_Vel.TG_IW = 0;
  }

  R_Wheel_A.RT = (float)((int16_t)AX_ENCODER_A_GetCounter() * TWD_WHEEL_SCALE);
  AX_ENCODER_A_SetCounter(0);
  R_Wheel_B.RT = (float)-((int16_t)AX_ENCODER_B_GetCounter() * TWD_WHEEL_SCALE);
  AX_ENCODER_B_SetCounter(0);

  R_Vel.RT_IX = (int16_t)(((R_Wheel_A.RT + R_Wheel_B.RT) / 2.0f) * 1000.0f);
  R_Vel.RT_IY = 0;
  R_Vel.RT_IW = (int16_t)(((-R_Wheel_A.RT + R_Wheel_B.RT) / TWD_WHEEL_BASE) * 1000.0f);

  if (R_Vel.TG_IX > R_VX_LIMIT) R_Vel.TG_IX = R_VX_LIMIT;
  if (R_Vel.TG_IX < -R_VX_LIMIT) R_Vel.TG_IX = -R_VX_LIMIT;
  if (R_Vel.TG_IY > R_VY_LIMIT) R_Vel.TG_IY = R_VY_LIMIT;
  if (R_Vel.TG_IY < -R_VY_LIMIT) R_Vel.TG_IY = -R_VY_LIMIT;
  if (R_Vel.TG_IW > R_VW_LIMIT) R_Vel.TG_IW = R_VW_LIMIT;
  if (R_Vel.TG_IW < -R_VW_LIMIT) R_Vel.TG_IW = -R_VW_LIMIT;

  R_Vel.TG_FX = R_Vel.TG_IX / 1000.0f;
  R_Vel.TG_FY = 0.0f;
  R_Vel.TG_FW = R_Vel.TG_IW / 1000.0f;

  R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FW * (TWD_WHEEL_BASE / 2.0f);
  R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FW * (TWD_WHEEL_BASE / 2.0f);

  R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, (float)R_Wheel_A.RT);
  R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, (float)R_Wheel_B.RT);

  AX_MOTOR_A_SetSpeed(-R_Wheel_A.PWM);
  AX_MOTOR_B_SetSpeed(-R_Wheel_B.PWM);
}

void AX_ROBOT_Stop(void)
{
  AX_MOTOR_A_SetSpeed(0);
  AX_MOTOR_B_SetSpeed(0);
}
