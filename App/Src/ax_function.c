#include "ax_function.h"
#include "ax_ccd.h"
#include "ax_robot.h"

void AX_FUN_Ls1(void)
{
  static float bias_last;
  float move_w;
  const float bias = (float)AX_CCD_GetOffset();

  R_Vel.TG_IX = ax_ccd_velocity;
  ax_ccd_offset = (int16_t)bias;

  move_w = -ax_ccd_kp * bias * 0.1f - ax_ccd_kd * (bias - bias_last) * 0.1f;
  R_Vel.TG_IW = (int16_t)(0.01f * move_w * ax_ccd_velocity);

  bias_last = bias;
}
