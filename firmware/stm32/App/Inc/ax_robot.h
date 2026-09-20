#ifndef AX_ROBOT_H
#define AX_ROBOT_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
  double RT;
  float TG;
  int16_t PWM;
} ROBOT_Wheel;

typedef struct
{
  int16_t RT_IX;
  int16_t RT_IY;
  int16_t RT_IW;
  int16_t TG_IX;
  int16_t TG_IY;
  int16_t TG_IW;
  float RT_FX;
  float RT_FY;
  float RT_FW;
  float TG_FX;
  float TG_FY;
  float TG_FW;
} ROBOT_Velocity;

#define PI 3.1416f
#define PID_RATE 50

#define VBAT_40P 1065
#define VBAT_20P 1012
#define VBAT_10P 984

#define TWD_WHEEL_DIAMETER 0.065f
#define TWD_WHEEL_BASE 0.165f
#define TWD_WHEEL_RESOLUTION 1560.0f
#define TWD_WHEEL_SCALE (PI * TWD_WHEEL_DIAMETER * PID_RATE / TWD_WHEEL_RESOLUTION)

#define R_VX_LIMIT 1500
#define R_VY_LIMIT 1200
#define R_VW_LIMIT 6280

#define CTL_FN1 0x03

extern ROBOT_Velocity R_Vel;
extern ROBOT_Wheel R_Wheel_A;
extern ROBOT_Wheel R_Wheel_B;
extern uint16_t R_Bat_Vol;
extern uint8_t ax_robot_move_enable;
extern uint8_t ax_control_mode;
extern int16_t ax_motor_kp;
extern int16_t ax_motor_kd;
extern int16_t ax_ccd_offset;
extern int16_t ax_ccd_velocity;
extern int16_t ax_ccd_kp;
extern int16_t ax_ccd_kd;
extern int32_t ax_robot_distance_mm;

void Robot_Task(void *parameter);
void AX_ROBOT_ResetDistance(void);
int32_t AX_ROBOT_GetDistanceMm(void);

#endif
