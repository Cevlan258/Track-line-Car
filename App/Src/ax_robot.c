#include "ax_robot.h"
#include "ax_function.h"
#include "ax_kinematics.h"

ROBOT_Velocity R_Vel;
ROBOT_Wheel R_Wheel_A;
ROBOT_Wheel R_Wheel_B;

uint16_t R_Bat_Vol;

int16_t ax_motor_kp = 600;
int16_t ax_motor_kd = 800;

uint8_t ax_robot_move_enable = 0;
uint8_t ax_control_mode = CTL_FN1;

int16_t ax_ccd_offset;
int16_t ax_ccd_velocity = 500;
int16_t ax_ccd_kp = 80;
int16_t ax_ccd_kd = 50;

void Robot_Task(void *parameter)
{
  TickType_t previous_wake_time = xTaskGetTickCount();
  const TickType_t time_increment = pdMS_TO_TICKS(20);

  (void)parameter;

  for (;;)
  {
    vTaskDelayUntil(&previous_wake_time, time_increment);

    if (ax_control_mode == CTL_FN1)
    {
      AX_FUN_Ls1();
    }

    AX_ROBOT_Kinematics();
  }
}
