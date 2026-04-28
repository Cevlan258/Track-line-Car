#ifndef AX_SERVO_H
#define AX_SERVO_H

#include "main.h"

#define AX_SERVO_CENTER_US 1500
#define AX_SERVO_MIN_US 1000
#define AX_SERVO_MAX_US 2000
#define AX_SERVO_RANGE_US 400
#define AX_SERVO_STEER_MAX 1000

void AX_SERVO_Init(void);
void AX_SERVO_SetPulseUs(uint16_t pulse_us);
void AX_SERVO_SetSteering(int16_t steering);
void AX_SERVO_Center(void);

#endif
