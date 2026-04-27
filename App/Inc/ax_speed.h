#ifndef AX_SPEED_H
#define AX_SPEED_H

#include "main.h"

int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current);
int16_t AX_SPEED_PidCtlB(float spd_target, float spd_current);

#endif
