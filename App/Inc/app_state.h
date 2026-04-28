#ifndef APP_STATE_H
#define APP_STATE_H

#include "main.h"
#include "cmsis_os.h"

typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_LINE_TASK,
  APP_STATE_LORA_2_1,
  APP_STATE_LORA_2_2,
  APP_STATE_RADAR_PRE_SCAN,
  APP_STATE_AVOID_LEFT,
  APP_STATE_AVOID_RIGHT,
  APP_STATE_FINISH
} AppStateId;

void AppState_Init(void);
void AppState_Task(void *argument);
uint8_t AppState_IsLineControlEnabled(void);
AppStateId AppState_Get(void);

#endif
