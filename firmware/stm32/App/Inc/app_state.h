#ifndef APP_STATE_H
#define APP_STATE_H

#include "main.h"
#include "cmsis_os.h"

typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_RUN,
  APP_STATE_LORA,
  APP_STATE_FINISH,
  APP_STATE_FAULT
} AppStateId;

void AppState_Init(void);
void AppState_Task(void *argument);
uint8_t AppState_IsLineControlEnabled(void);
AppStateId AppState_Get(void);

#endif
