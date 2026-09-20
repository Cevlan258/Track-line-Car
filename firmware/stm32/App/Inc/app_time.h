#ifndef APP_TIME_H
#define APP_TIME_H

#include "main.h"

void AppTime_Init(void);
void AppTime_TaskPoll(void);
void AppTime_GetBeijingTime(char *buf, uint32_t len);
HAL_StatusTypeDef AppTime_SetBeijingTime(uint8_t hour, uint8_t minute, uint8_t second);
uint8_t AppTime_IsValid(void);
void AppTime_UartRxCpltCallback(UART_HandleTypeDef *huart);

#endif
