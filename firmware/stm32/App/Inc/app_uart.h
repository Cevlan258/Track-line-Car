#ifndef APP_UART_H
#define APP_UART_H

#include "main.h"

void App_UartRxCpltCallback(UART_HandleTypeDef *huart);
void App_UartErrorCallback(UART_HandleTypeDef *huart);

#endif
