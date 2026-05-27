#include "app_uart.h"
#include "app_time.h"
#include "maix_link.h"
#include "radar.h"

void App_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    Radar_UartRxCpltCallback(huart);
  }
  else if (huart->Instance == USART1)
  {
    AppTime_UartRxCpltCallback(huart);
  }
  else if (huart->Instance == USART3)
  {
    MaixLink_UartRxCpltCallback(huart);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  App_UartRxCpltCallback(huart);
}
