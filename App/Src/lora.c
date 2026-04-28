#include "lora.h"
#include "app_config.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define LORA_BUSY_POLL_MS 2U
#define LORA_READY_TIMEOUT_MS 200U
#define LORA_TX_DONE_TIMEOUT_MS 500U

static HAL_StatusTypeDef lora_wait_ready(uint32_t timeout_ms)
{
  const uint32_t start = HAL_GetTick();

  while (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    HAL_Delay(LORA_BUSY_POLL_MS);
  }

  return HAL_OK;
}

void LoRa_Init(void)
{
  HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
  (void)lora_wait_ready(LORA_READY_TIMEOUT_MS);
}

HAL_StatusTypeDef LoRa_SendCheckpoint(uint8_t checkpoint, uint32_t elapsed_ms)
{
  char msg[96];
  HAL_StatusTypeDef ret;
  const uint32_t elapsed_s = elapsed_ms / 1000U;
  const uint32_t minutes = elapsed_s / 60U;
  const uint32_t seconds = elapsed_s % 60U;
  const int len = snprintf(msg, sizeof(msg),
                           "TEAM=%s;NAME=%s;TIME=%s;POINT=2.%u;ELAPSED=%02lu:%02lu\r\n",
                           APP_TEAM_ID,
                           APP_TEAM_NAME,
                           APP_CURRENT_TIME,
                           checkpoint,
                           minutes,
                           seconds);

  if (len <= 0)
  {
    return HAL_ERROR;
  }

  ret = lora_wait_ready(LORA_READY_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
  if (ret != HAL_OK)
  {
    return ret;
  }

  return lora_wait_ready(LORA_TX_DONE_TIMEOUT_MS);
}
