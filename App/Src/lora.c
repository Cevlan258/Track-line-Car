#include "lora.h"
#include "app_time.h"
#include "app_config.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define LORA_BUSY_POLL_MS 2U
#define LORA_READY_TIMEOUT_MS 200U
#define LORA_TX_DONE_TIMEOUT_MS 500U
#define LORA_MODE_SWITCH_TIMEOUT_MS 1000U
#define LORA_CONFIG_TIMEOUT_MS 300U
#define LORA_HMODE_CONFIG "AT+HMODE=0"
#define LORA_HMODE_UART_LORA "AT+HMODE=1"

static HAL_StatusTypeDef lora_wait_ready(uint32_t timeout_ms)
{
  const uint32_t start = HAL_GetTick();

  while (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    HAL_Delay(LORA_BUSY_POLL_MS);
  }

  return HAL_OK;
}

static HAL_StatusTypeDef lora_send_command(const char *cmd, uint32_t timeout_ms)
{
  HAL_StatusTypeDef ret = lora_wait_ready(LORA_READY_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = HAL_UART_Transmit(&huart2,
                          (uint8_t *)cmd,
                          (uint16_t)strlen(cmd),
                          100U);
  if (ret != HAL_OK)
  {
    return ret;
  }

  return lora_wait_ready(timeout_ms);
}

static HAL_StatusTypeDef lora_send_u32_command(const char *prefix, uint32_t value)
{
  char cmd[32];
  const int len = snprintf(cmd, sizeof(cmd), "%s%lu", prefix, (unsigned long)value);

  if ((len <= 0) || ((size_t)len >= sizeof(cmd)))
  {
    return HAL_ERROR;
  }

  return lora_send_command(cmd, LORA_CONFIG_TIMEOUT_MS);
}

static HAL_StatusTypeDef lora_apply_config(void)
{
  HAL_StatusTypeDef ret;

  ret = lora_send_command(LORA_HMODE_CONFIG, LORA_MODE_SWITCH_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = lora_send_u32_command("AT+RATE=", APP_LORA_AIR_RATE_2K4);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = lora_send_u32_command("AT+NETID=", APP_LORA_NETWORK_ID);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = lora_send_u32_command("AT+PACKET=", APP_LORA_PACKET_240_BYTES);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = lora_send_u32_command("AT+TRANS=", APP_LORA_TRANSPARENT_MODE);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = lora_send_u32_command("AT+ROUTER=", APP_LORA_RELAY_DISABLED);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = lora_send_u32_command("AT+KEY=", APP_LORA_KEY_DISABLED);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = lora_send_u32_command("AT+ADDR=", APP_LORA_LOCAL_ADDRESS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = lora_send_u32_command("AT+CHANNEL=", APP_LORA_CHANNEL);
  if (ret != HAL_OK)
  {
    return ret;
  }

  return lora_send_command(LORA_HMODE_UART_LORA, LORA_MODE_SWITCH_TIMEOUT_MS);
}

void LoRa_Init(void)
{
  (void)lora_apply_config();
}

HAL_StatusTypeDef LoRa_SendCheckpoint(uint8_t checkpoint, uint32_t elapsed_ms)
{
  char msg[96];
  char time_buf[9];
  HAL_StatusTypeDef ret;
  const uint32_t elapsed_s = elapsed_ms / 1000U;
  const uint32_t minutes = elapsed_s / 60U;
  const uint32_t seconds = elapsed_s % 60U;

  AppTime_GetBeijingTime(time_buf, sizeof(time_buf));

  const int len = snprintf(msg, sizeof(msg),
                           "TEAM=%s;NAME=%s;TIME=%s;POINT=2.%u;ELAPSED=%02lu:%02lu\r\n",
                           APP_TEAM_ID,
                           APP_TEAM_NAME,
                           time_buf,
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
