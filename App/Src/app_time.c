#include "app_time.h"
#include "ax_delay.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define APP_TIME_SCL_Pin GPIO_PIN_12
#define APP_TIME_SCL_GPIO_Port GPIOB
#define APP_TIME_SDA_Pin GPIO_PIN_13
#define APP_TIME_SDA_GPIO_Port GPIOB

#define DS3231_ADDR 0x68U
#define DS3231_REG_SECONDS 0x00U

#define APP_TIME_CMD_MAX 24U

typedef struct
{
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} AppTimeValue;

static uint8_t app_time_valid;
static uint8_t uart_rx_byte;
static volatile uint8_t cmd_ready;
static volatile uint8_t cmd_len;
static char cmd_buf[APP_TIME_CMD_MAX];
static char rx_buf[APP_TIME_CMD_MAX];
static uint8_t rx_len;

static void i2c_delay(void)
{
  AX_Delayus(5);
}

static void scl_high(void)
{
  HAL_GPIO_WritePin(APP_TIME_SCL_GPIO_Port, APP_TIME_SCL_Pin, GPIO_PIN_SET);
}

static void scl_low(void)
{
  HAL_GPIO_WritePin(APP_TIME_SCL_GPIO_Port, APP_TIME_SCL_Pin, GPIO_PIN_RESET);
}

static void sda_high(void)
{
  HAL_GPIO_WritePin(APP_TIME_SDA_GPIO_Port, APP_TIME_SDA_Pin, GPIO_PIN_SET);
}

static void sda_low(void)
{
  HAL_GPIO_WritePin(APP_TIME_SDA_GPIO_Port, APP_TIME_SDA_Pin, GPIO_PIN_RESET);
}

static GPIO_PinState sda_read(void)
{
  return HAL_GPIO_ReadPin(APP_TIME_SDA_GPIO_Port, APP_TIME_SDA_Pin);
}

static void i2c_start(void)
{
  sda_high();
  scl_high();
  i2c_delay();
  sda_low();
  i2c_delay();
  scl_low();
}

static void i2c_stop(void)
{
  sda_low();
  i2c_delay();
  scl_high();
  i2c_delay();
  sda_high();
  i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t data)
{
  uint8_t i;
  uint8_t ack;

  for (i = 0; i < 8U; i++)
  {
    if ((data & 0x80U) != 0U)
    {
      sda_high();
    }
    else
    {
      sda_low();
    }
    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
    data <<= 1;
  }

  sda_high();
  i2c_delay();
  scl_high();
  i2c_delay();
  ack = (sda_read() == GPIO_PIN_RESET) ? 1U : 0U;
  scl_low();
  return ack;
}

static uint8_t i2c_read_byte(uint8_t ack)
{
  uint8_t i;
  uint8_t data = 0;

  sda_high();
  for (i = 0; i < 8U; i++)
  {
    data <<= 1;
    scl_high();
    i2c_delay();
    if (sda_read() == GPIO_PIN_SET)
    {
      data |= 1U;
    }
    scl_low();
    i2c_delay();
  }

  if (ack != 0U)
  {
    sda_low();
  }
  else
  {
    sda_high();
  }
  i2c_delay();
  scl_high();
  i2c_delay();
  scl_low();
  sda_high();
  return data;
}

static uint8_t bcd_to_bin(uint8_t value)
{
  return (uint8_t)(((value >> 4) * 10U) + (value & 0x0FU));
}

static uint8_t bin_to_bcd(uint8_t value)
{
  return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

static HAL_StatusTypeDef ds3231_read_time(AppTimeValue *time)
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour_reg;
  uint8_t hour;

  if (time == NULL)
  {
    return HAL_ERROR;
  }

  i2c_start();
  if (i2c_write_byte((uint8_t)(DS3231_ADDR << 1)) == 0U)
  {
    i2c_stop();
    return HAL_ERROR;
  }
  if (i2c_write_byte(DS3231_REG_SECONDS) == 0U)
  {
    i2c_stop();
    return HAL_ERROR;
  }

  i2c_start();
  if (i2c_write_byte((uint8_t)((DS3231_ADDR << 1) | 1U)) == 0U)
  {
    i2c_stop();
    return HAL_ERROR;
  }

  sec = i2c_read_byte(1U);
  min = i2c_read_byte(1U);
  hour_reg = i2c_read_byte(0U);
  i2c_stop();

  if ((hour_reg & 0x40U) != 0U)
  {
    hour = bcd_to_bin((uint8_t)(hour_reg & 0x1FU));
    if ((hour_reg & 0x20U) != 0U)
    {
      if (hour < 12U)
      {
        hour = (uint8_t)(hour + 12U);
      }
    }
    else if (hour == 12U)
    {
      hour = 0;
    }
  }
  else
  {
    hour = bcd_to_bin((uint8_t)(hour_reg & 0x3FU));
  }

  time->second = bcd_to_bin((uint8_t)(sec & 0x7FU));
  time->minute = bcd_to_bin((uint8_t)(min & 0x7FU));
  time->hour = hour;

  if ((time->hour > 23U) || (time->minute > 59U) || (time->second > 59U))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef ds3231_write_time(uint8_t hour, uint8_t minute, uint8_t second)
{
  if ((hour > 23U) || (minute > 59U) || (second > 59U))
  {
    return HAL_ERROR;
  }

  i2c_start();
  if (i2c_write_byte((uint8_t)(DS3231_ADDR << 1)) == 0U)
  {
    i2c_stop();
    return HAL_ERROR;
  }
  if (i2c_write_byte(DS3231_REG_SECONDS) == 0U)
  {
    i2c_stop();
    return HAL_ERROR;
  }
  if (i2c_write_byte(bin_to_bcd(second)) == 0U)
  {
    i2c_stop();
    return HAL_ERROR;
  }
  if (i2c_write_byte(bin_to_bcd(minute)) == 0U)
  {
    i2c_stop();
    return HAL_ERROR;
  }
  if (i2c_write_byte(bin_to_bcd(hour)) == 0U)
  {
    i2c_stop();
    return HAL_ERROR;
  }
  i2c_stop();

  return HAL_OK;
}

static void app_time_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, APP_TIME_SCL_Pin | APP_TIME_SDA_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = APP_TIME_SCL_Pin | APP_TIME_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static uint8_t parse_two_digits(const char *s, uint8_t *value)
{
  if ((s[0] < '0') || (s[0] > '9') || (s[1] < '0') || (s[1] > '9'))
  {
    return 0;
  }

  *value = (uint8_t)(((uint8_t)(s[0] - '0') * 10U) + (uint8_t)(s[1] - '0'));
  return 1;
}

static uint8_t parse_settime(const char *cmd, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
  if (strncmp(cmd, "SETTIME=", 8U) != 0)
  {
    return 0;
  }
  if ((cmd[10] != ':') || (cmd[13] != ':') || (cmd[16] != '\0'))
  {
    return 0;
  }
  if ((parse_two_digits(&cmd[8], hour) == 0U) ||
      (parse_two_digits(&cmd[11], minute) == 0U) ||
      (parse_two_digits(&cmd[14], second) == 0U))
  {
    return 0;
  }

  return ((*hour <= 23U) && (*minute <= 59U) && (*second <= 59U)) ? 1U : 0U;
}

static void uart_reply(const char *msg)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
}

void AppTime_Init(void)
{
  AppTimeValue time;

  app_time_gpio_init();
  app_time_valid = (ds3231_read_time(&time) == HAL_OK) ? 1U : 0U;
  rx_len = 0;
  cmd_ready = 0;
  cmd_len = 0;
  (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

void AppTime_TaskPoll(void)
{
  char local_cmd[APP_TIME_CMD_MAX];
  uint8_t local_len;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  if (cmd_ready == 0U)
  {
    return;
  }

  __disable_irq();
  local_len = cmd_len;
  if (local_len >= APP_TIME_CMD_MAX)
  {
    local_len = APP_TIME_CMD_MAX - 1U;
  }
  memcpy(local_cmd, cmd_buf, local_len);
  local_cmd[local_len] = '\0';
  cmd_ready = 0;
  __enable_irq();

  if ((parse_settime(local_cmd, &hour, &minute, &second) != 0U) &&
      (AppTime_SetBeijingTime(hour, minute, second) == HAL_OK))
  {
    uart_reply("TIME OK\r\n");
  }
  else
  {
    uart_reply("TIME ERR\r\n");
  }
}

void AppTime_GetBeijingTime(char *buf, uint32_t len)
{
  AppTimeValue time;

  if ((buf == NULL) || (len == 0U))
  {
    return;
  }

  if (ds3231_read_time(&time) == HAL_OK)
  {
    app_time_valid = 1U;
    (void)snprintf(buf, len, "%02u:%02u:%02u", time.hour, time.minute, time.second);
  }
  else
  {
    app_time_valid = 0U;
    (void)snprintf(buf, len, "--:--:--");
  }
}

HAL_StatusTypeDef AppTime_SetBeijingTime(uint8_t hour, uint8_t minute, uint8_t second)
{
  const HAL_StatusTypeDef ret = ds3231_write_time(hour, minute, second);
  app_time_valid = (ret == HAL_OK) ? 1U : 0U;
  return ret;
}

uint8_t AppTime_IsValid(void)
{
  return app_time_valid;
}

void AppTime_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  if ((uart_rx_byte == '\r') || (uart_rx_byte == '\n'))
  {
    if ((rx_len > 0U) && (cmd_ready == 0U))
    {
      memcpy(cmd_buf, rx_buf, rx_len);
      cmd_len = rx_len;
      cmd_ready = 1U;
    }
    rx_len = 0;
  }
  else if (rx_len < (APP_TIME_CMD_MAX - 1U))
  {
    rx_buf[rx_len++] = (char)uart_rx_byte;
  }
  else
  {
    rx_len = 0;
  }

  (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}
