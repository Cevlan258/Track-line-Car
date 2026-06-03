#include "maix_link.h"
#include "app_config.h"
#include "usart.h"
#include <string.h>

static uint8_t rx_byte;
static uint8_t rx_frame[MAIX_LINK_COMMAND_FRAME_LEN];
static uint8_t rx_index;
static volatile uint8_t command_valid;
static volatile uint32_t last_rx_ms;
static volatile MaixLinkCommand current_command;
static uint32_t last_telemetry_ms;
static uint8_t telemetry_seq;
static uint8_t lora_status = MAIX_LINK_LORA_IDLE;

static void maix_link_publish_command(const MaixLinkCommand *command)
{
  if (command == NULL)
  {
    return;
  }

  current_command = *command;
  command_valid = 1U;
  last_rx_ms = HAL_GetTick();
}

static void maix_link_feed(uint8_t byte)
{
  MaixLinkCommand parsed;

  if (rx_index == 0U)
  {
    if (byte != MAIX_LINK_HEAD_0)
    {
      return;
    }
  }
  else if (rx_index == 1U)
  {
    if (byte != MAIX_LINK_HEAD_1)
    {
      rx_index = (byte == MAIX_LINK_HEAD_0) ? 1U : 0U;
      rx_frame[0] = MAIX_LINK_HEAD_0;
      return;
    }
  }
  else if ((rx_index == 2U) && (byte != MAIX_LINK_COMMAND_BODY_LEN))
  {
    rx_index = 0U;
    return;
  }

  rx_frame[rx_index++] = byte;

  if (rx_index >= MAIX_LINK_COMMAND_FRAME_LEN)
  {
    if (MaixLink_ParseCommandFrame(rx_frame, MAIX_LINK_COMMAND_FRAME_LEN, &parsed) == MAIX_LINK_PARSE_OK)
    {
      maix_link_publish_command(&parsed);
    }
    rx_index = 0U;
  }
}

void MaixLink_Init(void)
{
  MaixLinkCommand initial;

  memset(&initial, 0, sizeof(initial));
  initial.mode = MAIX_LINK_MODE_IDLE;
  current_command = initial;
  command_valid = 0U;
  last_rx_ms = 0U;
  rx_index = 0U;
  last_telemetry_ms = 0U;
  telemetry_seq = 0U;
  lora_status = MAIX_LINK_LORA_IDLE;

  (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}

void MaixLink_TaskPoll(const RadarSample *sample,
                       int32_t distance_mm,
                       uint16_t battery_x100,
                       uint8_t battery_percent,
                       uint8_t started,
                       uint8_t moving,
                       uint8_t fault)
{
  MaixLinkTelemetry telemetry;
  uint8_t frame[MAIX_LINK_TELEMETRY_FRAME_LEN];
  uint8_t i;
  uint32_t now = HAL_GetTick();

  if ((now - last_telemetry_ms) < APP_MAIX_TELEMETRY_PERIOD_MS)
  {
    return;
  }
  last_telemetry_ms = now;

  memset(&telemetry, 0, sizeof(telemetry));
  telemetry.seq = telemetry_seq++;
  telemetry.status_flags = 0U;
  if (started != 0U)
  {
    telemetry.status_flags |= MAIX_LINK_STATUS_FLAG_STARTED;
  }
  if (moving != 0U)
  {
    telemetry.status_flags |= MAIX_LINK_STATUS_FLAG_MOVING;
  }
  if (fault != 0U)
  {
    telemetry.status_flags |= MAIX_LINK_STATUS_FLAG_FAULT;
  }
  telemetry.lora_status = lora_status;
  telemetry.battery_percent = battery_percent;
  telemetry.battery_x100 = battery_x100;
  telemetry.distance_mm = distance_mm;

  if (sample != NULL)
  {
    telemetry.radar_count = (sample->target_count > MAIX_LINK_MAX_TARGETS) ? MAIX_LINK_MAX_TARGETS : sample->target_count;
    for (i = 0U; i < MAIX_LINK_MAX_TARGETS; i++)
    {
      telemetry.targets[i].valid = sample->targets[i].valid;
      telemetry.targets[i].x_mm = sample->targets[i].x_mm;
      telemetry.targets[i].y_mm = sample->targets[i].y_mm;
      telemetry.targets[i].speed_cm_s = sample->targets[i].speed_cm_s;
      telemetry.targets[i].distance_mm = sample->targets[i].distance_mm;
    }
  }

  if (MaixLink_EncodeTelemetryFrame(&telemetry, frame, sizeof(frame)) == MAIX_LINK_TELEMETRY_FRAME_LEN)
  {
    (void)HAL_UART_Transmit(&huart3, frame, sizeof(frame), 10U);
  }
}

uint8_t MaixLink_GetCommand(MaixLinkCommand *command, uint32_t timeout_ms)
{
  uint8_t fresh;

  if (command == NULL)
  {
    return 0U;
  }

  __disable_irq();
  *command = current_command;
  fresh = ((command_valid != 0U) && ((HAL_GetTick() - last_rx_ms) <= timeout_ms)) ? 1U : 0U;
  __enable_irq();

  return fresh;
}

uint8_t MaixLink_IsFresh(uint32_t timeout_ms)
{
  return ((command_valid != 0U) && ((HAL_GetTick() - last_rx_ms) <= timeout_ms)) ? 1U : 0U;
}

void MaixLink_SetLoraStatus(uint8_t status)
{
  lora_status = status;
}

void MaixLink_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  maix_link_feed(rx_byte);
  (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}

void MaixLink_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  /* 串口错误后丢弃半帧并重新打开接收，避免上电噪声导致链路永久断开。 */
  rx_index = 0U;
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  __HAL_UART_CLEAR_OREFLAG(huart);
  (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}
