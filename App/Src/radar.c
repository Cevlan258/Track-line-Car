#include "radar.h"
#include "usart.h"
#include <string.h>

#define LD2450_FRAME_LEN 30U
#define LD2450_HEADER_LEN 4U
#define LD2450_TARGET_LEN 8U

static uint8_t rx_byte;
static uint8_t frame[LD2450_FRAME_LEN];
static uint8_t frame_len;
static RadarSample current_sample;

static uint16_t le16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t ld2450_decode_signed(uint16_t raw)
{
  if ((raw & 0x8000U) != 0U)
  {
    return (int16_t)(raw - 0x8000U);
  }

  return (int16_t)(0 - (int32_t)raw);
}

static uint16_t isqrt32(uint32_t value)
{
  uint32_t bit = 1UL << 30;
  uint32_t result = 0;

  while (bit > value)
  {
    bit >>= 2;
  }

  while (bit != 0UL)
  {
    if (value >= (result + bit))
    {
      value -= result + bit;
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }
    bit >>= 2;
  }

  return (result > 0xFFFFUL) ? 0xFFFFU : (uint16_t)result;
}

static uint16_t target_distance_mm(int16_t x_mm, int16_t y_mm)
{
  const int32_t x = x_mm;
  const int32_t y = y_mm;
  const uint32_t xx = (uint32_t)(x * x);
  const uint32_t yy = (uint32_t)(y * y);

  return isqrt32(xx + yy);
}

static void radar_clear_sample(void)
{
  memset(&current_sample, 0, sizeof(current_sample));
}

static void radar_update_compat_fields(void)
{
  uint8_t i;
  uint16_t nearest = 0xFFFFU;
  uint8_t nearest_found = 0U;

  current_sample.valid = (current_sample.target_count > 0U) ? 1U : 0U;
  current_sample.target_state = current_sample.target_count;

  for (i = 0; i < RADAR_MAX_TARGETS; i++)
  {
    const RadarTarget *target = &current_sample.targets[i];
    if ((target->valid != 0U) && (target->distance_mm < nearest))
    {
      nearest = target->distance_mm;
      nearest_found = 1U;
    }
  }

  if (nearest_found != 0U)
  {
    current_sample.nearest_distance_mm = nearest;
    current_sample.moving_distance_cm = (uint16_t)(nearest / 10U);
    current_sample.detect_distance_cm = current_sample.moving_distance_cm;
    /* LD2450没有能量字段，这里用稳定的非零分数兼容上层旧判据。 */
    current_sample.moving_energy = 80U;
  }
  else
  {
    current_sample.nearest_distance_mm = 0U;
    current_sample.moving_distance_cm = 0U;
    current_sample.detect_distance_cm = 0U;
    current_sample.moving_energy = 0U;
  }

  current_sample.static_distance_cm = 0U;
  current_sample.static_energy = 0U;
}

static void radar_parse_ld2450_frame(const uint8_t *buf)
{
  uint8_t i;

  if ((buf[0] != 0xAAU) || (buf[1] != 0xFFU) ||
      (buf[2] != 0x03U) || (buf[3] != 0x00U) ||
      (buf[28] != 0x55U) || (buf[29] != 0xCCU))
  {
    return;
  }

  radar_clear_sample();

  for (i = 0; i < RADAR_MAX_TARGETS; i++)
  {
    const uint8_t *target_buf = &buf[LD2450_HEADER_LEN + ((uint16_t)i * LD2450_TARGET_LEN)];
    const uint16_t raw_x = le16(&target_buf[0]);
    const uint16_t raw_y = le16(&target_buf[2]);
    const uint16_t raw_speed = le16(&target_buf[4]);
    const uint16_t resolution = le16(&target_buf[6]);
    RadarTarget *target = &current_sample.targets[i];

    if ((raw_x == 0U) && (raw_y == 0U) && (raw_speed == 0U) && (resolution == 0U))
    {
      continue;
    }

    target->x_mm = ld2450_decode_signed(raw_x);
    target->y_mm = ld2450_decode_signed(raw_y);
    target->speed_cm_s = ld2450_decode_signed(raw_speed);
    target->resolution_mm = resolution;
    target->distance_mm = target_distance_mm(target->x_mm, target->y_mm);
    target->valid = (target->y_mm > 0) ? 1U : 0U;

    if (target->valid != 0U)
    {
      current_sample.target_count++;
    }
  }

  radar_update_compat_fields();
}

static void radar_feed_ld2450(uint8_t byte)
{
  static const uint8_t header[LD2450_HEADER_LEN] = {0xAAU, 0xFFU, 0x03U, 0x00U};

  if (frame_len < LD2450_HEADER_LEN)
  {
    if (byte != header[frame_len])
    {
      frame_len = (byte == header[0]) ? 1U : 0U;
      frame[0] = byte;
      return;
    }
  }

  frame[frame_len++] = byte;

  if (frame_len >= LD2450_FRAME_LEN)
  {
    radar_parse_ld2450_frame(frame);
    frame_len = 0U;
  }
}

void Radar_Init(void)
{
  radar_clear_sample();
  frame_len = 0U;
  (void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

void Radar_TaskPoll(void)
{
}

RadarSample Radar_GetSample(void)
{
  return current_sample;
}

void Radar_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    radar_feed_ld2450(rx_byte);
    (void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
  }
}
