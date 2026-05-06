#include "radar.h"
#include "usart.h"

#define RADAR_FRAME_MAX 96U
#define RADAR_SIMPLE_FRAME_LEN 5U
#define RADAR_STANDARD_MIN_LEN 16U
#define RADAR_STANDARD_DATA_TYPE 0x01U
#define RADAR_LEGACY_DATA_TYPE 0x02U
#define RADAR_LEGACY_HEAD 0xAAU
#define RADAR_ENERGY_GATES 16U

static uint8_t rx_byte;
static uint8_t frame[RADAR_FRAME_MAX];
static uint8_t frame_len;
static uint8_t simple_frame[RADAR_SIMPLE_FRAME_LEN];
static uint8_t simple_len;
static RadarSample current_sample;

static uint16_t le16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t le32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint8_t clamp_energy(uint32_t value)
{
  return (value > 255UL) ? 255U : (uint8_t)value;
}

static void radar_publish_simple(uint8_t target_state, uint16_t distance_cm)
{
  current_sample.target_state = target_state;
  current_sample.moving_distance_cm = distance_cm;
  current_sample.static_distance_cm = 0;
  current_sample.detect_distance_cm = distance_cm;
  current_sample.moving_energy = (target_state != 0U) ? 100U : 0U;
  current_sample.static_energy = 0;
  current_sample.valid = 1;
}

static void radar_parse_standard_frame(const uint8_t *buf, uint8_t len)
{
  uint16_t payload_len;

  if (len < RADAR_STANDARD_MIN_LEN)
  {
    return;
  }

  if ((buf[0] != 0xF4U) || (buf[1] != 0xF3U) || (buf[2] != 0xF2U) || (buf[3] != 0xF1U))
  {
    return;
  }

  if ((buf[len - 4U] != 0xF8U) || (buf[len - 3U] != 0xF7U) ||
      (buf[len - 2U] != 0xF6U) || (buf[len - 1U] != 0xF5U))
  {
    return;
  }

  payload_len = le16(&buf[4]);
  if ((uint16_t)(payload_len + 10U) != (uint16_t)len)
  {
    return;
  }

  if (buf[6] == RADAR_STANDARD_DATA_TYPE)
  {
    uint8_t i;
    uint32_t energy_max = 0;
    const uint16_t distance_cm = le16(&buf[8]);

    if (payload_len >= 70U)
    {
      for (i = 0; i < RADAR_ENERGY_GATES; i++)
      {
        const uint32_t energy = le32(&buf[12U + (uint16_t)i * 4U]);
        if (energy > energy_max)
        {
          energy_max = energy;
        }
      }
    }

    current_sample.target_state = buf[7];
    current_sample.moving_distance_cm = distance_cm;
    current_sample.moving_energy = clamp_energy(energy_max);
    current_sample.static_distance_cm = 0;
    current_sample.static_energy = 0;
    current_sample.detect_distance_cm = distance_cm;
    current_sample.valid = 1;
  }
  else if ((buf[6] == RADAR_LEGACY_DATA_TYPE) && (buf[7] == RADAR_LEGACY_HEAD) && (len >= 23U))
  {
    current_sample.target_state = buf[8];
    current_sample.moving_distance_cm = le16(&buf[9]);
    current_sample.moving_energy = buf[11];
    current_sample.static_distance_cm = le16(&buf[12]);
    current_sample.static_energy = buf[14];
    current_sample.detect_distance_cm = le16(&buf[15]);
    current_sample.valid = 1;
  }
}

static void radar_feed_simple(uint8_t byte)
{
  if (simple_len == 0U)
  {
    if (byte != 0x6EU)
    {
      return;
    }
  }

  simple_frame[simple_len++] = byte;
  if (simple_len >= RADAR_SIMPLE_FRAME_LEN)
  {
    if ((simple_frame[0] == 0x6EU) && (simple_frame[4] == 0x62U))
    {
      radar_publish_simple(simple_frame[1], le16(&simple_frame[2]));
    }
    simple_len = 0;
  }
}

static void radar_feed_standard(uint8_t byte)
{
  if (frame_len == 0U)
  {
    if (byte != 0xF4U)
    {
      return;
    }
  }

  frame[frame_len++] = byte;

  if (frame_len >= RADAR_FRAME_MAX)
  {
    frame_len = 0;
    return;
  }

  if ((frame_len >= 4U) &&
      ((frame[0] != 0xF4U) || (frame[1] != 0xF3U) || (frame[2] != 0xF2U) || (frame[3] != 0xF1U)))
  {
    frame_len = 0;
    return;
  }

  if ((frame_len >= 8U) &&
      (frame[frame_len - 4U] == 0xF8U) && (frame[frame_len - 3U] == 0xF7U) &&
      (frame[frame_len - 2U] == 0xF6U) && (frame[frame_len - 1U] == 0xF5U))
  {
    radar_parse_standard_frame(frame, frame_len);
    frame_len = 0;
  }
}

void Radar_Init(void)
{
  current_sample.valid = 0;
  frame_len = 0;
  simple_len = 0;
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
    radar_feed_simple(rx_byte);
    radar_feed_standard(rx_byte);
    (void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
  }
}
