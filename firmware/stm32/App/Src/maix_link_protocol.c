#include "maix_link_protocol.h"

#include <string.h>

static void put_u16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)(value & 0xFFU);
  p[1] = (uint8_t)(value >> 8);
}

static void put_i16(uint8_t *p, int16_t value)
{
  put_u16(p, (uint16_t)value);
}

static void put_i32(uint8_t *p, int32_t value)
{
  const uint32_t raw = (uint32_t)value;
  p[0] = (uint8_t)(raw & 0xFFUL);
  p[1] = (uint8_t)((raw >> 8) & 0xFFUL);
  p[2] = (uint8_t)((raw >> 16) & 0xFFUL);
  p[3] = (uint8_t)((raw >> 24) & 0xFFUL);
}

static uint16_t get_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t get_i16(const uint8_t *p)
{
  return (int16_t)get_u16(p);
}

uint16_t MaixLink_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

uint16_t MaixLink_EncodeCommandFrame(const MaixLinkCommand *command, uint8_t *frame, uint16_t frame_size)
{
  uint16_t crc;

  if ((command == NULL) || (frame == NULL) || (frame_size < MAIX_LINK_COMMAND_FRAME_LEN))
  {
    return 0U;
  }

  memset(frame, 0, MAIX_LINK_COMMAND_FRAME_LEN);
  frame[0] = MAIX_LINK_HEAD_0;
  frame[1] = MAIX_LINK_HEAD_1;
  frame[2] = MAIX_LINK_COMMAND_BODY_LEN;
  frame[3] = MAIX_LINK_TYPE_COMMAND;
  frame[4] = command->seq;
  frame[5] = command->mode;
  frame[6] = command->flags;
  frame[7] = command->route_step;
  frame[8] = command->checkpoint_request;
  frame[9] = command->confidence;
  put_i16(&frame[10], command->vx_mm_s);
  put_i16(&frame[12], command->yaw);

  crc = MaixLink_Crc16(&frame[2], (uint16_t)(MAIX_LINK_COMMAND_FRAME_LEN - 4U));
  put_u16(&frame[MAIX_LINK_COMMAND_FRAME_LEN - 2U], crc);
  return MAIX_LINK_COMMAND_FRAME_LEN;
}

MaixLinkParseStatus MaixLink_ParseCommandFrame(const uint8_t *frame, uint16_t len, MaixLinkCommand *command)
{
  uint16_t expected_crc;
  uint16_t actual_crc;

  if ((frame == NULL) || (command == NULL) || (len != MAIX_LINK_COMMAND_FRAME_LEN))
  {
    return MAIX_LINK_PARSE_LENGTH;
  }

  if ((frame[0] != MAIX_LINK_HEAD_0) || (frame[1] != MAIX_LINK_HEAD_1) ||
      (frame[2] != MAIX_LINK_COMMAND_BODY_LEN))
  {
    return MAIX_LINK_PARSE_HEADER;
  }

  if (frame[3] != MAIX_LINK_TYPE_COMMAND)
  {
    return MAIX_LINK_PARSE_TYPE;
  }

  expected_crc = MaixLink_Crc16(&frame[2], (uint16_t)(MAIX_LINK_COMMAND_FRAME_LEN - 4U));
  actual_crc = get_u16(&frame[MAIX_LINK_COMMAND_FRAME_LEN - 2U]);
  if (actual_crc != expected_crc)
  {
    return MAIX_LINK_PARSE_CRC;
  }

  command->seq = frame[4];
  command->mode = frame[5];
  command->flags = frame[6];
  command->route_step = frame[7];
  command->checkpoint_request = frame[8];
  command->confidence = frame[9];
  command->vx_mm_s = get_i16(&frame[10]);
  command->yaw = get_i16(&frame[12]);

  return MAIX_LINK_PARSE_OK;
}

uint16_t MaixLink_EncodeTelemetryFrame(const MaixLinkTelemetry *telemetry, uint8_t *frame, uint16_t frame_size)
{
  uint8_t i;
  uint16_t crc;

  if ((telemetry == NULL) || (frame == NULL) || (frame_size < MAIX_LINK_TELEMETRY_FRAME_LEN))
  {
    return 0U;
  }

  memset(frame, 0, MAIX_LINK_TELEMETRY_FRAME_LEN);
  frame[0] = MAIX_LINK_HEAD_0;
  frame[1] = MAIX_LINK_HEAD_1;
  frame[2] = MAIX_LINK_TELEMETRY_BODY_LEN;
  frame[3] = MAIX_LINK_TYPE_TELEMETRY;
  frame[4] = telemetry->seq;
  frame[5] = telemetry->status_flags;
  frame[6] = telemetry->lora_status;
  frame[7] = telemetry->battery_percent;
  put_u16(&frame[8], telemetry->battery_x100);
  put_i32(&frame[10], telemetry->distance_mm);
  frame[14] = (telemetry->radar_count > MAIX_LINK_MAX_TARGETS) ? MAIX_LINK_MAX_TARGETS : telemetry->radar_count;

  for (i = 0U; i < MAIX_LINK_MAX_TARGETS; i++)
  {
    const uint8_t offset = (uint8_t)(15U + (i * 9U));
    frame[offset] = telemetry->targets[i].valid;
    put_i16(&frame[offset + 1U], telemetry->targets[i].x_mm);
    put_i16(&frame[offset + 3U], telemetry->targets[i].y_mm);
    put_i16(&frame[offset + 5U], telemetry->targets[i].speed_cm_s);
    put_u16(&frame[offset + 7U], telemetry->targets[i].distance_mm);
  }

  crc = MaixLink_Crc16(&frame[2], (uint16_t)(MAIX_LINK_TELEMETRY_FRAME_LEN - 4U));
  put_u16(&frame[MAIX_LINK_TELEMETRY_FRAME_LEN - 2U], crc);
  return MAIX_LINK_TELEMETRY_FRAME_LEN;
}
