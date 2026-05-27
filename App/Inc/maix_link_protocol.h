#ifndef MAIX_LINK_PROTOCOL_H
#define MAIX_LINK_PROTOCOL_H

#include <stdint.h>

#define MAIX_LINK_HEAD_0 0xA6U
#define MAIX_LINK_HEAD_1 0x6AU

#define MAIX_LINK_TYPE_COMMAND 0x01U
#define MAIX_LINK_TYPE_TELEMETRY 0x81U

#define MAIX_LINK_COMMAND_BODY_LEN 11U
#define MAIX_LINK_COMMAND_FRAME_LEN (2U + 1U + MAIX_LINK_COMMAND_BODY_LEN + 2U)

#define MAIX_LINK_MAX_TARGETS 3U
#define MAIX_LINK_TELEMETRY_BODY_LEN (12U + (MAIX_LINK_MAX_TARGETS * 9U))
#define MAIX_LINK_TELEMETRY_FRAME_LEN (2U + 1U + MAIX_LINK_TELEMETRY_BODY_LEN + 2U)

#define MAIX_LINK_CMD_FLAG_LINE_VALID 0x01U
#define MAIX_LINK_CMD_FLAG_FINISH     0x02U
#define MAIX_LINK_CMD_FLAG_AVOIDING   0x04U

#define MAIX_LINK_STATUS_FLAG_STARTED 0x01U
#define MAIX_LINK_STATUS_FLAG_MOVING  0x02U
#define MAIX_LINK_STATUS_FLAG_FAULT   0x04U

typedef enum
{
  MAIX_LINK_MODE_IDLE = 0,
  MAIX_LINK_MODE_RUN,
  MAIX_LINK_MODE_FINISH,
  MAIX_LINK_MODE_FAULT
} MaixLinkMode;

typedef enum
{
  MAIX_LINK_LORA_IDLE = 0,
  MAIX_LINK_LORA_SENDING,
  MAIX_LINK_LORA_SENT,
  MAIX_LINK_LORA_ERROR
} MaixLinkLoraStatus;

typedef enum
{
  MAIX_LINK_PARSE_OK = 0,
  MAIX_LINK_PARSE_LENGTH,
  MAIX_LINK_PARSE_HEADER,
  MAIX_LINK_PARSE_TYPE,
  MAIX_LINK_PARSE_CRC
} MaixLinkParseStatus;

typedef struct
{
  uint8_t seq;
  uint8_t mode;
  uint8_t flags;
  uint8_t route_step;
  uint8_t checkpoint_request;
  uint8_t confidence;
  int16_t vx_mm_s;
  int16_t yaw;
} MaixLinkCommand;

typedef struct
{
  uint8_t valid;
  int16_t x_mm;
  int16_t y_mm;
  int16_t speed_cm_s;
  uint16_t distance_mm;
} MaixLinkRadarTarget;

typedef struct
{
  uint8_t seq;
  uint8_t status_flags;
  uint8_t lora_status;
  uint8_t battery_percent;
  uint16_t battery_x100;
  int32_t distance_mm;
  uint8_t radar_count;
  MaixLinkRadarTarget targets[MAIX_LINK_MAX_TARGETS];
} MaixLinkTelemetry;

uint16_t MaixLink_Crc16(const uint8_t *data, uint16_t len);
uint16_t MaixLink_EncodeCommandFrame(const MaixLinkCommand *command, uint8_t *frame, uint16_t frame_size);
MaixLinkParseStatus MaixLink_ParseCommandFrame(const uint8_t *frame, uint16_t len, MaixLinkCommand *command);
uint16_t MaixLink_EncodeTelemetryFrame(const MaixLinkTelemetry *telemetry, uint8_t *frame, uint16_t frame_size);

#endif
