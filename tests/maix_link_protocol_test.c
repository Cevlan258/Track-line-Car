#include "maix_link_protocol.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void test_command_round_trip(void)
{
  MaixLinkCommand input;
  MaixLinkCommand output;
  uint8_t frame[MAIX_LINK_COMMAND_FRAME_LEN];
  uint16_t len;

  memset(&input, 0, sizeof(input));
  input.seq = 42U;
  input.mode = MAIX_LINK_MODE_RUN;
  input.flags = MAIX_LINK_CMD_FLAG_LINE_VALID;
  input.route_step = 3U;
  input.checkpoint_request = 2U;
  input.confidence = 88U;
  input.vx_mm_s = 320;
  input.yaw = -750;

  len = MaixLink_EncodeCommandFrame(&input, frame, sizeof(frame));
  assert(len == MAIX_LINK_COMMAND_FRAME_LEN);
  assert(MaixLink_ParseCommandFrame(frame, len, &output) == MAIX_LINK_PARSE_OK);
  assert(output.seq == input.seq);
  assert(output.mode == input.mode);
  assert(output.flags == input.flags);
  assert(output.route_step == input.route_step);
  assert(output.checkpoint_request == input.checkpoint_request);
  assert(output.confidence == input.confidence);
  assert(output.vx_mm_s == input.vx_mm_s);
  assert(output.yaw == input.yaw);
}

static void test_command_crc_rejects_corruption(void)
{
  MaixLinkCommand input;
  MaixLinkCommand output;
  uint8_t frame[MAIX_LINK_COMMAND_FRAME_LEN];

  memset(&input, 0, sizeof(input));
  input.seq = 7U;
  input.mode = MAIX_LINK_MODE_RUN;
  input.vx_mm_s = 180;
  input.yaw = 400;

  assert(MaixLink_EncodeCommandFrame(&input, frame, sizeof(frame)) == MAIX_LINK_COMMAND_FRAME_LEN);
  frame[10] ^= 0x01U;
  assert(MaixLink_ParseCommandFrame(frame, sizeof(frame), &output) == MAIX_LINK_PARSE_CRC);
}

static void test_telemetry_frame_layout(void)
{
  MaixLinkTelemetry telemetry;
  uint8_t frame[MAIX_LINK_TELEMETRY_FRAME_LEN];
  uint16_t len;
  uint16_t crc_expected;
  uint16_t crc_actual;

  memset(&telemetry, 0, sizeof(telemetry));
  telemetry.seq = 9U;
  telemetry.status_flags = MAIX_LINK_STATUS_FLAG_STARTED | MAIX_LINK_STATUS_FLAG_MOVING;
  telemetry.lora_status = MAIX_LINK_LORA_SENT;
  telemetry.battery_percent = 76U;
  telemetry.battery_x100 = 1184U;
  telemetry.distance_mm = 123456L;
  telemetry.radar_count = 2U;
  telemetry.targets[0].valid = 1U;
  telemetry.targets[0].x_mm = -230;
  telemetry.targets[0].y_mm = 880;
  telemetry.targets[0].speed_cm_s = 0;
  telemetry.targets[0].distance_mm = 910U;
  telemetry.targets[1].valid = 1U;
  telemetry.targets[1].x_mm = 410;
  telemetry.targets[1].y_mm = 960;
  telemetry.targets[1].speed_cm_s = -12;
  telemetry.targets[1].distance_mm = 1040U;

  len = MaixLink_EncodeTelemetryFrame(&telemetry, frame, sizeof(frame));
  assert(len == MAIX_LINK_TELEMETRY_FRAME_LEN);
  assert(frame[0] == MAIX_LINK_HEAD_0);
  assert(frame[1] == MAIX_LINK_HEAD_1);
  assert(frame[2] == MAIX_LINK_TELEMETRY_BODY_LEN);
  assert(frame[3] == MAIX_LINK_TYPE_TELEMETRY);
  assert(frame[4] == 9U);
  assert(frame[5] == telemetry.status_flags);
  assert(frame[6] == MAIX_LINK_LORA_SENT);
  assert(frame[7] == 76U);
  assert(frame[14] == 2U);

  crc_expected = MaixLink_Crc16(&frame[2], (uint16_t)(MAIX_LINK_TELEMETRY_FRAME_LEN - 4U));
  crc_actual = (uint16_t)frame[MAIX_LINK_TELEMETRY_FRAME_LEN - 2U] |
               ((uint16_t)frame[MAIX_LINK_TELEMETRY_FRAME_LEN - 1U] << 8);
  assert(crc_actual == crc_expected);
}

int main(void)
{
  test_command_round_trip();
  test_command_crc_rejects_corruption();
  test_telemetry_frame_layout();
  puts("maix_link_protocol_test: PASS");
  return 0;
}
