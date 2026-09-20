#include "ax_ccd.h"
#include "app_config.h"
#include "usart.h"
#include <string.h>

#define OPENMV_HEAD_0 0xAAU
#define OPENMV_HEAD_1 0x55U
#define OPENMV_PAYLOAD_LEN 10U
#define OPENMV_FRAME_LEN (OPENMV_PAYLOAD_LEN + 4U)

#define OPENMV_FLAG_LINE_VALID 0x01U
#define OPENMV_FLAG_START      0x02U
#define OPENMV_FLAG_FINISH     0x04U
#define OPENMV_FLAG_MARKER     0x08U

#ifndef APP_OPENMV_TIMEOUT_MS
#define APP_OPENMV_TIMEOUT_MS 150U
#endif

static uint16_t ccd_last_frame[128];
static volatile AX_CCD_LineInfo ccd_line_info;
static AX_CCD_SegmentPreference ccd_segment_preference = AX_CCD_SEGMENT_NEAREST;
static AX_CCD_SegmentPreference ccd_last_sent_preference = AX_CCD_SEGMENT_RIGHT;
static volatile uint32_t openmv_last_rx_ms;
static uint8_t openmv_rx_byte;
static uint8_t openmv_frame[OPENMV_FRAME_LEN];
static uint8_t openmv_index;

static int16_t clamp_offset(int16_t offset)
{
  if (offset < -64)
  {
    return -64;
  }
  if (offset > 63)
  {
    return 63;
  }
  return offset;
}

static uint8_t clamp_pixel(uint8_t value)
{
  return (value > 127U) ? 127U : value;
}

static uint8_t openmv_checksum(const uint8_t *frame)
{
  uint8_t checksum = 0;
  uint8_t i;

  for (i = 2U; i < (OPENMV_FRAME_LEN - 1U); i++)
  {
    checksum ^= frame[i];
  }

  return checksum;
}

static int16_t le16_signed(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void update_synthetic_frame(const AX_CCD_LineInfo *info)
{
  uint8_t i;

  for (i = 0U; i < 128U; i++)
  {
    ccd_last_frame[i] = 255U;
  }

  if ((info->line_valid != 0U) && (info->left_edge <= info->right_edge))
  {
    for (i = (uint8_t)info->left_edge; i <= (uint8_t)info->right_edge && i < 128U; i++)
    {
      ccd_last_frame[i] = 0U;
    }
  }
}

static AX_CCD_LineInfo line_info_with_timeout(void)
{
  AX_CCD_LineInfo info = ccd_line_info;
  const uint8_t fresh = ((HAL_GetTick() - openmv_last_rx_ms) <= APP_OPENMV_TIMEOUT_MS) ? 1U : 0U;

  info.fresh = fresh;
  if (fresh == 0U)
  {
    info.line_valid = 0U;
    info.confidence = 0U;
  }

  return info;
}

static void publish_openmv_frame(const uint8_t *frame)
{
  AX_CCD_LineInfo info;
  const uint8_t flags = frame[4];
  int16_t offset = le16_signed(&frame[5]);
  uint8_t left = clamp_pixel(frame[7]);
  uint8_t right = clamp_pixel(frame[8]);

  offset = clamp_offset(offset);

  if (left > right)
  {
    const uint8_t temp = left;
    left = right;
    right = temp;
  }

  memset(&info, 0, sizeof(info));
  info.offset = offset;
  info.threshold = 128U;
  info.black_count = ((flags & OPENMV_FLAG_LINE_VALID) != 0U) ? (uint16_t)(right - left + 1U) : 0U;
  info.left_edge = left;
  info.right_edge = right;
  info.segment_count = frame[9];
  info.selected_segment = frame[10];
  info.line_valid = ((flags & OPENMV_FLAG_LINE_VALID) != 0U) ? 1U : 0U;
  info.marker_detected = ((flags & OPENMV_FLAG_MARKER) != 0U) ? 1U : 0U;
  info.road_type = frame[11];
  info.confidence = (frame[12] > 100U) ? 100U : frame[12];
  info.start_detected = ((flags & OPENMV_FLAG_START) != 0U) ? 1U : 0U;
  info.finish_detected = ((flags & OPENMV_FLAG_FINISH) != 0U) ? 1U : 0U;
  info.frame_seq = frame[3];
  info.fresh = 1U;

  ccd_line_info = info;
  openmv_last_rx_ms = HAL_GetTick();
  update_synthetic_frame(&info);
}

static void openmv_feed(uint8_t byte)
{
  if (openmv_index == 0U)
  {
    if (byte != OPENMV_HEAD_0)
    {
      return;
    }
  }
  else if (openmv_index == 1U)
  {
    if (byte != OPENMV_HEAD_1)
    {
      openmv_index = (byte == OPENMV_HEAD_0) ? 1U : 0U;
      openmv_frame[0] = OPENMV_HEAD_0;
      return;
    }
  }

  openmv_frame[openmv_index++] = byte;

  if (openmv_index == 3U)
  {
    if (openmv_frame[2] != OPENMV_PAYLOAD_LEN)
    {
      openmv_index = 0U;
    }
  }
  else if (openmv_index >= OPENMV_FRAME_LEN)
  {
    if (openmv_checksum(openmv_frame) == openmv_frame[OPENMV_FRAME_LEN - 1U])
    {
      publish_openmv_frame(openmv_frame);
    }
    openmv_index = 0U;
  }
}

void AX_CCD_Init(void)
{
  AX_CCD_LineInfo initial_info;

  memset(&initial_info, 0, sizeof(initial_info));
  initial_info.offset = 0;
  initial_info.threshold = 128U;
  initial_info.left_edge = 56U;
  initial_info.right_edge = 72U;
  initial_info.road_type = AX_VISION_ROAD_UNKNOWN;
  initial_info.fresh = 0U;
  ccd_line_info = initial_info;
  openmv_last_rx_ms = HAL_GetTick();
  update_synthetic_frame(&initial_info);

  (void)HAL_UART_Receive_IT(&huart3, &openmv_rx_byte, 1U);
}

void AX_CCD_GetData(uint16_t *pbuf)
{
  if (pbuf == NULL)
  {
    return;
  }

  memcpy(pbuf, ccd_last_frame, sizeof(ccd_last_frame));
}

int16_t AX_CCD_GetOffset(void)
{
  return line_info_with_timeout().offset;
}

const uint16_t *AX_CCD_GetLastFrame(void)
{
  return ccd_last_frame;
}

AX_CCD_LineInfo AX_CCD_GetLineInfo(void)
{
  return line_info_with_timeout();
}

void AX_CCD_SetSegmentPreference(AX_CCD_SegmentPreference preference)
{
  uint8_t cmd[4];

  ccd_segment_preference = preference;
  if (ccd_last_sent_preference == preference)
  {
    return;
  }

  cmd[0] = 0xA5U;
  cmd[1] = 0x5AU;
  cmd[2] = (uint8_t)preference;
  cmd[3] = (uint8_t)(cmd[0] ^ cmd[1] ^ cmd[2]);
  (void)HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 5U);
  ccd_last_sent_preference = preference;
}

void AX_CCD_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  openmv_feed(openmv_rx_byte);
  (void)HAL_UART_Receive_IT(&huart3, &openmv_rx_byte, 1U);
}

uint8_t AX_CCD_IsFresh(void)
{
  return ((HAL_GetTick() - openmv_last_rx_ms) <= APP_OPENMV_TIMEOUT_MS) ? 1U : 0U;
}
