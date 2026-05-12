#ifndef AX_CCD_H
#define AX_CCD_H

#include "main.h"

typedef struct
{
  int16_t offset;
  uint16_t threshold;
  uint16_t black_count;
  uint16_t left_edge;
  uint16_t right_edge;
  uint8_t segment_count;
  uint8_t selected_segment;
  uint8_t line_valid;
  uint8_t marker_detected;
  uint8_t road_type;
  uint8_t confidence;
  uint8_t start_detected;
  uint8_t finish_detected;
  uint8_t frame_seq;
} AX_CCD_LineInfo;

typedef enum
{
  AX_CCD_SEGMENT_NEAREST = 0,
  AX_CCD_SEGMENT_LEFT,
  AX_CCD_SEGMENT_RIGHT
} AX_CCD_SegmentPreference;

typedef enum
{
  AX_VISION_ROAD_UNKNOWN = 0,
  AX_VISION_ROAD_STRAIGHT,
  AX_VISION_ROAD_LEFT_CURVE,
  AX_VISION_ROAD_RIGHT_CURVE,
  AX_VISION_ROAD_FORK,
  AX_VISION_ROAD_CROSS,
  AX_VISION_ROAD_T_LEFT,
  AX_VISION_ROAD_T_RIGHT,
  AX_VISION_ROAD_FINISH
} AX_VisionRoadType;

void AX_CCD_Init(void);
void AX_CCD_GetData(uint16_t *pbuf);
int16_t AX_CCD_GetOffset(void);
const uint16_t *AX_CCD_GetLastFrame(void);
AX_CCD_LineInfo AX_CCD_GetLineInfo(void);
void AX_CCD_SetSegmentPreference(AX_CCD_SegmentPreference preference);
void AX_CCD_UartRxCpltCallback(UART_HandleTypeDef *huart);
uint8_t AX_CCD_IsFresh(void);

#endif
