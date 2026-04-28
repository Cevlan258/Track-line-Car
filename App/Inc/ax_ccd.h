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
  uint8_t line_valid;
  uint8_t marker_detected;
} AX_CCD_LineInfo;

void AX_CCD_Init(void);
void AX_CCD_GetData(uint16_t *pbuf);
int16_t AX_CCD_GetOffset(void);
const uint16_t *AX_CCD_GetLastFrame(void);
AX_CCD_LineInfo AX_CCD_GetLineInfo(void);

#endif
