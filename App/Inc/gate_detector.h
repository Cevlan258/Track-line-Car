#ifndef GATE_DETECTOR_H
#define GATE_DETECTOR_H

#include "main.h"
#include "radar.h"

typedef enum
{
  GATE_EVENT_NONE = 0,
  GATE_EVENT_PASSED
} GateDetectorEvent;

void GateDetector_Init(void);
void GateDetector_Update(const RadarSample *sample, uint32_t now_ms, int32_t distance_mm);
GateDetectorEvent GateDetector_GetEvent(void);
uint8_t GateDetector_GetCount(void);
const char *GateDetector_GetStateName(void);
uint16_t GateDetector_GetLastScore(void);

#endif
