#ifndef DISPLAY_SSD1309_H
#define DISPLAY_SSD1309_H

#include "ax_ccd.h"
#include "radar.h"

void Display_Init(void);
void Display_ShowStatus(const char *state, uint32_t elapsed_ms);
void Display_ShowStatusTime(const char *state, uint32_t elapsed_ms, const char *time_text);
void Display_ShowLineSensor(const uint16_t *frame, const AX_CCD_LineInfo *info, const char *state, uint16_t vin_x100, uint8_t battery_percent);
void Display_ShowLineGate(const uint16_t *frame, const AX_CCD_LineInfo *info, const RadarSample *sample, const char *state, uint8_t gate_count, const char *gate_state, uint16_t gate_score, uint16_t vin_x100, uint8_t battery_percent);
void Display_ShowRadar(const RadarSample *sample, RadarSide side, const char *state);
void Display_ShowRadarScope(const RadarSample *sample, uint8_t avoid_flags, const char *time_text);
void Display_ShowLora(uint8_t checkpoint, uint8_t sent, uint32_t elapsed_ms);

#endif
