#ifndef RADAR_H
#define RADAR_H

#include "main.h"

typedef enum
{
  RADAR_SIDE_UNKNOWN = 0,
  RADAR_SIDE_LEFT,
  RADAR_SIDE_RIGHT
} RadarSide;

typedef struct
{
  uint8_t valid;
  uint8_t target_state;
  uint16_t moving_distance_cm;
  uint8_t moving_energy;
  uint16_t static_distance_cm;
  uint8_t static_energy;
  uint16_t detect_distance_cm;
} RadarSample;

void Radar_Init(void);
void Radar_TaskPoll(void);
RadarSample Radar_GetSample(void);
void Radar_UartRxCpltCallback(UART_HandleTypeDef *huart);

#endif
