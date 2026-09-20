#ifndef MAIX_LINK_H
#define MAIX_LINK_H

#include "main.h"
#include "maix_link_protocol.h"
#include "radar.h"

void MaixLink_Init(void);
void MaixLink_TaskPoll(const RadarSample *sample,
                       int32_t distance_mm,
                       uint16_t battery_x100,
                       uint8_t battery_percent,
                       uint8_t started,
                       uint8_t moving,
                       uint8_t fault);
uint8_t MaixLink_GetCommand(MaixLinkCommand *command, uint32_t timeout_ms);
uint8_t MaixLink_IsFresh(uint32_t timeout_ms);
void MaixLink_SetLoraStatus(uint8_t status);
void MaixLink_UartRxCpltCallback(UART_HandleTypeDef *huart);
void MaixLink_UartErrorCallback(UART_HandleTypeDef *huart);

#endif
