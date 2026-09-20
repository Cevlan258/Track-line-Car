#ifndef LORA_H
#define LORA_H

#include "main.h"

void LoRa_Init(void);
HAL_StatusTypeDef LoRa_SendCheckpoint(uint8_t checkpoint, uint32_t elapsed_ms);

#endif
