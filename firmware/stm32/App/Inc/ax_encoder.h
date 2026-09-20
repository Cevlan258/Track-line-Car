#ifndef AX_ENCODER_H
#define AX_ENCODER_H

#include "main.h"

void AX_ENCODER_A_Init(void);
uint16_t AX_ENCODER_A_GetCounter(void);
void AX_ENCODER_A_SetCounter(uint16_t count);

void AX_ENCODER_B_Init(void);
uint16_t AX_ENCODER_B_GetCounter(void);
void AX_ENCODER_B_SetCounter(uint16_t count);

#endif
