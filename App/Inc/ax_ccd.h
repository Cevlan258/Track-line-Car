#ifndef AX_CCD_H
#define AX_CCD_H

#include "main.h"

void AX_CCD_Init(void);
void AX_CCD_GetData(uint16_t *pbuf);
int16_t AX_CCD_GetOffset(void);

#endif
