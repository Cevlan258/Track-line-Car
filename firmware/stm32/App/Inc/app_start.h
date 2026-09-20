#ifndef APP_START_H
#define APP_START_H

#include "main.h"

void AppStart_Init(void);
void AppStart_Poll(void);
uint8_t AppStart_IsStarted(void);
uint32_t AppStart_ElapsedMs(void);
void AppStart_FormatElapsed(char *buf, uint32_t len);

#endif
