#include "app_start.h"
#include "ax_key.h"
#include "cmsis_os.h"
#include <stdio.h>

static uint8_t started;
static uint8_t last_key;
static uint32_t start_tick;
static uint32_t last_edge_tick;

void AppStart_Init(void)
{
  started = 0;
  last_key = AX_KEY_Scan();
  start_tick = 0;
  last_edge_tick = osKernelGetTickCount();
}

void AppStart_Poll(void)
{
  const uint32_t now = osKernelGetTickCount();
  const uint8_t key = AX_KEY_Scan();

  if ((key != last_key) && ((now - last_edge_tick) > 40U))
  {
    last_edge_tick = now;
    last_key = key;

    if ((key != 0U) && (started == 0U))
    {
      started = 1;
      start_tick = now;
    }
  }
}

uint8_t AppStart_IsStarted(void)
{
  return started;
}

uint32_t AppStart_ElapsedMs(void)
{
  if (started == 0U)
  {
    return 0;
  }

  return osKernelGetTickCount() - start_tick;
}

void AppStart_FormatElapsed(char *buf, uint32_t len)
{
  const uint32_t elapsed_s = AppStart_ElapsedMs() / 1000U;
  const uint32_t minutes = elapsed_s / 60U;
  const uint32_t seconds = elapsed_s % 60U;

  if ((buf != NULL) && (len > 0U))
  {
    (void)snprintf(buf, len, "%02lu:%02lu", minutes, seconds);
  }
}
