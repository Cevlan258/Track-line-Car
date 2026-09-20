#include "ax_delay.h"

void AX_DELAY_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void AX_Delayus(uint32_t us)
{
  const uint32_t start = DWT->CYCCNT;
  const uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;

  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

void AX_Delayms(uint16_t ms)
{
  HAL_Delay(ms);
}
