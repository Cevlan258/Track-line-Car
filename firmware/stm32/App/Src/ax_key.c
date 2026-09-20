#include "ax_key.h"

void AX_KEY_Init(void)
{
}

uint8_t AX_KEY_Scan(void)
{
  return HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET ? 1U : 0U;
}
