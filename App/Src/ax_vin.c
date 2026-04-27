#include "ax_vin.h"
#include "adc.h"

#define ADC_REVISE 99.9f

void AX_VIN_Init(void)
{
  HAL_ADCEx_Calibration_Start(&hadc2);
}

uint16_t AX_VIN_GetVol_X100(void)
{
  uint32_t raw = 0;

  HAL_ADC_Start(&hadc2);
  if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
  {
    raw = HAL_ADC_GetValue(&hadc2);
  }
  HAL_ADC_Stop(&hadc2);

  return (uint16_t)((3.3f * 11.0f * ADC_REVISE * (float)raw) / 4095.0f);
}
