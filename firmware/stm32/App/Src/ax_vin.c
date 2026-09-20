#include "ax_vin.h"
#include "adc.h"

#define ADC_REVISE 99.9f

typedef struct
{
  uint16_t voltage_x100;
  uint8_t percent;
} VinSocPoint;

static const VinSocPoint vin_soc_table[] = {
  {1260U, 100U},
  {1240U, 90U},
  {1220U, 80U},
  {1200U, 70U},
  {1180U, 60U},
  {1160U, 50U},
  {1140U, 40U},
  {1110U, 30U},
  {1080U, 20U},
  {1050U, 10U},
  {990U, 0U},
};

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

uint8_t AX_VIN_GetPercent(uint16_t vin_x100)
{
  uint8_t i;

  if (vin_x100 >= vin_soc_table[0].voltage_x100)
  {
    return 100U;
  }

  for (i = 0; i < (uint8_t)((sizeof(vin_soc_table) / sizeof(vin_soc_table[0])) - 1U); i++)
  {
    const VinSocPoint high = vin_soc_table[i];
    const VinSocPoint low = vin_soc_table[i + 1U];

    if ((vin_x100 <= high.voltage_x100) && (vin_x100 >= low.voltage_x100))
    {
      const uint16_t voltage_span = high.voltage_x100 - low.voltage_x100;
      const uint16_t voltage_delta = vin_x100 - low.voltage_x100;
      const uint8_t percent_span = high.percent - low.percent;

      if (voltage_span == 0U)
      {
        return low.percent;
      }

      return (uint8_t)(low.percent + ((uint32_t)voltage_delta * percent_span) / voltage_span);
    }
  }

  return 0U;
}
