#include "ax_ccd.h"
#include "adc.h"
#include "ax_delay.h"

static void ccd_delay_us(void)
{
  AX_Delayus(1);
}

static uint16_t ccd_get_adc_data(void)
{
  uint32_t value = 0;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    value = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  return (uint16_t)value;
}

void AX_CCD_Init(void)
{
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
}

void AX_CCD_GetData(uint16_t *pbuf)
{
  uint8_t i;

  HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
  ccd_delay_us();

  HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);
  ccd_delay_us();

  HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
  ccd_delay_us();

  for (i = 0; i < 128; i++)
  {
    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);
    ccd_delay_us();
    pbuf[i] = ccd_get_adc_data() >> 4;
    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
    ccd_delay_us();
  }
}

int16_t AX_CCD_GetOffset(void)
{
  uint8_t i;
  static uint16_t ccd[128];
  uint16_t ccd_min;
  uint16_t ccd_max;
  uint16_t ccd_threshold;
  uint16_t ccd_left = 0;
  uint16_t ccd_right = 127;

  AX_CCD_GetData(ccd);

  ccd_min = ccd[0];
  ccd_max = ccd[0];
  for (i = 0; i < 128; i++)
  {
    if (ccd_min > ccd[i])
    {
      ccd_min = ccd[i];
    }
    if (ccd_max < ccd[i])
    {
      ccd_max = ccd[i];
    }
  }

  ccd_threshold = (ccd_min + ccd_max) / 2U;

  for (i = 0; i < 128; i++)
  {
    if (ccd[i] < ccd_threshold)
    {
      ccd_left = i;
      break;
    }
  }

  for (i = 127; i > 0; i--)
  {
    if (ccd[i] < ccd_threshold)
    {
      ccd_right = i;
      break;
    }
  }

  return (int16_t)(((int16_t)ccd_left + (int16_t)ccd_right) / 2 - 64);
}
