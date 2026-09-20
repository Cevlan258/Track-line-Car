/* USER CODE BEGIN Header */
/**
  * 中文说明：本段为工程生成代码说明。
  */
/* USER CODE END Header */

/* 包含文件 ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有宏 --------------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef        htim6;
/* 私有函数声明 --------------------------------------------------------------*/
void TIM6_IRQHandler(void);
/* Private functions ---------------------------------------------------------*/

/**
  * 中文说明：本段为工程生成代码说明。
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock, uwAPB1Prescaler = 0U;

  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;

  HAL_StatusTypeDef     status = HAL_OK;

  /* 使能 TIM6 时钟 */
  __HAL_RCC_TIM6_CLK_ENABLE();

  /* 获取时钟配置 */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* 获取 APB1 预分频 */
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;
  /* 计算 TIM6 时钟 */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
  }

  /* 计算预分频值，使 TIM6 计数时钟等于 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* 初始化 TIM6 */
  htim6.Instance = TIM6;

  /* 按以下方式初始化 TIMx 外设：
   * Period = [(TIM6CLK/1000) - 1]. to have a (1/1000) s time base.
   * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   * ClockDivision = 0
   * Counter direction = Up
   */
  htim6.Init.Period = (1000000U / 1000U) - 1U;
  htim6.Init.Prescaler = uwPrescalerValue;
  htim6.Init.ClockDivision = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  status = HAL_TIM_Base_Init(&htim6);
  if (status == HAL_OK)
  {
    /* 以中断模式启动 TIM 时基 */
    status = HAL_TIM_Base_Start_IT(&htim6);
    if (status == HAL_OK)
    {
    /* 使能 TIM6 全局中断 */
        HAL_NVIC_EnableIRQ(TIM6_IRQn);
      /* Configure the SysTick IRQ priority */
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* 配置 TIM 中断优先级 */
        HAL_NVIC_SetPriority(TIM6_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

 /* 返回函数状态 */
  return status;
}

/**
  * 中文说明：本段为工程生成代码说明。
  */
void HAL_SuspendTick(void)
{
  /* 禁用 TIM6 更新中断 */
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
  * 中文说明：本段为工程生成代码说明。
  */
void HAL_ResumeTick(void)
{
  /* 使能 TIM6 更新中断 */
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

