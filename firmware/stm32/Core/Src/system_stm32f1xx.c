/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */  
  
/**
  * 中文说明：本段为工程生成代码说明。
  */

#include "stm32f1xx.h"

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

#if !defined  (HSE_VALUE) 
  #define HSE_VALUE               8000000U /*!< Default value of the External oscillator in Hz.
                                                This value can be provided and adapted by the user application. */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE               8000000U /*!< Default value of the Internal oscillator in Hz.
                                                This value can be provided and adapted by the user application. */
#endif /* HSI_VALUE */

/*!< Uncomment the following line if you need to use external SRAM  */ 
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F100xE || STM32F101xE || STM32F101xG || STM32F103xE || STM32F103xG */

/* 注意：下列向量表地址必须与链接脚本配置一致。 */
/* 如果需要重定位向量表，可取消下列定义；否则保留启动地址自动映射。 */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
/* 如需将向量表重定位到 SRAM，可取消下列定义；否则重定位到 Flash。 */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

  /* SystemCoreClock 会在系统时钟变化时更新，用于保存当前内核时钟。 */
uint32_t SystemCoreClock = 8000000;
const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)
#ifdef DATA_IN_ExtSRAM
  static void SystemInit_ExtMemCtl(void); 
#endif /* DATA_IN_ExtSRAM */
#endif /* STM32F100xE || STM32F101xE || STM32F101xG || STM32F103xE || STM32F103xG */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */
void SystemInit (void)
{
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)
  #ifdef DATA_IN_ExtSRAM
    SystemInit_ExtMemCtl(); 
  #endif /* DATA_IN_ExtSRAM */
#endif 

  /* 配置向量表位置 ----------------------------------------------------------*/
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* 向量表重定位到内部 SRAM。 */
#endif /* USER_VECT_TAB_ADDRESS */
}

/**
  * 中文说明：本段为工程生成代码说明。
  */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0U, pllmull = 0U, pllsource = 0U;

#if defined(STM32F105xC) || defined(STM32F107xC)
  uint32_t prediv1source = 0U, prediv1factor = 0U, prediv2factor = 0U, pll2mull = 0U;
#endif /* STM32F105xC */

#if defined(STM32F100xB) || defined(STM32F100xE)
  uint32_t prediv1factor = 0U;
#endif /* STM32F100xB or STM32F100xE */
    
  /* 获取 SYSCLK 时钟源 -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;
  
  switch (tmp)
  {
    case 0x00U:  /* HSI 作为系统时钟 */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04U:  /* HSE 作为系统时钟 */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08U:  /* PLL 作为系统时钟 */

      /* 获取 PLL 时钟源和倍频系数 --------------------------------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      
#if !defined(STM32F105xC) && !defined(STM32F107xC)      
      pllmull = ( pllmull >> 18U) + 2U;
      
      if (pllsource == 0x00U)
      {
        /* HSI 振荡器二分频作为 PLL 输入 */
        SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
      }
      else
      {
 #if defined(STM32F100xB) || defined(STM32F100xE)
       prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1U;
       /* HSE 振荡器作为 PREDIV1 输入 */
       SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull; 
 #else
        /* HSE 作为 PLL 输入 */
        if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET)
        {/* HSE 振荡器二分频 */
          SystemCoreClock = (HSE_VALUE >> 1U) * pllmull;
        }
        else
        {
          SystemCoreClock = HSE_VALUE * pllmull;
        }
 #endif
      }
#else
      pllmull = pllmull >> 18U;
      
      if (pllmull != 0x0DU)
      {
         pllmull += 2U;
      }
      else
      { /* PLL 倍频系数为输入时钟的 6.5 倍 */
        pllmull = 13U / 2U; 
      }
            
      if (pllsource == 0x00U)
      {
        /* HSI 振荡器二分频作为 PLL 输入 */
        SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
      }
      else
      {/* PREDIV1 作为 PLL 输入 */
        
        /* 获取 PREDIV1 时钟源和分频系数 */
        prediv1source = RCC->CFGR2 & RCC_CFGR2_PREDIV1SRC;
        prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1U;
        
        if (prediv1source == 0U)
        { 
          /* HSE 振荡器作为 PREDIV1 输入 */
          SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;          
        }
        else
        {/* PLL2 时钟作为 PREDIV1 输入 */
          
          /* 获取 PREDIV2 分频系数和 PLL2 倍频系数 */
          prediv2factor = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> 4U) + 1U;
          pll2mull = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> 8U) + 2U; 
          SystemCoreClock = (((HSE_VALUE / prediv2factor) * pll2mull) / prediv1factor) * pllmull;                         
        }
      }
#endif /* STM32F105xC */ 
      break;

    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  
  /* 计算 HCLK 时钟频率 ----------------*/
  /* 获取 HCLK 预分频 */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
  /* HCLK 时钟频率 */
  SystemCoreClock >>= tmp;  
}

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)
/**
  * 中文说明：本段为工程生成代码说明。
  */ 
#ifdef DATA_IN_ExtSRAM
/**
  * 中文说明：本段为工程生成代码说明。
  */ 
void SystemInit_ExtMemCtl(void) 
{
  __IO uint32_t tmpreg;
  /*!< FSMC Bank1 NOR/SRAM3 is used for the STM3210E-EVAL, if another Bank is 
    required, then adjust the Register Addresses */

  /* 使能 FSMC 时钟 */
  RCC->AHBENR = 0x00000114U;

  /* RCC 外设时钟使能后的延时 */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_FSMCEN);
  
  /* 使能 GPIOD、GPIOE、GPIOF 和 GPIOG 时钟 */
  RCC->APB2ENR = 0x000001E0U;
  
  /* RCC 外设时钟使能后的延时 */
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPDEN);

  (void)(tmpreg);
  
/* 中文分区说明 ------------------------------------------------------------*/
/* 中文分区说明 ------------------------------------------------------------*/
/* 中文分区说明 ------------------------------------------------------------*/  
/* 中文分区说明 ------------------------------------------------------------*/
/* 中文分区说明 ------------------------------------------------------------*/
  
  GPIOD->CRL = 0x44BB44BBU;  
  GPIOD->CRH = 0xBBBBBBBBU;

  GPIOE->CRL = 0xB44444BBU;  
  GPIOE->CRH = 0xBBBBBBBBU;

  GPIOF->CRL = 0x44BBBBBBU;  
  GPIOF->CRH = 0xBBBB4444U;

  GPIOG->CRL = 0x44BBBBBBU;  
  GPIOG->CRH = 0x444B4B44U;
   
/* 中文分区说明 ------------------------------------------------------------*/  
/* 中文分区说明 ------------------------------------------------------------*/
  
  FSMC_Bank1->BTCR[4U] = 0x00001091U;
  FSMC_Bank1->BTCR[5U] = 0x00110212U;
}
#endif /* DATA_IN_ExtSRAM */
#endif /* STM32F100xE || STM32F101xE || STM32F101xG || STM32F103xE || STM32F103xG */

/**
  * 中文说明：本段为工程生成代码说明。
  */

/**
  * 中文说明：本段为工程生成代码说明。
  */
  
/**
  * 中文说明：本段为工程生成代码说明。
  */
