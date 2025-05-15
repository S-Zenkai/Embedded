/**
  ******************************************************************************
  * @file    cs32f10x_conf.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, CKS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CS32F10x_CONF_H
#define __CS32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
#include "cs32f10x_adc.h"
#include "cs32f10x_bkp.h"
#include "cs32f10x_can.h"
//#include "cs32f10x_cec.h"
#include "cs32f10x_crc.h"
//#include "cs32f10x_dac.h"
#include "cs32f10x_dbgmcu.h"
#include "cs32f10x_dma.h"
#include "cs32f10x_exti.h"
#include "cs32f10x_flash.h"
//#include "cs32f10x_fsmc.h"
#include "cs32f10x_gpio.h"
#include "cs32f10x_i2c.h"
#include "cs32f10x_iwdg.h"
#include "cs32f10x_pwr.h"
#include "cs32f10x_rcc.h"
#include "cs32f10x_rtc.h"
//#include "cs32f10x_sdio.h"
#include "cs32f10x_spi.h"
#include "cs32f10x_tim.h" 
#include "cs32f10x_usart.h"
#include "cs32f10x_wwdg.h"
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __CS32F10x_CONF_H */

/******************* (C) COPYRIGHT 2017 CKS *****END OF FILE****/
