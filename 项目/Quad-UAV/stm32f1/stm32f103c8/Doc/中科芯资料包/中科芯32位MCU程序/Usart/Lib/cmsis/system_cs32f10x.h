/**
  ******************************************************************************
  * @file    system_cs32f10x.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-DEC-2017
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Header File.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. 
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup cs32f10x_system
  * @{
  */  
  
/**
  * @brief Define to prevent recursive inclusion
  */
#ifndef __SYSTEM_CS32F10X_H
#define __SYSTEM_CS32F10X_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup CS32F10x_System_Includes
  * @{
  */

/**
  * @}
  */


/** @addtogroup CS32F10x_System_Exported_types
  * @{
  */

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

/**
  * @}
  */

/** @addtogroup CS32F10x_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup CS32F10x_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup CS32F10x_System_Exported_Functions
  * @{
  */
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_CS32F10X_H */

/**
  * @}
  */
  
/**
  * @}
  */  
/******************* (C) COPYRIGHT 2017 CKS *****END OF FILE****/
