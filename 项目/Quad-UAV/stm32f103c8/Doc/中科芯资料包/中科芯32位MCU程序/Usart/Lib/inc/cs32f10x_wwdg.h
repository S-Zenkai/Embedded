/**
  ******************************************************************************
  * @file    cs32f10x_wwdg.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-DEC-2011
  * @brief   This file contains all the functions prototypes for the WWDG firmware
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CS32F10x_WWDG_H
#define __CS32F10x_WWDG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cs32f10x.h"

/** @addtogroup CS32F10x_StdPeriph_Driver
  * @{
  */

/** @addtogroup WWDG
  * @{
  */ 

/** @defgroup WWDG_Exported_Types
  * @{
  */ 
  
/**
  * @}
  */ 

/** @defgroup WWDG_Exported_Constants
  * @{
  */ 
  
/** @defgroup WWDG_Prescaler 
  * @{
  */ 
  
#define WWDG_Prescaler_1    ((uint32_t)0x00000000)
#define WWDG_Prescaler_2    ((uint32_t)0x00000080)
#define WWDG_Prescaler_4    ((uint32_t)0x00000100)
#define WWDG_Prescaler_8    ((uint32_t)0x00000180)
#define IS_WWDG_PRESCALER(PRESCALER) (((PRESCALER) == WWDG_Prescaler_1) || \
                                      ((PRESCALER) == WWDG_Prescaler_2) || \
                                      ((PRESCALER) == WWDG_Prescaler_4) || \
                                      ((PRESCALER) == WWDG_Prescaler_8))
#define IS_WWDG_WINDOW_VALUE(VALUE) ((VALUE) <= 0x7F)
#define IS_WWDG_COUNTER(COUNTER) (((COUNTER) >= 0x40) && ((COUNTER) <= 0x7F))

/**
  * @}
  */ 

/**
  * @}
  */ 

/** @defgroup WWDG_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup WWDG_Exported_Functions
  * @{
  */ 
  
void WWDG_DeInit(void);
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);
void WWDG_Enable(uint8_t Counter);
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __CS32F10x_WWDG_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2017 CKS *****END OF FILE****/
