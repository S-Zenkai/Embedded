/**
  ******************************************************************************
  * @file    sbus.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/25
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SBUS_H
#define __SBUS_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "globle.h"
#include <stdbool.h>
/* Exported define ------------------------------------------------------------*/


#define PWM_Scale_Factor                (PWM_TARGRT_MAX-PWM_TARGRT_MIN)/(SBUS_RANGR_MAX-SBUS_RANGE_MIN)
/* Exported types ------------------------------------------------------------*/
/* Exported contants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
bool sbus_decode(uint8_t* decode,uint16_t* pwm,uint8_t channel);
void sbus_init(void);
#endif /* __SBUS_H */
 
