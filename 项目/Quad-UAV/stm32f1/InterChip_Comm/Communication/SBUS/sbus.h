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
#include "pro_common.h"
#include <stdbool.h>
/* Exported define ------------------------------------------------------------*/

/*重新定SBUS_DEBUG函数*/
#define DEBUG_SBUS_ENABLE
#ifdef DEBUG_SBUS_ENABLE
#define SBUS_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define SBUS_DEBUG(format, ...)
#endif

// #define SBUS_RANGE_MIN 200.0f
// #define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

// #define SBUS_MAX_VALUES  8

// #define SBUS_PACKAGE_INTERVAL_TIME   3  /*ms*/
/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGR_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))


#define PWM_Scale_Factor                (PWM_TARGRT_MAX-PWM_TARGRT_MIN)/(SBUS_RANGR_MAX-SBUS_RANGE_MIN)
/* Exported types ------------------------------------------------------------*/
/* Exported contants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
bool sbus_decode(uint8_t* decode,uint16_t* pwm,uint8_t channel);
void sbus_init(void);
void get_rc_data(uint16_t *pwm, uint8_t channel);
#endif /* __SBUS_H */
 
