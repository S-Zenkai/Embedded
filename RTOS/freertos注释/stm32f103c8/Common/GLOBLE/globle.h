/**
  ******************************************************************************
  * @file    globle.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/04/03
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBLE_H
#define __GLOBLE_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/*******************************sbus相关****************************************/
/*sbus信号范围*/
#define SBUS_RANGE_MIN                  341.0f
#define SBUS_RANGR_MAX                  1706.0f
/*目标pwm范围*/
#define PWM_TARGRT_MIN                  1000.0f
#define PWM_TARGRT_MAX                  2000.0f
/*sbus协议8位通道数*/
#define SBUS_INPUT_CHANNELS             25
/*解码后的sbus协议11位通道数*/
#define SBUS_PWM_CHANNELS               16
/*遥控器支持通道*/
#define RC_PWM_CHANNELS                 9

extern uint8_t sbus_DF_TC;/*一个数据帧(data frame)传输完成标志*/
extern uint8_t sbus_buff[2][SBUS_INPUT_CHANNELS]; /*多缓冲区,防止解码时数据被覆盖*/
extern uint8_t active_buff;/*接收缓冲区指针*/
extern uint16_t pwm_buff[RC_PWM_CHANNELS];

#endif /* __GLOBLE_H */
