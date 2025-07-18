/**
  ******************************************************************************
  * @file    common.h
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
#ifndef __PRO_COMMON_H
#define __PRO_COMMON_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "com_type.h"
#include "stdbool.h"

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
extern uint8_t sbus_buff[SBUS_INPUT_CHANNELS]; /*多缓冲区,防止解码时数据被覆盖*/
extern uint8_t active_buff;/*接收缓冲区指针*/
extern uint16_t pwm_buff[RC_PWM_CHANNELS];

extern uint16_t rc_data[RC_PWM_CHANNELS];/*存放RC数据*/
extern bool rc_decode_done;

/**********************************DMA相关*************************************/

#if GPS_UBX_IDLE_INTERRUPT_ENABLE
/*UBX-DMA缓冲区大小*/
/*需根据实际ubx包大小修改*/
#define UBX_DMA_BUFF_SIZE 512
/*UBX-DMA双缓冲区*/
extern uint8_t ubx_dma_buff[2][UBX_DMA_BUFF_SIZE];
/*活动缓冲区*/
extern uint8_t *ubx_active_buff;
#endif

/********************************GPS/UBX相关**********************************/

/*GPS-UBX使能空闲中断接收*/
#define GPS_UBX_IDLE_INTERRUPT_ENABLE 0
/*GPS-UBX缓冲区大小*/
//#define UBX_BUFF_SIZE 1024
//extern uint8_t ubx_buff[UBX_BUFF_SIZE];
//extern ringbuff_t ubx_rb;


/*****************************************************************************/

extern Axis3i16_t mpu6000_raw_acc;
extern Axis3i16_t mpu6000_raw_gyro;
extern Axis3f_t mpu6000_cal_acc;
extern Axis3f_t mpu6000_cal_gyro;

/*****************************************************************************/
//extern u16_u8_union_t rc_data[9];
extern uint8_t safety_switch;

#endif /* __PRO_COMMON_H */
