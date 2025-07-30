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
//#include "stm32f4xx.h"
//#include "com_type.h"

/*限幅函数*/
#define LIMIT(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))


/*角度转弧度*/
#define PI 3.1415926
#define ANGLE_TO_RADIAN (PI/180.0)
/*弧度转角度*/
#define RADIAN_TO_ANGLE (180.0/PI)



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

// extern Axis3i16_t mpu6000_raw_acc;
// extern Axis3i16_t mpu6000_raw_gyro;
// extern Axis3f_t mpu6000_cal_acc;
// extern Axis3f_t mpu6000_cal_gyro;

/*****************************************************************************/
//extern u16_u8_union_t rc_data[9];
// extern uint8_t safety_switch;



#endif /* __PRO_COMMON_H */
