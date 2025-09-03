/**
  ******************************************************************************
  * @file    pro_common.h
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
#include "stm32f4xx.h"
#include "com_type.h"

/*�޷�����*/
#define LIMIT(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))


/*�Ƕ�ת����*/
#define PI 3.1415926f
#define ANGLE_TO_RADIAN (PI/180.0f)
/*����ת�Ƕ�*/
#define RADIAN_TO_ANGLE (180.0f/PI)



/**********************************DMA���*************************************/

#if GPS_UBX_IDLE_INTERRUPT_ENABLE
/*UBX-DMA��������С*/
/*�����ʵ��ubx����С�޸�*/
#define UBX_DMA_BUFF_SIZE 512
/*UBX-DMA˫������*/
extern uint8_t ubx_dma_buff[2][UBX_DMA_BUFF_SIZE];
/*�������*/
extern uint8_t *ubx_active_buff;
#endif

/********************************GPS/UBX���**********************************/

/*GPS-UBXʹ�ܿ����жϽ���*/
#define GPS_UBX_IDLE_INTERRUPT_ENABLE 0
/*GPS-UBX��������С*/
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
extern uint8_t safety_switch;





#endif /* __PRO_COMMON_H */
