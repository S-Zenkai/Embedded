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
#include "stm32f4xx.h"


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

#endif /* __GLOBLE_H */
