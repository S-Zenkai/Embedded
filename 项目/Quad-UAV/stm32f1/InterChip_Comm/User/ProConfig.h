/**
  ******************************************************************************
  * @file    ProConfig.h
  * @author  
  * @version 
  * @data    2025/06/18
  * @brief   ��Ŀ���ã���������Ŀ��Ӧ����ʲô���ġ�����PID�����Ƕ��١���ѭ��Ƶ���Ƕ��١����ܿ���
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRO_CONFIG_H
#define __PRO_CONFIG_H

/*SBUSʹ��*/
#define     SBUS_Enable             1
/*�ж�ʹ��*/
#define     Interrupt_Enable        1

/*****************************��ѭ��Ƶ�ʼ�������ѭ��Ƶ��**************************************/
/*ѭ��Ƶ��*/
#define LOOP_1000_Hz 1000
#define LOOP_500_Hz 500
#define LOOP_250_Hz 250
#define LOOP_200_Hz 200
#define LOOP_100_Hz 100
#define LOOP_50_Hz 50
#define LOOP_25_Hz 25
#define LOOP_10_Hz 10
#define LOOP_5_Hz 5
/*��ѭ��Ƶ��*/
#define LOOP_MAIN_LOOP LOOP_1000_Hz
/*ѭ��Ƶ���趨*/
#define LOOP_FREQ_SET(Freq,Tick) ((Tick%(LOOP_MAIN_LOOP/Freq))==0)
/**********************************�ж����ȼ�����**********************************/
/*DMA1_Channel6_IRQ������USART2_RX��˫��ͨ��*/
#define DMA1_C6_IRQ_PRIORITY 1
#define DMA1_C6_IRQ_SUB_PRIORITY 1
/*DMA1_Channel7_IRQ������USART2_TX��˫��ͨ��*/
#define DMA1_C7_IRQ_PRIORITY 1
#define DMA1_C7_IRQ_SUB_PRIORITY 1

#endif /* __PRO_CONFIG_H */

