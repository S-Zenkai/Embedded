/**
  ******************************************************************************
  * @file    ProConfig.h
  * @author  
  * @version 
  * @data    2025/06/18
  * @brief   项目配置，定义了项目“应该是什么样的”，如PID参数是多少、主循环频率是多少、功能开关
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

/*SBUS使能*/
#define     SBUS_Enable             1
/*中断使能*/
#define     Interrupt_Enable        1

/*****************************主循环频率及各任务循环频率**************************************/
/*循环频率*/
#define LOOP_1000_Hz 1000
#define LOOP_500_Hz 500
#define LOOP_250_Hz 250
#define LOOP_200_Hz 200
#define LOOP_100_Hz 100
#define LOOP_50_Hz 50
#define LOOP_25_Hz 25
#define LOOP_10_Hz 10
#define LOOP_5_Hz 5
/*主循环频率*/
#define LOOP_MAIN_LOOP LOOP_1000_Hz
/*循环频率设定*/
#define LOOP_FREQ_SET(Freq,Tick) ((Tick%(LOOP_MAIN_LOOP/Freq))==0)
/**********************************中断优先级配置**********************************/
/*DMA1_Channel6_IRQ，用于USART2_RX，双机通信*/
#define DMA1_C6_IRQ_PRIORITY 1
#define DMA1_C6_IRQ_SUB_PRIORITY 1
/*DMA1_Channel7_IRQ，用于USART2_TX，双机通信*/
#define DMA1_C7_IRQ_PRIORITY 1
#define DMA1_C7_IRQ_SUB_PRIORITY 1

#endif /* __PRO_CONFIG_H */

