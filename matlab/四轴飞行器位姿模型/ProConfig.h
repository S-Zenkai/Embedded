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
/*TIM6定时中断*/
#define TIM6_DAC_IRQ_PRIORITY 15
#define TIM6_DAC_IRQ_SUB_PRIORITY 0
/*DMA1_Stream2_IRQ*/
#define DMA1_S2_IRQ_PRIORITY 1
#define DMA1_S2_IRQ_SUB_PRIORITY 1
/*DMA1_Stream6_IRQ*/
#define DMA1_S6_IRQ_PRIORITY 1
#define DMA1_S6_IRQ_SUB_PRIORITY 1
/*DMA2_Stream1_IRQ，用于USART6_RX，双机通信*/
#define DMA2_C5_S1_IRQ_PRIORITY 1
#define DMA2_C5_S1_IRQ_SUB_PRIORITY 1
/*DMA2_Stream6_IRQ，用于USART6_TX，双机通信*/
#define DMA2_C5_S6_IRQ_PRIORITY 1
#define DMA2_C5_S6_IRQ_SUB_PRIORITY 1

/**********************************飞机状态**********************************/
/*油门限幅,影响飞机悬停与升降快慢*/
#define THROTTLE_MAX 1200.0
/*最小油门*/
#define THROTTLE_MIN 150.0

// /*滚转*/
// #define ROLL_ANGLE_MAX 180
// /*俯仰*/
// #define PITCH_ANGLE_MAX 180
// /*偏航*/
// #define YAW_ANGLE_MAX 180

/**********************************RC配置**********************************/
/*摇杆行程最大值*/
#define RC_MAX 2000
/*摇杆最小值*/
#define RC_MIN 1000
/*中值*/
#define RC_MID 1500
/*开关行程最大值*/
#define SWITCH_MAX 2000
/*开关行程最小值*/
#define SWITCH_MIN 1000
/*开关中值*/
#define SWITCH_MID 1500
/**********************************PID模块参数**********************************/
/*PID参数*/
/*角度环*/
#define ANGLE_ROLL_KP 600.0
#define ANGLE_PITCH_KP 600.0
#define ANGLE_YAW_KP 1500.0
/*角度环只有P*/
#define ANGLE_ROLL_KI 
#define ANGLE_PITCH_KI 
#define ANGLE_YAW_KI 

#define ANGLE_ROLL_KD 
#define ANGLE_PITCH_KD 
#define ANGLE_YAW_KD 
/*角速度环*/
#define RATE_ROLL_KP 80.0
#define RATE_PITCH_KP 80.0
#define RATE_YAW_KP 9000.0

#define RATE_ROLL_KI 200.0
#define RATE_PITCH_KI 200.0
#define RATE_YAW_KI 0.0

#define RATE_ROLL_KD 2.0
#define RATE_PITCH_KD 2.0
#define RATE_YAW_KD 0.0

/*采样时间*/
#define ANG_VEL_SAMPLE_TIME 0.0001

/*yaw期望角速度灵敏度，越大飞机转的越快，无单位，非真实角速度*/
#define YAW_RATE_SENSITIVITY 672.0
/*角速度积分限幅*/
#define ANG_VEL_INTEGRAL_MAX 300.0
/*roll期望值最大值(°)*/
#define ROLL_ANGLE_TARGET_MAX 30.0
/*pitch期望值最大值(°)*/
#define PITCH_ANGLE_TARGET_MAX 30.0
/*角度(roll、pitch)比例输出限幅*/
#define ANGLE_P_OUTPUT_MAX 600.0

#endif /* __PRO_CONFIG_H */

