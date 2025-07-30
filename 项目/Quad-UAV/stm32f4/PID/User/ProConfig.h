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
/*TIM6��ʱ�ж�*/
#define TIM6_DAC_IRQ_PRIORITY 15
#define TIM6_DAC_IRQ_SUB_PRIORITY 0
/*DMA1_Stream2_IRQ*/
#define DMA1_S2_IRQ_PRIORITY 1
#define DMA1_S2_IRQ_SUB_PRIORITY 1
/*DMA1_Stream6_IRQ*/
#define DMA1_S6_IRQ_PRIORITY 1
#define DMA1_S6_IRQ_SUB_PRIORITY 1
/*DMA2_Stream1_IRQ������USART6_RX��˫��ͨ��*/
#define DMA2_C5_S1_IRQ_PRIORITY 1
#define DMA2_C5_S1_IRQ_SUB_PRIORITY 1
/*DMA2_Stream6_IRQ������USART6_TX��˫��ͨ��*/
#define DMA2_C5_S6_IRQ_PRIORITY 1
#define DMA2_C5_S6_IRQ_SUB_PRIORITY 1

/**********************************�ɻ�״̬**********************************/
/*�����޷�,Ӱ��ɻ���ͣ����������*/
#define THROTTLE_MAX 1200.0f
/*��С����*/
#define THROTTLE_MIN 150.0f

// /*��ת*/
// #define ROLL_ANGLE_MAX 180
// /*����*/
// #define PITCH_ANGLE_MAX 180
// /*ƫ��*/
// #define YAW_ANGLE_MAX 180

/**********************************RC����**********************************/
/*ҡ���г����ֵ*/
#define RC_MAX 2000
/*ҡ����Сֵ*/
#define RC_MIN 1000
/*��ֵ*/
#define RC_MID 1500
/*�����г����ֵ*/
#define SWITCH_MAX 2000
/*�����г���Сֵ*/
#define SWITCH_MIN 1000
/*������ֵ*/
#define SWITCH_MID 1500
/**********************************PIDģ�����**********************************/
/*PID����*/
/*�ǶȻ�*/
#define ANGLE_ROLL_KP 600.0f
#define ANGLE_PITCH_KP 600.0f
#define ANGLE_YAW_KP 1500.0f
/*�ǶȻ�ֻ��P*/
#define ANGLE_ROLL_KI 
#define ANGLE_PITCH_KI 
#define ANGLE_YAW_KI 

#define ANGLE_ROLL_KD 
#define ANGLE_PITCH_KD 
#define ANGLE_YAW_KD 
/*���ٶȻ�*/
#define RATE_ROLL_KP 80.0f
#define RATE_PITCH_KP 80.0f
#define RATE_YAW_KP 350.0f

#define RATE_ROLL_KI 200.0f
#define RATE_PITCH_KI 200.0f
#define RATE_YAW_KI 250.0f

#define RATE_ROLL_KD 2.0f
#define RATE_PITCH_KD 2.0f
#define RATE_YAW_KD 0.0f

/*����ʱ��*/
#define ANG_VEL_SAMPLE_TIME 0.002f

/*yaw�������ٶ������ȣ�Խ��ɻ�ת��Խ�죬�޵�λ������ʵ���ٶ�*/
#define YAW_RATE_SENSITIVITY 672.0f
/*���ٶȻ����޷�*/
#define ANG_VEL_INTEGRAL_MAX 300.0f
/*roll����ֵ���ֵ(��)*/
#define ROLL_ANGLE_TARGET_MAX 30.0f
/*pitch����ֵ���ֵ(��)*/
#define PITCH_ANGLE_TARGET_MAX 30.0f
/*�Ƕ�(roll��pitch)��������޷�*/
#define ANGLE_P_OUTPUT_MAX 600.0f

#endif /* __PRO_CONFIG_H */

