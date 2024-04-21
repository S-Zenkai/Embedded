/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   I/O线中断应用bsp
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./pvd/bsp_pvd.h"


 /**
  * @brief  配置 PVD
  * @param  无
  * @retval 无
  */
void PVD_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	
  NVIC_InitTypeDef NVIC_InitStructure;
	
	/*开启PWR*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);					

  
  /* 配置NVIC为优先级组1 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  
  /* 配置中断源：按键1 */
  NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
  /* 配置抢占优先级 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 配置子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断通道 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
		

	/* 选择EXTI的信号源 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line16;	
	/* EXTI为中断模式 */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 上升沿中断 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* 使能中断 */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	//3.3V引脚 的电压低于2.6V，会产生中断，其中3.3V引脚与STM32的VDD直接相连
	PWR_PVDLevelConfig(PWR_PVDLevel_2V6);
	
	PWR_PVDCmd(ENABLE);
	
	
}
/*********************************************END OF FILE**********************/
