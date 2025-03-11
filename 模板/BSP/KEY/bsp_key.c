/**
  ******************************************************************************
  * @file    bsp_key.c
  * @author  
  * @version V1.0
  * @date    2024.10.17
  * @brief   按键检测（支持是否连按）
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 F103-指南者 STM32 开发板 
  *
  ******************************************************************************
  */ 
  
#include "bsp_key.h"  
#include "bsp_systick.h"

/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
void KEY_Init(void)
{
	GPIO_InitTypeDef KEY_GPIO_InitStruct;
	
	/*开启按键端口的时钟*/
	KEY_GPIO_APBxClock_FUN(KEY_GPIO_CLK,ENABLE);
	
	//选择按键的引脚
	KEY_GPIO_InitStruct.GPIO_Pin = KEY1_GPIO_Pin; 
	// 设置按键的引脚为浮空输入
	KEY_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//使用结构体初始化按键
	GPIO_Init(KEY1_GPIO_PORT, &KEY_GPIO_InitStruct);
	
	//选择按键的引脚
	KEY_GPIO_InitStruct.GPIO_Pin = KEY2_GPIO_Pin; 
	//设置按键的引脚为浮空输入
	KEY_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//使用结构体初始化按键
	GPIO_Init(KEY2_GPIO_PORT, &KEY_GPIO_InitStruct);	
}


/**
 * @brief		检测哪个按键被按下，可选择是否支持连按
 * @param		是否支持连按，可以是以下值：
 *					@arg	0：支持连按
 *          @arg	1：不支持连按
 * @retval  按键x是否被按下
 * - 1：按键1被按下
 * - 2：按键2被按下
 */
uint8_t Key_CheckPress(uint8_t val)
{
	static  uint8_t flag=1;
	if(flag==1&&(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_Pin)==1||GPIO_ReadInputDataBit(KEY2_GPIO_PORT,KEY2_GPIO_Pin)==1))
	{
		if(val==0)
			flag=0;
		if(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_Pin)==1)
			return 1;
		if(GPIO_ReadInputDataBit(KEY2_GPIO_PORT,KEY2_GPIO_Pin)==1)
			return 2;
	}
	if(!(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_Pin)==1||GPIO_ReadInputDataBit(KEY2_GPIO_PORT,KEY2_GPIO_Pin)==1))
	{
		flag=1;
		return 0;
	}
	return 0;
}

uint8_t Key_CheckPress(uint8_t val)
{
	static  uint8_t flag=1;
	if(flag==1&&(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_Pin)==1||
	GPIO_ReadInputDataBit(KEY2_GPIO_PORT,KEY2_GPIO_Pin)==1||
	GPIO_ReadInputDataBit(KEY3_GPIO_PORT, KEY3_GPIO_Pin)==1
	||GPIO_ReadInputDataBit(KEY4_GPIO_PORT,KEY4_GPIO_Pin)==1))
	{
		if(val==0)
			flag=0;
		if(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_Pin)==1)
			return 1;
		if(GPIO_ReadInputDataBit(KEY2_GPIO_PORT,KEY2_GPIO_Pin)==1)
			return 2;
			if(GPIO_ReadInputDataBit(KEY3_GPIO_PORT, KEY3_GPIO_Pin)==1)
			return 3;
		if(GPIO_ReadInputDataBit(KEY4_GPIO_PORT,KEY4_GPIO_Pin)==1)
			return 4;
	}
	if(!(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_Pin)==1||
	GPIO_ReadInputDataBit(KEY2_GPIO_PORT,KEY2_GPIO_Pin)==1||
	GPIO_ReadInputDataBit(KEY3_GPIO_PORT, KEY3_GPIO_Pin)==1
	||GPIO_ReadInputDataBit(KEY4_GPIO_PORT,KEY4_GPIO_Pin)==1))
	{
		flag=1;
		return 0;
	}
	return 0;
}


