/**
  ******************************************************************************
  * @file    bsp_key.c
  * @author  
  * @version V1.0
  * @date    2024.10.17
  * @brief   ������⣨֧���Ƿ�������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-ָ���� STM32 ������ 
  *
  ******************************************************************************
  */ 
  
#include "bsp_key.h"  
#include "bsp_systick.h"

/**
  * @brief  ���ð����õ���I/O��
  * @param  ��
  * @retval ��
  */
void KEY_Init(void)
{
	GPIO_InitTypeDef KEY_GPIO_InitStruct;
	
	/*���������˿ڵ�ʱ��*/
	KEY_GPIO_APBxClock_FUN(KEY_GPIO_CLK,ENABLE);
	
	//ѡ�񰴼�������
	KEY_GPIO_InitStruct.GPIO_Pin = KEY1_GPIO_Pin; 
	// ���ð���������Ϊ��������
	KEY_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//ʹ�ýṹ���ʼ������
	GPIO_Init(KEY1_GPIO_PORT, &KEY_GPIO_InitStruct);
	
	//ѡ�񰴼�������
	KEY_GPIO_InitStruct.GPIO_Pin = KEY2_GPIO_Pin; 
	//���ð���������Ϊ��������
	KEY_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//ʹ�ýṹ���ʼ������
	GPIO_Init(KEY2_GPIO_PORT, &KEY_GPIO_InitStruct);	
}


/**
 * @brief		����ĸ����������£���ѡ���Ƿ�֧������
 * @param		�Ƿ�֧������������������ֵ��
 *					@arg	0��֧������
 *          @arg	1����֧������
 * @retval  ����x�Ƿ񱻰���
 * - 1������1������
 * - 2������2������
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


