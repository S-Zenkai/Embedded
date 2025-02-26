#include "bsp_key.h"



static void KEY_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	KEY_GPIO_APBxClock_FUN(KEY_GPIO_CLC, ENABLE);
	
	//KEY1相关GPIO配置
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin=KEY1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
	
	//KEY2相关GPIO配置
	GPIO_InitStructure.GPIO_Pin=KEY2_GPIO_PIN;
	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);
}

void KEY_Init(void)
{
	KEY_GPIO_Config();
}

/**
  *@brief  获取哪个按键被按下,控制是否支持连按
  *@param  val：控制是否支持按键连按
  *        @arg 0：支持连按
  *        @arg 1:不支持连按
  *@retval 0：无按键被按下
           1：按键1被按下
           2：按键2被按下
 */
uint8_t Get_Keyx_Status(uint8_t val)
{
	//静态变量flag只初始化一次且函数结束不销毁
	static uint8_t flag=1;
	if(flag==1&&(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN)||GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN)))
	{
		if(val==0)
			flag=0;
		if(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN))
			return 1;
		if(GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN))
			return 2;
	}
	if(!(GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN)||GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN)))
	{
		flag=1;
		return 0;
	}
	return 0;
}



