#include "bsp_key.h"



static void KEY_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	KEY_GPIO_APBxClock_FUN(KEY_GPIO_CLC, ENABLE);
	
	//KEY1���GPIO����
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin=KEY1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
	
	//KEY2���GPIO����
	GPIO_InitStructure.GPIO_Pin=KEY2_GPIO_PIN;
	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);
}

void KEY_Init(void)
{
	KEY_GPIO_Config();
}

/**
  *@brief  ��ȡ�ĸ�����������,�����Ƿ�֧������
  *@param  val�������Ƿ�֧�ְ�������
  *        @arg 0��֧������
  *        @arg 1:��֧������
  *@retval 0���ް���������
           1������1������
           2������2������
 */
uint8_t Get_Keyx_Status(uint8_t val)
{
	//��̬����flagֻ��ʼ��һ���Һ�������������
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



