#include "stm32f10x.h"
#include "bsp-led.h"
#include "bsp_key.h"

void Delay(uint32_t count)
{
	while(count)
	{count--;}
}

#if 0
//流水灯
int main(void)
{
	LED_G_Config();
	LED_B_Config();
	LED_R_Config();
	while(1)
	{
		LED_B_ON;
		Delay(0xFFFFFF);
		LED_B_OFF;
		Delay(0xFFFFFF);
		LED_R_ON;
		Delay(0xFFFFFF);
		LED_R_OFF;
		Delay(0xFFFFFF);
		LED_G_ON;
		Delay(0xFFFFFF);
		LED_G_OFF;
		Delay(0xFFFFFF);
		LED_B_OFF;
		LED_R_OFF;
		LED_G_OFF;
		Delay(0xFFFFFF);
		LED_B_TOGGLE;
		Delay(0xFFFFFF);
		
	}
}

#elseif 0
//按键检测
int main(void)
{
	LED_G_Config();
	LED_R_Config();
	KEY1_Config();
	KEY2_Config();
	LED_R_OFF;
	LED_G_OFF;
	while(1)
	{
	  if(GPIO_ReadInputDataBit(KEY1_Port, KEY1_Pin)==KEY1_ON)
		{
			while(GPIO_ReadInputDataBit(KEY1_Port, KEY1_Pin)==KEY1_ON);
			LED_G_TOGGLE;
		}
		if(GPIO_ReadInputDataBit(KEY2_Port, KEY2_Pin)==KEY2_ON)
		{
			while(GPIO_ReadInputDataBit(KEY2_Port, KEY2_Pin)==KEY2_ON);
			LED_R_TOGGLE;
		}
	}
}

#elseif 0
//使用位带操作实现流水灯
#define  GPIOB_ODR    (GPIOB_BASE+0x0C)
#define  GPIOB_ODR_AliasAddr(n)     *((uint32_t*)((GPIOB_ODR & 0xF0000000)+0x02000000+((GPIOB_ODR &0x00FFFFFF)<<5)+(n<<2)))
int main(void)
{
	LED_G_Config();
//	LED_B_Config();
//	LED_R_Config();
	while(1)
	{
	  GPIOB_ODR_AliasAddr(0)=1;
	  Delay(0xFFFFF);
	  GPIOB_ODR_AliasAddr(0)=0;
	  Delay(0xFFFFF);
	}
}

#else
#define  GPIOA_IDR     (GPIOA_BASE+0x08)
#define  GPIOA_IDR_AliasAddr(n)     *((uint32_t*)((GPIOA_IDR & 0xF0000000)+0x02000000+((GPIOA_IDR &0x00FFFFFF)<<5)+(n<<2)))

int main(void)
{
	LED_G_Config();
	KEY1_Config();
	while(1)
	{
		if(GPIOA_IDR_AliasAddr(0)==1)
		{
			while(GPIOA_IDR_AliasAddr(0)==1);
			LED_G_TOGGLE;
		}
		
	}
	
}


#endif




