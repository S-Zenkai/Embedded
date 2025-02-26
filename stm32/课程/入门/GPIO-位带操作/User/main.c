#include "stm32f10x.h"
#include "bsp.h"
#include "bsp_key.h"

//ODR基地址
#define GPIOB_ODR_Addr (GPIOB_BASE+0x0C)
//位带别名地址
#define GPIOB_ODR_AliasAddr(n) (*((uint32_t*)(0x42000000+(GPIOB_ODR_Addr-0x40000000)*8*4+n*4)))

//IDR基地址
#define GPIOA_IDR_Addr (GPIOA_BASE+0x08)
//位带别名地址
#define GPIOA_IDR_AliasAddr(n) *((uint32_t*)(0x42000000+(GPIOA_IDR_Addr-0x40000000)*8*4+n*4))

//IDR基地址
#define GPIOC_IDR_Addr (GPIOC_BASE+0x08)
//位带别名地址
#define GPIOC_IDR_AliasAddr(n) *((uint32_t*)(0x42000000+(GPIOC_IDR_Addr-0x40000000)*8*4+n*4))

//软件延时
void Delay(uint32_t count)
{
	for(;count > 0;count--)
		;
}

#if 0
int main(void)
{
	LED_G_Config();
//	LED_G(ON);
	
	LED_B_Config();
//	LED_B(OFF);
	
	RCC_APB2PeriphClockCmd( KEY1_GPIO_CLK|KEY2_GPIO_CLK, ENABLE);
	KEY1_Config();
	KEY2_Config();
	while(1)
	{
		if (KEY_CHECK(KEY1_GPIO_Port,KEY1_GPIO_Pin)==ON)
		{
			LED_G_TOGGLE;
		}
		if (KEY_CHECK(KEY2_GPIO_Port,KEY2_GPIO_Pin)==ON)
		{
			LED_B_TOGGLE;
		}
		
	}
}

#elseif 0
int main(void)
{
	LED_G_Config();
	//LED_B_Config();
	while(1)
	{
	  GPIOB_ODR_AliasAddr(0)=0;
		Delay(0xFFFFFF);
		GPIOB_ODR_AliasAddr(0)=1;
		Delay(0xFFFFFF);
	}
}

#else
int main(void)
{
	LED_R_Config();
	RCC_APB2PeriphClockCmd( KEY1_GPIO_CLK|KEY2_GPIO_CLK, ENABLE);
	KEY1_Config();
	KEY2_Config();
	while(1)
	{
		if(GPIOA_IDR_AliasAddr(0)==1)
		{
			while(GPIOA_IDR_AliasAddr(0)==1);
			LED_R_TOGGLE;
		}
		if(GPIOC_IDR_AliasAddr(13)==1)
		{
			//while(GPIOC_IDR_AliasAddr(13)==1);
			LED_R_TOGGLE;
		}
	}
}
#endif


