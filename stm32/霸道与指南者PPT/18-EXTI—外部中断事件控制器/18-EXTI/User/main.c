#include "stm32f10x.h"   // 相当于51单片机中的  #include <reg51.h>
#include "bsp_led.h"

#include "bs
p_exti.h"

void Delay( uint32_t count )
{
	for(; count!=0; count--);
}

int main(void)
{
	// 来到这里的时候，系统的时钟已经被配置成72M。
	LED_GPIO_Config();
	
	EXIT_Key_Config();

	while(1)
	{

	}
}

// 作业

// 1- 把KEY2-PC13 也用 EXTI 的方式实现

// 2- 如果 PA0 PB0 PC0 都连接到 EXTI0 的时候怎么办？
// GPIOA->IDR  GPIOB->IDR 

// 3- 把跟硬件相关的GPIO都通过宏定义来实现