#include "stm32f10x.h"   // 相当于51单片机中的  #include <reg51.h>
#include "bsp_led.h"
#include "bsp_key.h"

void Delay( uint32_t count )
{
	for(; count!=0; count--);
}

int main(void)
{
	// 来到这里的时候，系统的时钟已经被配置成72M。
	LED_GPIO_Config();
	KEY_GPIO_Config();
	
	while(1)
	{
		if( Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN) ==KEY_ON )
			LED_G_TOGGLE;
		
		// if
	}
}

// 作业

// 把KEY2的代码也完成，按一次，LED_R_TOGGLE

