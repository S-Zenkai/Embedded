#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp.h"
#if 1
int main(void)
{
//	extern uint16_t ret;
	extern uint8_t data;
	USART_Config();
//	printf("%d",ret);
	printf("abc");
	while(1)
	{
//		printf("%d",data);
	}
}

#else

int main(void)
{
	extern uint8_t ret;
	USART_Config();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	LED_G_Config();
	LED_B_Config();
	LED_R_Config();
	while(1)
	{
		switch (ret)
		{
			case 1:
				LED_OFF;
				LED_R(ON);
			  break;
			case '20':
				LED_OFF;
			  LED_G(ON);
				break;
			case 3:
				LED_OFF;
			  LED_B(ON);
				break;
			default:
				LED_OFF;
		}
	}
}

#endif


