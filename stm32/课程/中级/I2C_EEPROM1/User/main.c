#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp.h"
#if 1
int main(void)
{
	extern uint16_t ret;
	USART_Config();
	printf("%d",ret);
	printf("abc");
		//uint16_t ret=1;
//	uint8_t sz=0;
//	uint8_t arr[10]={0,1,2,3,4,5,6,7,8,9};
//	sz=sizeof(arr)/sizeof(arr[0]);
	  //USART_Config();
	//SendData_Byte(Debug_USART,1);
	//SendData_TwoBytes(Debug_USART,0x368F);
	//SendData_Char(Debug_USART, "hallow word\n");
	//SendData_Arr(Debug_USART, arr, sz);
	//printf("%s\n","hellow word");
//	  ret=getchar();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	LED_G_Config();
	LED_B_Config();
	LED_R_Config();
	//LED_R(ON);
	while(1)
	{
//		scanf("%s",&ret);
		//ret=getchar();
//		printf("%c\n",ret);
//		switch (ret)
//		{
//			case 1:
//				LED_OFF;
//				LED_R(ON);
//			  break;
//			case 2:
//				LED_OFF;
//			  LED_G(ON);
//				break;
//			case 3:
//				LED_OFF;
//			  LED_B(ON);
//				break;
//			default:
//				LED_OFF;
//		}
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


