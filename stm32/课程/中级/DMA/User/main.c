//#include "stm32f10x.h"
#include "dma_m2p.h"
#include "bsp_led.h"

void Delay(uint32_t count)
{
	for(;count > 0;count--)
		;
}
int main(void)
{
//	extern const uint32_t Asour[sz];
//	extern uint32_t Adest[sz];
	extern uint8_t Asour[sz];
	uint32_t i=0;
	for(i=0;i<sz;i++)
	{
		Asour[i]=i;
	}
	DMA_M2P_Config();
//����Ҫȥ����һ�д��룬��������������ѭ����ӦΪ������DMA���仹û����ɣ�������M2M��һ��
//	while(RESET==DMA_GetFlagStatus(DMA_M2M_TCFlag));
	USART_Config();
	USART_DMACmd(Debug_USART, USART_DMAReq_Tx, ENABLE);
//	printf("%d",Asour[3]);
	while(1);
//	LED_Config();
//  LED_Y_ON;
//	Delay(0xFFFFFF);
//	LED_Y_OFF;
//	Delay(0xFFFFFF);
//	while(1)
//	{
//		if(flag==1)
//		{
//	    LED_G_ON;
//		  Delay(0xFFFFFF);
//		  LED_G_OFF;
//		  Delay(0xFFFFFF);
//		}
//		else
//		{
//	    LED_R_ON;
//		  Delay(0xFFFFFF);
//		  LED_R_OFF;
//		  Delay(0xFFFFFF);
//		}
//	}
}

