#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_sys.h"
#include "bsp_systick.h"
#include "bsp_btim.h"
#include "FreeRTOS_demo.h"

int main(void)
{
	USART_Config();
	TIM_Init();
	SysTick_Init(72);
	FreeRTOS_demo();
}
