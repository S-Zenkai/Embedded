#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_systick.h"
#include "bsp_led.h"
#include "FreeRTOS_demo.h"

int main(void)
{
	USART_Config();
	SysTick_Init(72);
	LED_Init();
	FreeRTOS_demo();
}
