#include "stm32f10x.h"
#include "bsp_sys.h"
#include "bsp_led.h"
#include "freertos_demo.h"
#include "bsp_systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_usart.h"
#include "bsp_key.h"


int main(void)
{
	HSE_SetSysClock(RCC_PLLMul_9);
	LED_Init();
	SysTick_Init(72);
	USART_Config();
	KEY_Init();
	FreeRTOS_demo();
}



