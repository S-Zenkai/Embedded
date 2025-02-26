#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_usart.h"
#include "bsp_sys.h"
#include "bsp_systick.h"
#include "FreeRTOS_demo.h"

int main(void)
{
	LED_Init();
	KEY_Init();
	USART_Config();
	ExtiInit();
	SysTick_Init(72);
	FreeRTOS_demo();
}
