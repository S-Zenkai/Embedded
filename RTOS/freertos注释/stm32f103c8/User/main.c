#include "stm32f10x.h"
#include "bsp_sys.h"
#include "freertos_demo.h"
#include "bsp_systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_usart.h"


int main(void)
{
	HSE_SetSysClock(RCC_PLLMul_9);
	usart_init();
	FreeRTOS_demo();
}



