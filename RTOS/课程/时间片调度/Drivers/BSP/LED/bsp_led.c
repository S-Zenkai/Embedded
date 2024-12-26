#include "bsp_led.h"

/**
 * @brief       LED初始化函数(内部调用)
 * @param       无
 * @retval      无
 */
static void LED_GPIO_Config(void)
{
	GPIO_InitTypeDef LED_GPIO_InitStruct;
	LED_GPIO_APBxClock_FUN(LED_GPIO_CLK,ENABLE);
	
	/*LEDR*/
	LED_GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	LED_GPIO_InitStruct.GPIO_Pin=LED_R_GPIO_Pin;
	LED_GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(LED_R_GPIO_PORT, &LED_GPIO_InitStruct);
	
	/*LEDG*/
	LED_GPIO_InitStruct.GPIO_Pin=LED_G_GPIO_Pin;
	GPIO_Init(LED_G_GPIO_PORT, &LED_GPIO_InitStruct);
	
	/*LEDB*/
	LED_GPIO_InitStruct.GPIO_Pin=LED_B_GPIO_Pin;
	GPIO_Init(LED_B_GPIO_PORT, &LED_GPIO_InitStruct);
	
	/*保持初始时LED关*/
	LED_OFF;
}

/**
 * @brief       LED初始化函数(外部调用)
 * @param       无
 * @retval      无
 */
void LED_Init(void)
{
	LED_GPIO_Config();
}
