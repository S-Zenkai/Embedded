#include "bsp_led.h"

/**
 * @brief       LED��ʼ������(�ڲ�����)
 * @param       ��
 * @retval      ��
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
	
	/*���ֳ�ʼʱLED��*/
	LED_OFF;
}

/**
 * @brief       LED��ʼ������(�ⲿ����)
 * @param       ��
 * @retval      ��
 */
void LED_Init(void)
{
	LED_GPIO_Config();
}
