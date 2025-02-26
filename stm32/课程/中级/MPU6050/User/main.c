/**
 * *****************************************************************************
 * @file        main.c
 * @brief       独立模式多通道ADC实验，使用DMA转移规则组数据寄存器数据
 * @author
 * @date        2024-12-05
 * @version
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * 实验平台:
 *
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "oled.h"



/*----------------------------------------------------------------------------*/
int main(void)
{
		RCC_ClocksTypeDef RCC_Clocks;
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Init(72);
	OLED_Init();
	OLED_ShowString(0, 0, "motor_speed=", OLED_6X8);
	OLED_Update();
    while (1)
    {
        delay_ms(500);
    }
}
