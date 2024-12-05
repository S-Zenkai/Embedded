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
#include "OLED.h"
#include "bsp_usart.h"
#include "bsp_adc.h"
#include "bsp_dma.h"
#include "bsp_systick.h"
/*----------------------------------------------------------------------------*/
extern uint16_t DataBuffer[3];
int main(void)
{
    DMAInit();
    ADCInit();
    OLED_Init();
    SysTick_Init(72);
    while (1)
    {
        OLED_ShowString(0, 0, "ADC1:0000,+0.0V", OLED_8X16);
        OLED_ShowNum(40, 0, DataBuffer[0], 4, OLED_8X16);
        OLED_ShowFloatNum(80, 0, (double)DataBuffer[0] / 4096 * 3.3, 1, 1, OLED_8X16);
        OLED_ShowString(0, 16, "ADC2:0000,+0.0V", OLED_8X16);
        OLED_ShowNum(40, 16, DataBuffer[1], 4, OLED_8X16);
        OLED_ShowFloatNum(80, 16, (double)DataBuffer[1] / 4096 * 3.3, 1, 1, OLED_8X16);
        OLED_ShowString(0, 32, "ADC3:0000,+0.0V", OLED_8X16);
        OLED_ShowNum(40, 32, DataBuffer[2], 4, OLED_8X16);
        OLED_ShowFloatNum(80, 32, (double)DataBuffer[2] / 4096 * 3.3, 1, 1, OLED_8X16);
        OLED_Update();
        delay_ms(100);
    }
}
