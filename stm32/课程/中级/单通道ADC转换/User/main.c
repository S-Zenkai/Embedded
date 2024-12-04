/**
 * *****************************************************************************
 * @file        main.c
 * @brief
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2024-11-28
 * @version     0.1
 * @copyright   xxxx技有限公司
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
/*----------------------------------------------------------------------------*/
uint16_t conv_valul = 0;
int main(void)
{
    ADCInit();
    OLED_Init();
    while (1)
    {
        conv_valul = GetConvValue();
        // OLED_Printf(0, 0, OLED_8X16, "转换值 = %d", conv_valul);
        OLED_ShowNum(0, 0, conv_valul, 4, OLED_8X16);
        // OLED_ShowString(72, 18, "%", OLED_8X16);
        OLED_Update();
    }
}
