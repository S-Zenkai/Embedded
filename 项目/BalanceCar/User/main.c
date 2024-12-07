/**
 * *****************************************************************************
 * @file        main.c
 * @brief       平衡小车主函数
 * @author
 * @date        2024-11-22
 * @version     0.1
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * 实验平台:
 *
 * *****************************************************************************
 */
#include "stm32f10x.h"
#include "bsp_tb6612.h"
#include "oled.h"
#include "bsp_encoder.h"
#include "bsp_systick.h"
int main(void)
{
    OLED_Init();
    TB6612Init();
    EncoderInit();
    SystickInit(72);
    SetPWMADuty(7200 * 2);
    while (1)
    {
        delay_ms(1000);
        // OLED_ShowSignedNum(0, 0, GetSpeed(), 5, OLED_8X16);
        OLED_ShowFloatNum(0, 0, GetSpeed(), 5, 5, OLED_8X16);
        OLED_Update();
    }
}
