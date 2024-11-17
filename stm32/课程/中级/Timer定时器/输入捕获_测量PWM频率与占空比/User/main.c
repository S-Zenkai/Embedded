/**
 * *****************************************************************************
 * @file        main.c
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2024-11-13
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
#include "bsp_pwm.h"
#include "bsp_ic.h"
/*----------------------------------------------------------------------------*/
uint16_t freq = 1;
uint8_t duty;
int main(void)
{
  TimOCInit();
  TimICInit();
  USART_Config();
  OLED_Init();
  while (1)
  {
    freq = GetFreq();
    duty = GetDuty();
    OLED_Printf(0, 0, OLED_8X16, "freq = %dHz", freq);
    OLED_Printf(0, 18, OLED_8X16, "duty = %d", duty);
    OLED_ShowString(72, 18, "%", OLED_8X16);
    OLED_Update();
  }
}


