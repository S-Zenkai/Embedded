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
#include "bsp_systick.h"
#include "bsp_encoder.h"
/*----------------------------------------------------------------------------*/
uint16_t freq = 1;
uint8_t duty;
int main(void)
{
  USART_Config();
  OLED_Init();
  EncoderInit();
  SysTick_Init(72);
  while (1)
  {
      OLED_ShowSignedNum(1, 1, GetSpeed(), 5, OLED_8X16);
      OLED_ShowString(72, 18, "%", OLED_8X16);
      OLED_Update();
      delay_ms(1000);
  }
}


