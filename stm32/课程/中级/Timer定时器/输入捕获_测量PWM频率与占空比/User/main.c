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
#include "bsp_systick.h"
/*----------------------------------------------------------------------------*/
uint16_t freq = 1;
uint8_t duty;
int main(void)
{
  TimOCInit();
  TimICInit();
  USART_Config();
	SysTick_Init(72);
  OLED_Init();
  while (1)
  {
    freq = GetFreq();
    duty = GetDuty();
//    OLED_Printf(0, 0, OLED_8X16, "freq = %dHz", freq);
//    OLED_Printf(0, 18, OLED_8X16, "duty = %d", duty);
//    OLED_ShowString(72, 18, "%", OLED_8X16);
//    OLED_Update();
//		printf("freq = %dHz,",freq);
//		printf("duty = %d,",duty);
//		printf("Capture=%d\n",TIM_GetCapture1(TIM4));
		printf("%d,",freq);
		printf("%d,",duty);
		printf("%d\n",TIM_GetCapture1(TIM4));
		delay_ms(10);
		
  }
}


