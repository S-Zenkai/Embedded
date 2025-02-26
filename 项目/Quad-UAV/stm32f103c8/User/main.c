/**
 * *****************************************************************************
 * @file        main.c
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2025-01-23
 * @version     
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:
 * 
 * *****************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_sys.h"
#include "bsp_systick.h"

void Delay(uint32_t count)
{
	for(;count > 0;count--)
		;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
//		HSE_SetSysClock(RCC_PLLMul_2);
//    amber_init();
//    while (1)
//    {
//			GPIO_ResetBits(GPIOB, GPIO_Pin_14);
//			Delay(0xFFFFF);
//			GPIO_SetBits(GPIOB, GPIO_Pin_14);
//			Delay(0xFFFFF);
//    }
	
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	SystickInit(36);
  while (1)
  {
		delay_ms(500);
  }
}







