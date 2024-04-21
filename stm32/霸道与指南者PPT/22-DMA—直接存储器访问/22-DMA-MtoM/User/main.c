/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   测试led
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_dma_mtm.h"

extern const uint32_t aSRC_Const_Buffer[BUFFER_SIZE];
extern uint32_t aDST_Buffer[BUFFER_SIZE];

#define SOFT_DELAY Delay(0x0FFFFF);

void Delay(__IO u32 nCount); 

/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{	
	uint8_t status=0;
	
	/* LED 端口初始化 */
	LED_GPIO_Config();
	LED_YELLOW;
	Delay(0xFFFFFF);
	
  MtM_DMA_Config();
	
	while( DMA_GetFlagStatus(MTM_DMA_FLAG_TC) == RESET );
	
	status = Buffercmp(aSRC_Const_Buffer,aDST_Buffer,BUFFER_SIZE);
	
	if( status == 0 )
  {
		LED_RED;
  }
  else
  {
		LED_GREEN;
	}
	
	while (1)
	{
	
	}
}

void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/
