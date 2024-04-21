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
#include "bsp_dma_mtp.h"

extern uint8_t SendBuff[SENDBUFF_SIZE];

#define SOFT_DELAY Delay(0x0FFFFF);

void Delay(__IO u32 nCount); 

/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{	
	uint16_t i=0;
	/* LED 端口初始化 */
	LED_GPIO_Config();
	
	USART_Config();
	
	for(i=0; i<SENDBUFF_SIZE; i++)
	{
		SendBuff[i] = 'P';
	}
	
  USARTx_DMA_Config();
	
	USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Tx, ENABLE);
	
	while (1)
	{
    LED1_TOGGLE
    Delay(0xFFFFF);		
	}
}

void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/


















