/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   串口中断接收测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"
#include <stdlib.h>


uint32_t testValue =129;

uint8_t testGrup[20]={45,65,78,54,121};


#define __EXRAM  __attribute__((section("EXRAM")))

//__attribute__((section("EXRAM")))
uint8_t EXtestGrup[1024] __attribute__((section("EXRAM"))) ={43,32,3};

uint8_t EXtestGrup2[1024] __EXRAM ={56,56,65,11,};

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
	
	uint8_t inerTestValue=19;
	uint8_t *p;
	
	
	testValue ++;
	testGrup[0]++;
	EXtestGrup[0]++;
	EXtestGrup2[0]++;
	inerTestValue++;
	
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  USART_Config();
	
		
	printf("欢迎使用秉火STM32开发板\n\n\n\n");
	
	p = (uint8_t *)malloc(20);
	
	*p = 50;
	
	
	printf("\r\ntestValue值：%d,地址：0x%x",testValue,(uint32_t)&testValue);
	
	printf("\r\ntestGrup值：%d,地址：0x%x",testGrup[0],(uint32_t)&testGrup);
	
	printf("\r\ninerTestValue值：%d,地址：0x%x",inerTestValue,(uint32_t)&inerTestValue);
	
	printf("\r\nEXtestGrup值：%d,地址：0x%x",EXtestGrup[0],(uint32_t)&EXtestGrup);

	printf("\r\nEXtestGrup2值：%d,地址：0x%x",EXtestGrup2[0],(uint32_t)&EXtestGrup2);
	
	printf("\r\nmalloc 空间存储的值：%d,地址：0x%x",*p,(uint32_t)p);


  while(1)
	{	
		
	}	
}
/*********************************************END OF FILE**********************/
