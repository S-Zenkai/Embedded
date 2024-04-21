 /**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SRAM测试
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
#include "./usart/bsp_usart.h"
#include "./led/bsp_led.h"
#include "./sram/bsp_sram.h"

//1.使用指针对SRAM进行读写
//2.使用绝对地址的方式访问SRAM

//使用__attribute__定义变量时，需要定义为全局变量
uint8_t testValue __attribute__ ((at (SRAM_BASE_ADDR+0x40))) ;




/**
  * @brief  测试SRAM是否正常 
  * @param  None
  * @retval 正常返回1，异常返回0
  */
uint8_t SRAM_Test(void)
{
  /*写入数据计数器*/
  uint32_t counter=0;
  
  /* 8位的数据 */
  uint8_t ubWritedata_8b = 0, ubReaddata_8b = 0;  
  
  /* 16位的数据 */
  uint16_t uhWritedata_16b = 0, uhReaddata_16b = 0; 
  
  printf("正在检测SRAM，以8位、16位的方式读写sdram...");


  /*按8位格式读写数据，并校验*/
  
  /* 把SRAM数据全部重置为0 ，IS62WV51216_SIZE是以8位为单位的 */
  for (counter = 0x00; counter < SRAM_SIZE; counter++)
  {
    *(__IO uint8_t*) (SRAM_BASE_ADDR + counter) = (uint8_t)0x0;
  }
  
  /* 向整个SRAM写入数据  8位 */
  for (counter = 0; counter < SRAM_SIZE; counter++)
  {
    *(__IO uint8_t*) (SRAM_BASE_ADDR + counter) = (uint8_t)(ubWritedata_8b + counter);
  }
  
  /* 读取 SRAM 数据并检测*/
  for(counter = 0; counter<SRAM_SIZE;counter++ )
  {
    ubReaddata_8b = *(__IO uint8_t*)(SRAM_BASE_ADDR + counter);  //从该地址读出数据
    
    if(ubReaddata_8b != (uint8_t)(ubWritedata_8b + counter))      //检测数据，若不相等，跳出函数,返回检测失败结果。
    {
      printf("8位数据读写错误！");
      return 0;
    }
  }
	
  
  /*按16位格式读写数据，并检测*/
  
  /* 把SRAM数据全部重置为0 */
  for (counter = 0x00; counter < SRAM_SIZE/2; counter++)
  {
    *(__IO uint16_t*) (SRAM_BASE_ADDR + 2*counter) = (uint16_t)0x00;
  }
  
  /* 向整个SRAM写入数据  16位 */
  for (counter = 0; counter < SRAM_SIZE/2; counter++)
  {
    *(__IO uint16_t*) (SRAM_BASE_ADDR + 2*counter) = (uint16_t)(uhWritedata_16b + counter);
  }
  
    /* 读取 SRAM 数据并检测*/
  for(counter = 0; counter<SRAM_SIZE/2;counter++ )
  {
    uhReaddata_16b = *(__IO uint16_t*)(SRAM_BASE_ADDR + 2*counter);  //从该地址读出数据
    
    if(uhReaddata_16b != (uint16_t)(uhWritedata_16b + counter))      //检测数据，若不相等，跳出函数,返回检测失败结果。
    {
      printf("16位数据读写错误！");

      return 0;
    }
  }
  
  printf("SRAM读写测试正常！"); 
  /*检测正常，return 1 */
  return 1;

}


/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
int main(void)
{ 	
	uint8_t *p; 
	uint16_t *p16; 
	float *pf; 
//	uint8_t temp;
	
	LED_GPIO_Config();
	LED_BLUE;
	
	/* 配置串口为：115200 8-N-1 */
	USART_Config();
	printf("\r\n 这是一个SRAM测试实验 \r\n");	
	SRAM_Init();
	
	//把数据强制转换成指针
	p = (uint8_t *)SRAM_BASE_ADDR; 
	
	*p = 0xAB;	
	
	
//	temp = *p;
	
	printf("\r\n读出的数据内容为：0x%x\r\n",*p);
	
	p16 = (uint16_t *)(SRAM_BASE_ADDR+10); 
	
	*p16 = 0xcdef;
	
	printf("\r\n读出的数据内容为：0x%x\r\n",*p16);
	
	
	pf = (float *)(SRAM_BASE_ADDR+20); 
	
	*pf = 56.35;
	
	printf("\r\n读出的数据内容为：0x%.2f\r\n",*pf);

	testValue = 50;
	
	printf("testValue =%d,testValue addr = 0x%x ",testValue,(unsigned int)&testValue);
	
	
	SRAM_Test();
	
	while(1);  
}





/*********************************************END OF FILE**********************/
