/*********************************************************************************************************
*
* File                : main.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, CKS
*                                       http://www.cksic.com
*                                          All Rights Reserved
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "cs32f10x.h"
#include "systick.h"
#include <stdio.h>
#include "usart.h"
#include "led.h" 

/* Private function prototypes -----------------------------------------------*/
void USART_Configuration(void);
 

/* Private variables ---------------------------------------------------------*/

int main(void)
{
  u8 rec_cnt;
	u8 i;
	u32 cnt=0;
	Delay_Init();//延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组
	LED_Init();//LED状态指示灯初始化
	USART_Configuration();//串口初始化
  while (1)
  {
		cnt++;
		if(Rec.state ==2)
		{
			Rec.state = 0;
		  rec_cnt = Rec.cnt;
			Rec.cnt = 0;
		
			printf("\r\n您发送的信息为：\r\n");
			for(i=0;i<rec_cnt;i++)
			{
				USART_SendData(USART1, Rec.buf[i]);//向串口1发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			} 			
		}
		else
		{
			Delay(100);//2秒	
			if(cnt%20==0)//2秒
			{	
				printf("\r\n中科芯  CKS串口实验，请输入测试信息：");
			}
		}
    
		if(cnt%2 ==0)
		{
			LED1_OFF;
		}
		else
		{
			LED1_ON;
		}
	}
}
 



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
