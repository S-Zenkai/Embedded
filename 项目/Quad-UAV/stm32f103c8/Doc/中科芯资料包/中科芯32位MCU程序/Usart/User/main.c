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
	Delay_Init();//��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ���
	LED_Init();//LED״ָ̬ʾ�Ƴ�ʼ��
	USART_Configuration();//���ڳ�ʼ��
  while (1)
  {
		cnt++;
		if(Rec.state ==2)
		{
			Rec.state = 0;
		  rec_cnt = Rec.cnt;
			Rec.cnt = 0;
		
			printf("\r\n�����͵���ϢΪ��\r\n");
			for(i=0;i<rec_cnt;i++)
			{
				USART_SendData(USART1, Rec.buf[i]);//�򴮿�1��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			} 			
		}
		else
		{
			Delay(100);//2��	
			if(cnt%20==0)//2��
			{	
				printf("\r\n�п�о  CKS����ʵ�飬�����������Ϣ��");
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
