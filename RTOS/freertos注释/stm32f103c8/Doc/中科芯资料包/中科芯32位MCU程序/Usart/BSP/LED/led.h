#ifndef __LED__H
#define __LED__H

#define LED1_ON  GPIO_SetBits(GPIOA,GPIO_Pin_1);	//LED1	
#define LED1_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_1);		
#define LED2_ON  GPIO_SetBits(GPIOA,GPIO_Pin_2);		
#define LED2_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_2);		
#define LED3_ON  GPIO_SetBits(GPIOA,GPIO_Pin_3);		
#define LED3_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_3);		
#define LED4_ON  GPIO_SetBits(GPIOA,GPIO_Pin_4);		
#define LED4_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_4);		
void LED_Init(void);

#endif
