#ifndef _BSP_USART_H
#define _BSP_USART_H
#include "stm32f10x.h"
#include "stdio.h"

#define Debuh_USART1 1
#define Debuh_USART2 0
#define Debuh_USART3 0
#define Debuh_USART4 0
#define Debuh_USART5 0

//USART1
#if Debuh_USART1
#define    Debug_USART_GPIO           GPIOA
#define    Debug_USART_GPIO_TXPin     GPIO_Pin_9
#define    Debug_USART_GPIO_RXPin     GPIO_Pin_10
#define    Debug_USART_GPIO_CLC       RCC_APB2Periph_GPIOA
#define    Debug_USART                USART1
#define    Debug_USART_CLC            RCC_APB2Periph_USART1
#define    Debug_USART_BaudRate       115200
#define    Debug_USART_IRQn           USART1_IRQn
#define    Debug_USART_IRQHandler     USART1_IRQHandler
#define    Debug_RCC_PeriphClockCmd     RCC_APB2PeriphClockCmd
#endif

//USART2
#if Debuh_USART2
#define    Debug_USART_GPIO           GPIOA
#define    Debug_USART_GPIO_TXPin     GPIO_Pin_2
#define    Debug_USART_GPIO_RXPin     GPIO_Pin_3
#define    Debug_USART_GPIO_CLC       RCC_APB2Periph_GPIOA
#define    Debug_USART                USART2
#define    Debug_USART_CLC            RCC_APB1Periph_USART2
#define    Debug_USART_BaudRate       115200
#define    Debug_USART_IRQn           USART2_IRQn
#define    Debug_USART_IRQHandler     USART2_IRQHandler
#define    Debug_RCC_PeriphClockCmd     RCC_APB1PeriphClockCmd
#endif






void NVIC_Config(void);
void USART_Config(void);
void SendData_Byte(USART_TypeDef* USARTx, uint8_t Data);
void SendData_TwoBytes(USART_TypeDef* USARTx, uint16_t Data);
void SendData_Char(USART_TypeDef* USARTx, char* pData);
void SendData_Arr(USART_TypeDef* USARTx, uint8_t* pData, uint8_t sz);

#endif /*_BSP_USART_H*/


