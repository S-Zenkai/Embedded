/**
  ******************************************************************************
  * @file    bsp_usart.h
  * @author  kai
  * @version V1
  * @date    2025/2/19
  * @brief   usart��������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  * ע���F4ϵ�е�Ƭ�������Ÿ��ù��ܣ�����Ҫ���ö�Ӧ���ú���
  *
  ******************************************************************************
  */

#ifndef _BSP_USART_H
#define _BSP_USART_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
#include "stdio.h"
/*-----------------------------------macro------------------------------------*/
/*USART1�궨��*/
#define     USART7_CLK_FUN              RCC_APB1PeriphClockCmd
#define     USART7_CLK                  RCC_APB1Periph_UART7
#define     USART7_GPIO_TX              GPIOE
#define     USART7_GPIOPin_TX           GPIO_Pin_8
#define     USART7_GPIO_RX              GPIOE
#define     USART7_GPIOPin_RX           GPIO_Pin_7
/*USART2�궨��*/
#define     USART2_CLK_FUN              RCC_APB1PeriphClockCmd
#define     USART2_CLK                  RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA
#define     USART2_GPIO_TX              GPIOA
#define     USART2_GPIOPin_TX           GPIO_Pin_2
#define     USART2_GPIO_RX              GPIOA
#define     USART2_GPIOPin_RX           GPIO_Pin_3
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void USARTInit(void);
void USARTx_SendByte(USART_TypeDef *USARTx, uint16_t Data);
uint8_t USARTx_ReceiveByte(USART_TypeDef *USARTx);
void Usart_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t ch);
/*------------------------------------test------------------------------------*/


#if 0
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






// void NVIC_Config(void);
void UsartInit(void);
void SendData_Byte(USART_TypeDef* USARTx, uint8_t Data);
void SendData_TwoBytes(USART_TypeDef* USARTx, uint16_t Data);
void SendData_Char(USART_TypeDef* USARTx, char* pData);
void SendData_Arr(USART_TypeDef* USARTx, uint8_t* pData, uint8_t sz);
#endif

#endif /*_BSP_USART_H*/
