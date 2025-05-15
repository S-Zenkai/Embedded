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
#include "stdbool.h"
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
void usart_init(void);
void USART4_SetBaudRate( uint32_t baudrate);
bool USART_SendBytes(USART_TypeDef *USARTx, uint8_t *Data, uint32_t length, uint32_t timeout);
bool USART_ReceiveBytes(USART_TypeDef *USARTx, uint8_t *Data, uint32_t length, uint32_t timeout);
/*------------------------------------test------------------------------------*/

#endif /*_BSP_USART_H*/
