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
/*UART7�궨��*/
#define     DEBUGE_USART                UART7
#define     DEBUG_USART_CLK_FUN         RCC_APB1PeriphClockCmd
#define     DEBUG_USART_CLK             RCC_APB1Periph_UART7

#define     DEBUG_USART_GPIO_CLK_FUN    RCC_AHB1PeriphClockCmd
#define     DEBUG_USART_GPIO_CLK        RCC_AHB1Periph_GPIOE

#define     DEBUG_USART_TX_PORT         GPIOE
#define     DEBUG_USART_TX_Pin          GPIO_Pin_8
#define     DEBUG_USART_RX_PORT         GPIOE
#define     DEBUG_USART_RX_Pin          GPIO_Pin_7

#define     DEBUG_USART_AF_PinSource_TX      GPIO_PinSource8
#define     DEBUG_USART_AF_PinSource_RX      GPIO_PinSource7
#define     DEBUG_USART_AF                  GPIO_AF_UART7


/*UART8�궨��*/
#define     MAVLINK_USART                 UART8
#define     MAVLINK_USART_CLK_FUN         RCC_APB1PeriphClockCmd
#define     MAVLINK_USART_CLK             RCC_APB1Periph_UART8

#define     MAVLINK_USART_GPIO_CLK_FUN    RCC_AHB1PeriphClockCmd
#define     MAVLINK_USART_GPIO_CLK        RCC_AHB1Periph_GPIOE

#define     MAVLINK_USART_TX_PORT         GPIOE
#define     MAVLINK_USART_TX_Pin          GPIO_Pin_1
#define     MAVLINK_USART_RX_PORT         GPIOE
#define     MAVLINK_USART_RX_Pin          GPIO_Pin_0

#define     MAVLINK_USART_AF_PinSource_TX      GPIO_PinSource1
#define     MAVLINK_USART_AF_PinSource_RX      GPIO_PinSource0
#define     MAVLINK_USART_AF                   GPIO_AF_UART8

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void usart_init(void);
void USART4_SetBaudRate( uint32_t baudrate);
bool USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *Data, uint16_t length, uint32_t timeout);
bool USART_ReceiveBytes(USART_TypeDef *USARTx, uint8_t *Data, uint32_t length, uint32_t timeout);
/*------------------------------------test------------------------------------*/

#endif /*_BSP_USART_H*/
