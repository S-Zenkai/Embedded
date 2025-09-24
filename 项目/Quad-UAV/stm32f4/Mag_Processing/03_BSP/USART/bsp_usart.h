/**
  ******************************************************************************
  * @file    bsp_usart.h
  * @author  kai
  * @version V1
  * @date    2025/2/19
  * @brief   usart驱动程序头文件
  ******************************************************************************
  * @attention
  *
  * 注意对F4系列单片机的引脚复用功能，还需要调用对应复用函数
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
/*************************************************************************************/
/*UART7宏定义*/
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

/*************************************************************************************/
/*UART8宏定义*/
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

/*************************************************************************************/
/*USART6宏定义*/
#define     ICC_USART                 USART6
#define     ICC_USART_CLK_FUN         RCC_APB2PeriphClockCmd
#define     ICC_USART_CLK             RCC_APB2Periph_USART6

#define     ICC_USART_GPIO_CLK_FUN    RCC_AHB1PeriphClockCmd
#define     ICC_USART_GPIO_CLK        RCC_AHB1Periph_GPIOC

#define     ICC_USART_TX_PORT         GPIOC
#define     ICC_USART_TX_Pin          GPIO_Pin_6
#define     ICC_USART_RX_PORT         GPIOC
#define     ICC_USART_RX_Pin          GPIO_Pin_7

#define     ICC_USART_AF_PinSource_TX      GPIO_PinSource6
#define     ICC_USART_AF_PinSource_RX      GPIO_PinSource7
#define     ICC_USART_AF                   GPIO_AF_USART6
/***************************************************************************************/
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void usart_init(void);
void USART4_SetBaudRate( uint32_t baudrate);
bool USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *Data, uint16_t length, uint32_t timeout);
bool USART_ReceiveBytes(USART_TypeDef *USARTx, uint8_t *Data, uint32_t length, uint32_t timeout);
void USART_DMA_Send(DMA_Stream_TypeDef *DMAy_Streamx, uint8_t *buffer, uint16_t len);
/*------------------------------------test------------------------------------*/

#endif /*_BSP_USART_H*/
