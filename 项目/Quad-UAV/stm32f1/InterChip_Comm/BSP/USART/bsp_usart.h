/**
  ******************************************************************************
  * @file    bsp_usart.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/27
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BSP_USART_H
#define _BSP_USART_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "proConfig.h"
#include "stdio.h"
#include "stdbool.h"
/* Exported define ------------------------------------------------------------*/

/*************************************************************************************/
/*USART2宏定义(双机通信)*/
#define     ICC_USART                 USART2
#define     ICC_USART_CLK_FUN         RCC_APB1PeriphClockCmd
#define     ICC_USART_CLK             RCC_APB1Periph_USART2

#define     ICC_USART_GPIO_CLK_FUN    RCC_APB2PeriphClockCmd
#define     ICC_USART_GPIO_CLK        RCC_APB2Periph_GPIOA

#define     ICC_USART_TX_PORT         GPIOA
#define     ICC_USART_TX_Pin          GPIO_Pin_2
#define     ICC_USART_RX_PORT         GPIOA
#define     ICC_USART_RX_Pin          GPIO_Pin_3
/***************************************************************************************/


/* Exported types ------------------------------------------------------------*/
/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void usart_init(void);
bool USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *Data, uint16_t length, uint32_t timeout);
void USART_DMA_Send(DMA_Channel_TypeDef *DMAy_Channelx, uint8_t *buffer, uint16_t len);
#endif /* _BSP_USART_H */




