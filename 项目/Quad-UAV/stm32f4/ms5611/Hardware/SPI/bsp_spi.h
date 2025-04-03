/**
  ******************************************************************************
  * @file    bsp_spi.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/02/28
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_SPI_H
#define __BSP_SPI_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define     SPI_SCK_L           GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#define     SPI_SCK_H           GPIO_SetBits(GPIOA, GPIO_Pin_5)
/* Exported functions ------------------------------------------------------- */

void spi_init(void);
uint8_t SPI_TansmissionReceiveByte(uint8_t data);
#endif /* __BSP_SPI_H */


 
