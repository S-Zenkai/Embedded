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
#define     SPI_CS_L           GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define     SPI_CS_H           GPIO_SetBits(GPIOC, GPIO_Pin_2)
/* Exported functions ------------------------------------------------------- */

void spi_init(void);
uint8_t SPI_TR_Byte(uint8_t data);
#endif /* __BSP_SPI_H */


 
