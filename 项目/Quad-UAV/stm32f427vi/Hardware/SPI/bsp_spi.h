/**
 * *****************************************************************************
 * @file        bsp_spi.h
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2025-01-24
 * @version     
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:
 * 
 * *****************************************************************************
 */

#ifndef __BSP_SPI_H 
#define __BSP_SPI_H 

/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
/*-----------------------------------macro------------------------------------*/
#define     MPU_SPI_CS          1

#define     SPI_SCK_L           GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#define     SPI_SCK_H           GPIO_SetBits(GPIOA, GPIO_Pin_5)

#define     SPI_MOSI_L           GPIO_ResetBits(GPIOA, GPIO_Pin_7)
#define     SPI_MOSI_H           GPIO_SetBits(GPIOA, GPIO_Pin_7)

#define     SPI_MISO_L           GPIO_ResetBits(GPIOA, GPIO_Pin_6)
#define     SPI_MISO_H           GPIO_SetBits(GPIOA, GPIO_Pin_6)

#define     MPU_SPI_CS_L           GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define     MPU_SPI_CS_H           GPIO_SetBits(GPIOC, GPIO_Pin_2)
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void spiinit(uint8_t spi_cs);
uint8_t spi_tansmission_receive_byte(uint8_t data);
/*------------------------------------test------------------------------------*/


#endif	/* __BSP_SPI_H */
