/**
 * *****************************************************************************
 * @file        bsp_spi.c
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2025-01-24
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:
 * 
 * *****************************************************************************
 */
/*----------------------------------include-----------------------------------*/
#include "bsp_spi_simu.h"
/*-----------------------------------macro------------------------------------*/
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

void gpio_configure(uint8_t spi_cs)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    if(spi_cs==MPU_SPI_CS)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);
    }
    /*SCK*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*MOSI*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*CS*/
    if(spi_cs==MPU_SPI_CS)
    {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    }
    /*MISO*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}



void spi_init(uint8_t spi_cs)
{
	if(spi_cs==MPU_SPI_CS)
    {
        gpio_configure(MPU_SPI_CS);
        MPU_SPI_CS_H;
        /*CPOL=0,CPHA=0,模式0*/
        SPI_SCK_L;
    }

}


/*------------------------------------test------------------------------------*/

