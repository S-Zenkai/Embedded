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
#include "bsp_spi.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

#if 0
void gpio_configure(uint8_t spi_cs)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    if(spi_cs==MPU_SPI_CS)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);
    }
    /*SCK，复用推挽输出*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    /*MOSI，复用推挽输出*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    /*MISO，浮空输入*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*CS，推挽输出*/
    if(spi_cs==MPU_SPI_CS)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    }
}
#endif

void gpio_configure(uint8_t spi_cs)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;/*GPIO工作于输出模式时默认下拉（低电平），即使设置为上拉不影响引脚默认电平状态*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    // if(spi_cs==MPU_SPI_CS)
    // {
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    // GPIO_Init(GPIOC, &GPIO_InitStructure);
    // }

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*附加*/
    MPU_SPI_CS_H;
    // GPIO_SetBits(GPIOA,GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
}

void spi_configure(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    /*附加*/
    SPI_Cmd(SPI1,DISABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    /*最大时钟频率1M*/
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;/*有些不同*/
    /*MPU6000支持模式0和模式3*/
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    // SPI_InitStructure.SPI_CRCPolynomial = ;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;/*这里原先使用的是硬件*/
    SPI_InitStructure.SPI_CRCPolynomial=10;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
//    SPI1_Read_Write_Byte(0xff);
}

void spiinit(uint8_t spi_cs)
{
	// if(spi_cs==MPU_SPI_CS)
    // {
    //     gpio_configure(MPU_SPI_CS);
    //     spi_configure();
        
    // }
    gpio_configure(MPU_SPI_CS);
        spi_configure();
}

uint8_t spi_tansmission_receive_byte(uint8_t data)
{
    uint16_t ret;
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET)
        ;
    SPI_I2S_SendData(SPI1, data);
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET)
        ;
    ret = SPI_I2S_ReceiveData(SPI1);
    return ret;
}

/*------------------------------------test------------------------------------*/

