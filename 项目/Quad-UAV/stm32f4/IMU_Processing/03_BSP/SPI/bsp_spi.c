/**
  ******************************************************************************
  * @file    bsp_spi.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/02/28
  * @brief   spi驱动文件
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_spi.h"
#include "bsp_systick.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/*超时时间*/
#define SPI_TIMEOUT_US 100
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  SPI配置
  * @note   无
  * @param  无
  * @retval 无
  */
static void SPI_Configure(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);

    /*SCK(PA5)*/
    /*复用推挽输出*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*MOSI*(PA7)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*MISO(PA6)*/
    /*SPI MISO能通信的关键是配置为复用，配置GPIO_PuPd_NOPULL应该是代表浮空，由外设控制输入*/
    /*GPIO_Mode_AF优先级大于GPIO_OType_PP，故该模式配置的不是复用推挽，而是由外设控制的浮空输入*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*复用配置*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    /*MPU6000：CS(PC2)*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*MPU6000的SPI最大时钟频率1M*/
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    /*MPU6000支持模式0和模式3*/
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CRCPolynomial = 0;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI1, &SPI_InitStructure);
}

/**
  * @brief  SPI初始化
  * @note   无   
  * @param  无
  * @retval 无
  */
void spi_init(void)
{
    SPI_Configure();
    SPI_Cmd(SPI1, ENABLE);
}

/**
  * @brief  通过SPI发送接收一个数据
  * @note   因为SPI工作于全双工模式，发送一个字节后必然会接收一个字节。
  * @param  data：要发送的数据
  * @retval 接收到的数据
  */
uint8_t SPI_TR_Byte(uint8_t data)
{
    uint32_t timeout_start = GetTick();
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        if(GetTick() - timeout_start > SPI_TIMEOUT_US)
        {
            return 0;
        }
    }
    SPI_I2S_SendData(SPI1, data);
    timeout_start = GetTick();
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET)
    {
        if(GetTick() - timeout_start > SPI_TIMEOUT_US)
        {
            return 0;
        }
    }
    return SPI_I2S_ReceiveData(SPI1);
}

