/**
  ******************************************************************************
  * @file    bsp_spi.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/02/28
  * @brief   spi�����ļ�
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_spi.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  SPI���GPIO��ʼ��
  * @note   �� 
  * @param  ��
  * @retval ��
  */
static void SPI_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /*SCK(PA5)*/
    /*�����������*/
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
    /*SPI MISO��ͨ�ŵĹؼ�������Ϊ���ã�����GPIO_PuPd_NOPULLӦ���Ǵ����գ��������������*/
    /*GPIO_Mode_AF���ȼ�����GPIO_OType_PP���ʸ�ģʽ���õĲ��Ǹ������죬������������Ƶĸ�������*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*��������*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
    
}

/**
  * @brief  SPI����
  * @note   ��
  * @param  ��
  * @retval ��
  */
static void SPI_Configure(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    /*MPU6000��SPI���ʱ��Ƶ��1M*/
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    /*MPU6000֧��ģʽ0��ģʽ3*/
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
  * @brief  SPI��ʼ��
  * @note   ��   
  * @param  ��
  * @retval ��
  */
void spi_init(void)
{
    SPI_GPIO_Configure();
    SPI_Configure();
    SPI_Cmd(SPI1, ENABLE);
}

/**
  * @brief  ͨ��SPI���ͽ���һ������
  * @note   ��ΪSPI������ȫ˫��ģʽ������һ���ֽں��Ȼ�����һ���ֽڡ�
  * @param  data��Ҫ���͵�����
  * @retval ���յ�������
  */
uint8_t SPI_TansmissionReceiveByte(uint8_t data)
{
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
      ;
    SPI_I2S_SendData(SPI1, data);
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET)
      ;
    return SPI_I2S_ReceiveData(SPI1);
}

