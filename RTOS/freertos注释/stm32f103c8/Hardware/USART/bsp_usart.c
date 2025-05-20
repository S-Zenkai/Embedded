/**
  ******************************************************************************
  * @file    bsp_usart.c
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"
#include "bsp_systick.h"
#include "globle.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void USART_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*优先级组*/

    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  USART3-GPIO引脚配置
  * @note   
  * @param  无
  * @retval 无
  */
static void USART_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    #if SBUS_Enable
    /*引脚配置*/
    /*RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    #endif
}

/**
  * @brief  USART3配置
  * @note   
  * @param  无
  * @retval 无
  */
static void USART_Configure(void)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    #if SBUS_Enable
    USART_InitStructure.USART_BaudRate = 100000;/*SBUS规定波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_Even;;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;/*8位数据位+偶校验*/
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART3, ENABLE);
    #endif
}

/**
  * @brief  USART初始化
  * @note   
  * @param  无
  * @retval 无
  */
void usart_init(void)
{
    USART_GPIO_Configure();
    USART_Configure();
    USART_NVIC_Configure();
}

uint8_t counter;
void USART3_IRQHandler(void)
{
    
    if(USART_GetITStatus(USART3, USART_IT_IDLE)==SET)
    {
        /*清楚空闲标志*/
        USART3->SR;
        USART3->DR;
        /*获得剩余未转移通道数*/
        counter = DMA_GetCurrDataCounter(DMA1_Channel3);
        if(counter==0&&sbus_buff[active_buff][0]==0x0F&&sbus_buff[active_buff][24]==0x00)
        {
            sbus_DF_TC = SET;
        }
        /*切换缓冲区*/
        active_buff ^= 1;
        DMA_Cmd(DMA1_Channel3, DISABLE);
        DMA1_Channel3->CMAR = (uint32_t)sbus_buff[active_buff];
        DMA_SetCurrDataCounter(DMA1_Channel3, SBUS_INPUT_CHANNELS);
        DMA_Cmd(DMA1_Channel3, ENABLE);
				
    }
}


