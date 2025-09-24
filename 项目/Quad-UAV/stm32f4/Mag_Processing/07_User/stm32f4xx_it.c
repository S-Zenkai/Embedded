/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.8.1
 * @date    27-January-2022
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"

/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t gps_rx_temp;
extern uint8_t mavlink_rx_temp;
extern ringbuff_t MAVLINK_RX_RingBuffMgr;
extern __IO FlagStatus Main_Loop_Update_Flag;
extern uint8_t icc_rx_temp;
extern ringbuff_t ICC_RX_RingBuffMgr;
extern __IO FlagStatus ICC_Send_Done_Flag;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/




/**
 * @brief  DMA1_Stream2_IRQHandler
 * @note
 * @param  无
 * @retval 无
 */
void DMA1_Stream2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
        rb_push(gps_rx_temp, &m_GPS_RX_RingBuffMgr);
    }
}

/**
 * @brief  DMA1_Stream6_IRQHandler
 * @note
 * @param  无
 * @retval 无
 */
void DMA1_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
        rb_push(mavlink_rx_temp, &MAVLINK_RX_RingBuffMgr);
    }
}

/**
 * @brief  DMA2_Stream1_IRQHandler
 * @note
 * @param  无
 * @retval 无
 */
void DMA2_Stream1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
        rb_push(icc_rx_temp, &ICC_RX_RingBuffMgr);
    }
}

/**
 * @brief  DMA2_Stream2_IRQHandler
 * @note
 * @param  无
 * @retval 无
 */
void DMA2_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);
        ICC_Send_Done_Flag = SET;
    }
}

/**
 * @}
 */
