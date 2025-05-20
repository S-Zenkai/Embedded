/**
 * *****************************************************************************
 * @file        bsp_iic.h
 * @brief
 * @author
 * @date        2024-12-05
 * @version     0.1
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * 实验平台:
 *
 * *****************************************************************************
 */
#ifndef __BSP_IIC_H
#define __BSP_IIC_H
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*SCL、SDA引脚宏*/
#define     IIC_GPIO_ClockFun               RCC_APB2PeriphClockCmd
#define     IIC_GPIO_CLK                    RCC_APB2Periph_GPIOB
#define     IIC_SCL_PORT                    GPIOB
#define     IIC_SCL_PIN                     GPIO_Pin_12
#define     IIC_SDA_PORT                    GPIOB
#define     IIC_SDA_PIN                     GPIO_Pin_13
/*SCL、SDA读写操作*/
#define     IIC_SCL_WH            GPIO_SetBits(IIC_SCL_PORT, IIC_SCL_PIN)
#define     IIC_SCL_WL            GPIO_ResetBits(IIC_SCL_PORT, IIC_SCL_PIN)
#define     IIC_SDA_WH            GPIO_SetBits(IIC_SDA_PORT, IIC_SDA_PIN)
#define     IIC_SDA_WL            GPIO_ResetBits(IIC_SDA_PORT, IIC_SDA_PIN)
#define     IIC_SDA_R             GPIO_ReadInputDataBit(IIC_SDA_PORT, IIC_SDA_PIN)
/**/
#define     IIC_ASK               0
#define     IIC_NASK              1
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void IICInit(void);
void IIC_Start(void);
void IIC_SendAck(uint8_t ack);
uint8_t IIC_ReceiveAck(void);
void IIC_SendByte(uint8_t data);
uint8_t IIC_ReceiveByte(void);
void IIC_Stop(void);
/*------------------------------------test------------------------------------*/
#endif /* __BSP_IIC_H */
