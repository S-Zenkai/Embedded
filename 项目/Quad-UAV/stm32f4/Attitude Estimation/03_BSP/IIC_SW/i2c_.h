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
#ifndef __IIC_H
#define __IIC_H
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
/*-----------------------------------macro------------------------------------*/
/*SCL、SDA引脚操作*/
#define IIC_SCL_H GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define IIC_SCL_L GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define IIC_SDA_H GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define IIC_SDA_L GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define IIC_SDA_R GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)
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
#endif /* __IIC_H */
