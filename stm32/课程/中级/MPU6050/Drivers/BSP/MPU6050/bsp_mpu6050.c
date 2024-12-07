/**
 * *****************************************************************************
 * @file        bsp_mpu6050.c
 * @brief       
 * @author      
 * @date        2024-12-05
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:    stm32f103vet6
 * 
 * *****************************************************************************
 */
/*----------------------------------include-----------------------------------*/
#include "bsp_mpu6050.h"
#include "bsp_iic.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void MPU6050_SendByte(uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    IIC_SendByte(reg);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    IIC_SendByte(data);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    IIC_Stop();
}

void MPU6050_SendBytes(uint8_t reg, uint8_t* databuff,uint16_t buffsz)
{
    uint16_t i;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    IIC_SendByte(reg);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    for (i = 0; i < buffsz;i++)
    {
        IIC_SendByte(databuff[i]);
        while (IIC_ReceiveAck() == IIC_ASK)
            ;
    }
    IIC_Stop();
}

uint8_t MPU6050_ReadByte(uint8_t reg, uint8_t data)
{
    uint8_t ret;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    IIC_SendByte(reg);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    IIC_Start();
    IIC_SendByte(MPU6050_ADR);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    ret = IIC_ReceiveByte();
    IIC_SendAck(IIC_NASK);
    IIC_Stop();
}

void MPU6050_ReadBytes(uint8_t reg, uint8_t* databuff,uint16_t buffsz)
{
    uint8_t ret;
    uint8_t i = 0;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    IIC_SendByte(reg);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    IIC_Start();
    IIC_SendByte(MPU6050_ADR);
    while (IIC_ReceiveAck() == IIC_ASK)
        ;
    for (i = 0; i < buffsz-1;i++)
    {
        databuff[i] = IIC_ReceiveByte();
        IIC_SendAck(IIC_ASK);
    }
    databuff[buffsz - 1] = IIC_ReceiveByte();
    IIC_SendAck(IIC_NASK);
    IIC_Stop();
}
    /*------------------------------------test------------------------------------*/
