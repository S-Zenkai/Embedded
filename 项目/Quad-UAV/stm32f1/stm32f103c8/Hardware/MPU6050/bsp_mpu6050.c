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
#include "bsp_systick.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief       向MPU6050寄存器写一个数据
 * 
 * @param       reg 寄存器地址
 * @param       data 要写的数据
 */
void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    /*应该使用一定方法处理未收到应答信号的情况，但不能用while循环处理，会造成时序出错*/
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    IIC_SendByte(data);
    IIC_ReceiveAck();
    IIC_Stop();
}

/**
 * @brief       向MPU6050寄存器连续写数据
 * 
 * @param       reg 寄存器地址
 * @param       databuff 要写的数据缓冲区指针
 * @param       buffsz 数据缓存区大小
 */
void MPU6050_WriteRegContinue(uint8_t reg, uint8_t *databuff, uint16_t buffsz)
{
    uint16_t i;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    for (i = 0; i < buffsz;i++)
    {
        IIC_SendByte(databuff[i]);
        IIC_ReceiveAck();
    }
    IIC_Stop();
}

/**
 * @brief       读MPU6050寄存器
 * 
 * @param       reg 寄存器地址
 * @return      uint8_t 读出的数据
 */
uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t ret;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    /*注意，如果使用while处理应答信号，无法正常通信，猜测可能是因为无法满足时序要求*/
    // while (IIC_ReceiveAck() == IIC_ASK)
    //     ;
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    IIC_Start();
    IIC_SendByte(MPU6050_ADR);
    IIC_ReceiveAck();
    ret = IIC_ReceiveByte();
    IIC_SendAck(IIC_NASK);
    IIC_Stop();
    return ret;
}

/**
 * @brief       连续读MPU6050寄存器
 *
 * @param       reg 寄存器地址
 * @param       databuff    数据缓冲区
 * @param       buffsz      数据缓存区大小
 */
void MPU6050_ReadRegContinue(uint8_t reg, uint8_t* databuff,uint16_t buffsz)
{
    uint8_t i = 0;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    IIC_Start();
    IIC_SendByte(MPU6050_ADR);
    IIC_ReceiveAck();
    for (i = 0; i < buffsz-1;i++)
    {
        databuff[i] = IIC_ReceiveByte();
        IIC_SendAck(IIC_ASK);
    }
    databuff[buffsz - 1] = IIC_ReceiveByte();
    IIC_SendAck(IIC_NASK);
    IIC_Stop();
}

/**
 * @brief       MPU6050初始化，包括IIC通信引脚初始化、唤醒MPU6050、设置量程、设置采样率、设置数据滤波器
 * 
 */
// void MPU6050Init(void)
// {
//     IICInit();
//     MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);/*唤醒MPU6050，时钟源为X轴陀螺仪*/
//     MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); 
//     MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);/*不启动自检，+-2000满量程*/
//     MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); /*不启动自检，+-2000满量程*/
//     MPU6050_WriteReg(MPU6050_SMPRT_DIV, 100);/*设置采样率*/
//     MPU6050_WriteReg(MPU6050_CONFIG, 100);   /*设置数字滤波器*/
// }

void MPU6050Init(void)
{
    IICInit();
    /*复位MPU6050所有寄存器（唤醒前可以写电源管理寄存器）*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);/*复位MPU6050所有寄存器（唤醒前可以写电源管理寄存器）*/
    delay_ms(100);/*复位需要一定时间*/
    /*唤醒MPU6050*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    /*不启动自检，+-2000满量程*/
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    /*不启动自检，+-2g满量程（小外力时使用2g即可）*/
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
    /*使能加速度计和陀螺仪各个轴(可不使能，复位后各个轴便使能了)*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    /*选择时钟源为X轴陀螺仪*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    /*设置采样率(MPU6050采样率最小为4HZ，最大为8KHZ)*/
    /*采样率公式：f=...*/
    MPU6050_WriteReg(MPU6050_SMPRT_DIV, 9);
    /*设置数字滤波器（需要根据香农采样定理设置带宽）*/
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);
}

/**
 * @brief       获取MPU6050测量数据，注意，获取的数据为2进制补码值，
 *
 * @param       MPU6050_DataStruct 指向MPU6050_DataTypedef结构体的指针，该结构体包含MPU6050测量数据
 */
void MPU6050GetData(MPU6050_DataTypedef* MPU6050_DataStruct)
{
    MPU6050_DataStruct->ACCEL_XOUT = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    MPU6050_DataStruct->ACCEL_XOUT <<= 8;
    MPU6050_DataStruct->ACCEL_XOUT |= MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);

    MPU6050_DataStruct->ACCEL_YOUT = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    MPU6050_DataStruct->ACCEL_YOUT <<= 8;
    MPU6050_DataStruct->ACCEL_YOUT |= MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);

    MPU6050_DataStruct->ACCEL_ZOUT = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    MPU6050_DataStruct->ACCEL_ZOUT <<= 8;
    MPU6050_DataStruct->ACCEL_ZOUT |= MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);

    MPU6050_DataStruct->GYRO_XOUT = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    MPU6050_DataStruct->GYRO_XOUT <<= 8;
    MPU6050_DataStruct->GYRO_XOUT |= MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);

    MPU6050_DataStruct->GYRO_YOUT = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    MPU6050_DataStruct->GYRO_YOUT <<= 8;
    MPU6050_DataStruct->GYRO_YOUT |= MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);

    MPU6050_DataStruct->GYRO_ZOUT = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    MPU6050_DataStruct->GYRO_ZOUT <<= 8;
    MPU6050_DataStruct->GYRO_ZOUT |= MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
}
/*------------------------------------test------------------------------------*/
#if MPU6050TestFun
#include "stm32f10x.h"
#include "OLED.h"
#include "bsp_usart.h"
#include "bsp_mpu6050.h"
void MPU6050Test(void)
{
    MPU6050_DataTypedef MPU6050_DataStructure;
    OLED_Init();
	SystickInit(72);
    uint8_t ret = 0;
    MPU6050Init();
    ret = MPU6050_ReadReg(MPU6050_WHO_AM_I);
    OLED_ShowString(0, 0, "ID:00", OLED_6X8);
    OLED_ShowHexNum(24, 0, ret, 2, OLED_6X8);
    OLED_Update();
    while (1)
    {
        MPU6050GetData(&MPU6050_DataStructure);
        /*注意，MPU6050数据寄存器取出的数据是有符号数 */
        OLED_ShowString(0, 8, "ACCEL:X 0000", OLED_6X8);
        OLED_ShowFloatNum(48, 8, (float)MPU6050_DataStructure.ACCEL_XOUT / 32768 * 2, 2, 2, OLED_6X8);

        OLED_ShowString(0, 16, "ACCEL:Y 0000", OLED_6X8);
        OLED_ShowFloatNum(48, 16, (float)MPU6050_DataStructure.ACCEL_YOUT / 32768 * 2, 2, 2, OLED_6X8);

        OLED_ShowString(0, 24, "ACCEL:Z 0000", OLED_6X8);
        OLED_ShowFloatNum(48, 24, (float)MPU6050_DataStructure.ACCEL_ZOUT / 32768 * 2, 2, 2, OLED_6X8);

        OLED_ShowString(0, 32, "GYRO:X 0000", OLED_6X8);
        OLED_ShowFloatNum(42, 32, (float)MPU6050_DataStructure.GYRO_XOUT / 32768 * 2000, 2, 2, OLED_6X8);

        OLED_ShowString(0, 40, "GYRO:Y 0000", OLED_6X8);
        OLED_ShowFloatNum(42, 40, (float)MPU6050_DataStructure.GYRO_YOUT / 32768 * 2000, 2, 2, OLED_6X8);

        OLED_ShowString(0, 48, "GYRO:Z 0000", OLED_6X8);
        OLED_ShowFloatNum(42, 48, (float)MPU6050_DataStructure.GYRO_ZOUT / 32768 * 2000, 2, 2, OLED_6X8);
        OLED_Update();
        delay_ms(100);
    }
}
#endif
