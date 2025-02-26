/**
 * *****************************************************************************
 * @file        bsp_mpu6000.c
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2025-01-26
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
#include "bsp_mpu6000.h"
#include "bsp_spi.h"
#include "bsp_systick.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
// void MPU6000_WriteReg(uint8_t reg,uint8_t data)
// {
//     MPU_SPI_CS_L;
//     while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET)
//         ;
//     // SPI_I2S_SendData(SPI1, reg&(0<<7));
//     SPI_I2S_SendData(SPI1, reg);
//     while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET)
//         ;
//     SPI_I2S_SendData(SPI1, data);
//     MPU_SPI_CS_H;
// }

void MPU6000_WriteReg(uint8_t reg,uint8_t data)
{
    MPU_SPI_CS_L;
    delay_ms(100);
    spi_tansmission_receive_byte(reg);
    spi_tansmission_receive_byte(data);
    MPU_SPI_CS_H;
    delay_ms(100);
}

// uint8_t MPU6000_ReadReg(uint8_t reg)
// {
// 	uint16_t ret;
//     MPU_SPI_CS_L;
//     while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET)
//         ;
//     // SPI_I2S_SendData(SPI1, reg|(1<<7));
//     SPI_I2S_SendData(SPI1, reg|0x80);
//     while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET)
//         ;
//     (void)SPI_I2S_ReceiveData(SPI1);
//     // SPI_I2S_SendData(SPI1, reg);
//     SPI_I2S_SendData(SPI1, 0x00);
//     while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET)
//         ;
//     ret = SPI_I2S_ReceiveData(SPI1);
// 	MPU_SPI_CS_H;
//     return ret;
// }


uint8_t MPU6000_ReadReg(uint8_t reg)
{
	uint16_t ret;
    MPU_SPI_CS_L;
    delay_ms(100);
    spi_tansmission_receive_byte(reg|0x80);
    ret = spi_tansmission_receive_byte(0xff);
	MPU_SPI_CS_H;
    delay_ms(100);
    return ret;
}

// uint8_t MPU6000_Write_ReadReg(uint8_t reg)
// {
//     uint16_t ret;
//     // MPU_SPI_CS_L;
//     while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET)
//         ;
//     // SPI_I2S_SendData(SPI1, reg&(0<<7));
//     SPI_I2S_SendData(SPI1, reg);
//     while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET)
//         ;
//     ret = SPI_I2S_ReceiveData(SPI1);
// 	// MPU_SPI_CS_H;
//     return ret;
// }

// uint8_t MPU6000_WriteReg(uint8_t reg,uint8_t data)
// {
//     uint8_t status;
//     MPU_SPI_CS_L;
//     status=MPU6000_Write_ReadReg(reg);
//     MPU6000_Write_ReadReg(data);
//     MPU_SPI_CS_H;
//     return status;
// }

// uint8_t MPU6000_ReadReg(uint8_t reg)
// {
// 	uint16_t ret;
//     MPU_SPI_CS_L;
//     // while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET)
//     //     ;
//     // SPI_I2S_SendData(SPI1, reg&(0<<7));
//     MPU6000_Write_ReadReg( reg);
//     // while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET)
//     //     ;
//     ret = MPU6000_Write_ReadReg( 0xff);
// 	MPU_SPI_CS_H;
//     return ret;
// }

void mpu6000_init(void)
{
    spiinit(MPU_SPI_CS);
    if (MPU6000_ReadReg(MPU6000_WHO_AM_I) == 0x92)
    {
        /*复位MPU6000所有寄存器（唤醒前可以写电源管理寄存器）*/
        MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x80); /*复位MPU6000所有寄存器（唤醒前可以写电源管理寄存器）*/
        delay_ms(100);                              /*复位需要一定时间*/
        /*唤醒MPU6000*/
        MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x00);
        /*不启动自检，+-2000满量程*/
        MPU6000_WriteReg(MPU6000_GYRO_CONFIG, 0x18);
        /*不启动自检，+-2g满量程（小外力时使用2g即可）*/
        MPU6000_WriteReg(MPU6000_ACCEL_CONFIG, 0x00);
        /*使能加速度计和陀螺仪各个轴(可不使能，复位后各个轴便使能了)*/
        MPU6000_WriteReg(MPU6000_PWR_MGMT_2, 0x00);
        /*选择时钟源为X轴陀螺仪*/
        MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x01);
        /*设置采样率(MPU6000采样率最小为4HZ，最大为8KHZ)*/
        /*采样率公式：f=...*/
        MPU6000_WriteReg(MPU6000_SMPRT_DIV, 9);
        /*设置数字滤波器（需要根据香农采样定理设置带宽）*/
        MPU6000_WriteReg(MPU6000_CONFIG, 0x03);
    }
    // /*复位MPU6000所有寄存器（唤醒前可以写电源管理寄存器）*/
    // MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x80);/*复位MPU6000所有寄存器（唤醒前可以写电源管理寄存器）*/
    // delay_ms(100);/*复位需要一定时间*/
    // /*唤醒MPU6000*/
    // MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x00);
    // /*不启动自检，+-2000满量程*/
    // MPU6000_WriteReg(MPU6000_GYRO_CONFIG, 0x18);
    // /*不启动自检，+-2g满量程（小外力时使用2g即可）*/
    // MPU6000_WriteReg(MPU6000_ACCEL_CONFIG, 0x00);
    // /*使能加速度计和陀螺仪各个轴(可不使能，复位后各个轴便使能了)*/
    // MPU6000_WriteReg(MPU6000_PWR_MGMT_2, 0x00);
    // /*选择时钟源为X轴陀螺仪*/
    // MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x01);
    // /*设置采样率(MPU6000采样率最小为4HZ，最大为8KHZ)*/
    // /*采样率公式：f=...*/
    // MPU6000_WriteReg(MPU6000_SMPRT_DIV, 9);
    // /*设置数字滤波器（需要根据香农采样定理设置带宽）*/
    // MPU6000_WriteReg(MPU6000_CONFIG, 0x03);
}

/**
 * @brief       获取MPU6000测量数据，注意，获取的数据为2进制补码值，
 *
 * @param       MPU6000_DataStruct 指向MPU6000_DataTypedef结构体的指针，该结构体包含MPU6000测量数据
 */
void MPU6000GetData(MPU6000_DataTypedef* MPU6000_DataStruct)
{
    MPU6000_DataStruct->ACCEL_XOUT = MPU6000_ReadReg(MPU6000_ACCEL_XOUT_H);
    MPU6000_DataStruct->ACCEL_XOUT <<= 8;
    MPU6000_DataStruct->ACCEL_XOUT |= MPU6000_ReadReg(MPU6000_ACCEL_XOUT_L);

    MPU6000_DataStruct->ACCEL_YOUT = MPU6000_ReadReg(MPU6000_ACCEL_YOUT_H);
    MPU6000_DataStruct->ACCEL_YOUT <<= 8;
    MPU6000_DataStruct->ACCEL_YOUT |= MPU6000_ReadReg(MPU6000_ACCEL_YOUT_L);

    MPU6000_DataStruct->ACCEL_ZOUT = MPU6000_ReadReg(MPU6000_ACCEL_ZOUT_H);
    MPU6000_DataStruct->ACCEL_ZOUT <<= 8;
    MPU6000_DataStruct->ACCEL_ZOUT |= MPU6000_ReadReg(MPU6000_ACCEL_ZOUT_L);

    MPU6000_DataStruct->GYRO_XOUT = MPU6000_ReadReg(MPU6000_GYRO_XOUT_H);
    MPU6000_DataStruct->GYRO_XOUT <<= 8;
    MPU6000_DataStruct->GYRO_XOUT |= MPU6000_ReadReg(MPU6000_GYRO_XOUT_L);

    MPU6000_DataStruct->GYRO_YOUT = MPU6000_ReadReg(MPU6000_GYRO_YOUT_H);
    MPU6000_DataStruct->GYRO_YOUT <<= 8;
    MPU6000_DataStruct->GYRO_YOUT |= MPU6000_ReadReg(MPU6000_GYRO_YOUT_L);

    MPU6000_DataStruct->GYRO_ZOUT = MPU6000_ReadReg(MPU6000_GYRO_ZOUT_H);
    MPU6000_DataStruct->GYRO_ZOUT <<= 8;
    MPU6000_DataStruct->GYRO_ZOUT |= MPU6000_ReadReg(MPU6000_GYRO_ZOUT_L);
}
/*------------------------------------test------------------------------------*/



