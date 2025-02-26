/**
 * *****************************************************************************
 * @file        bsp_mpu6000.h
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

#ifndef __BSP_MPU6000_H 
#define __BSP_MPU6000_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f4xx.h"
/*-----------------------------------macro------------------------------------*/
/*mpu6000寄存器*/
#define     MPU6000_WHO_AM_I                0x75
#define     MPU6000_PWR_MGMT_1              0x6B
#define     MPU6000_PWR_MGMT_2              0x6C
#define     MPU6000_GYRO_CONFIG             0x1B
#define     MPU6000_ACCEL_CONFIG            0x1C
#define     MPU6000_SMPRT_DIV               0x19
#define     MPU6000_CONFIG                  0x1A

#define     MPU6000_ACCEL_XOUT_H            0x3B
#define     MPU6000_ACCEL_XOUT_L            0x3C
#define     MPU6000_ACCEL_YOUT_H            0x3D
#define     MPU6000_ACCEL_YOUT_L            0x3E
#define     MPU6000_ACCEL_ZOUT_H            0x3F
#define     MPU6000_ACCEL_ZOUT_L            0x40

#define     MPU6000_GYRO_XOUT_H            0x43
#define     MPU6000_GYRO_XOUT_L            0x44
#define     MPU6000_GYRO_YOUT_H            0x45
#define     MPU6000_GYRO_YOUT_L            0x46
#define     MPU6000_GYRO_ZOUT_H            0x47
#define     MPU6000_GYRO_ZOUT_L            0x48

/*----------------------------------typedef-----------------------------------*/
/**
 * @brief       MPU6000测量数据结构体定义
 * 
 */
typedef struct
{
    int16_t ACCEL_XOUT;
    int16_t ACCEL_YOUT;
    int16_t ACCEL_ZOUT;
    int16_t GYRO_XOUT;
    int16_t GYRO_YOUT;
    int16_t GYRO_ZOUT;
} MPU6000_DataTypedef;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
// void MPU6000_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6000_Write_ReadReg(uint8_t reg);
uint8_t MPU6000_ReadReg(uint8_t reg);
void mpu6000_init(void);
void MPU6000GetData(MPU6000_DataTypedef *MPU6000_DataStruct);
/*------------------------------------test------------------------------------*/

#endif	/* __BSP_MPU6000_H */
