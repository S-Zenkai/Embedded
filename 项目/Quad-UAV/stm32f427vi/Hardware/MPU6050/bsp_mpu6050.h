/**
 * *****************************************************************************
 * @file        bsp_mpu6050.h
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

#ifndef __BSP_MPU6050_H 
#define __BSP_MPU6050_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*mpu6050 iic通信地址（110100x），最后一位由引脚AD0决定）*/
/*这里地址融合了读写位，AD0=0，写操作为1，读操作为0*/
#define        MPU6050_ADR          0xD1
#define        MPU6050_ADW          0xD0

/*mpu6050寄存器*/
#define     MPU6050_WHO_AM_I                0x75
#define     MPU6050_PWR_MGMT_1              0x6B
#define     MPU6050_PWR_MGMT_2              0x6C
#define     MPU6050_GYRO_CONFIG             0x1B
#define     MPU6050_ACCEL_CONFIG            0x1C
#define     MPU6050_SMPRT_DIV               0x19
#define     MPU6050_CONFIG                  0x1A

#define     MPU6050_ACCEL_XOUT_H            0x3B
#define     MPU6050_ACCEL_XOUT_L            0x3C
#define     MPU6050_ACCEL_YOUT_H            0x3D
#define     MPU6050_ACCEL_YOUT_L            0x3E
#define     MPU6050_ACCEL_ZOUT_H            0x3F
#define     MPU6050_ACCEL_ZOUT_L            0x40

#define     MPU6050_GYRO_XOUT_H            0x43
#define     MPU6050_GYRO_XOUT_L            0x44
#define     MPU6050_GYRO_YOUT_H            0x45
#define     MPU6050_GYRO_YOUT_L            0x46
#define     MPU6050_GYRO_ZOUT_H            0x47
#define     MPU6050_GYRO_ZOUT_L            0x48

/*MPU6050功能测试函数使能宏，0：不测试；1：测试*/
#define     MPU6050TestFun                 0
/*----------------------------------typedef-----------------------------------*/
/**
 * @brief       MPU6050测量数据结构体定义
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
} MPU6050_DataTypedef;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void MPU6050Init(void);
void MPU6050_WriteReg(uint8_t reg, uint8_t data);
void MPU6050_WriteRegContinue(uint8_t reg, uint8_t *databuff, uint16_t buffsz);
uint8_t MPU6050_ReadReg(uint8_t reg);
void MPU6050_ReadRegContinue(uint8_t reg, uint8_t *databuff, uint16_t buffsz);
void MPU6050GetData(MPU6050_DataTypedef *MPU6050_DataStruct);

#if MPU6050TestFun
void MPU6050Test(void);
#endif
/*------------------------------------test------------------------------------*/

#endif	/* __BSP_MPU6050_H */
