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
 * ʵ��ƽ̨:
 * 
 * *****************************************************************************
 */

#ifndef __BSP_MPU6050_H 
#define __BSP_MPU6050_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*mpu6050 iicͨ�ŵ�ַ��110100x�������һλ������AD0������*/
/*�����ַ�ں��˶�дλ��AD0=0��д����Ϊ1��������Ϊ0*/
#define        MPU6050_ADR          0xD1
#define        MPU6050_ADW          0xD0

/*mpu6050�Ĵ���*/
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

/*MPU6050���ܲ��Ժ���ʹ�ܺ꣬0�������ԣ�1������*/
#define     MPU6050TestFun                 0
/*----------------------------------typedef-----------------------------------*/
/**
 * @brief       MPU6050�������ݽṹ�嶨��
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
