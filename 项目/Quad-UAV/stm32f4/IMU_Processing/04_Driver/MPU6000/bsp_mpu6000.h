/**
  ******************************************************************************
  * @file    bsp_mpu6000.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/02/28
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
 /* Define to prevent recursive inclusion -------------------------------------*/
 #ifndef __BSP_MPU6000_H
 #define __BSP_MPU6000_H
 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"
 #include "bsp_spi.h"
 #include "com_type.h"
 #include <stdbool.h>
 
 /* Exported macro ------------------------------------------------------------*/

 /*CS���Ų���*/
#define     MPU6000_SPI_CS_L            SPI_CS_L
#define     MPU6000_SPI_CS_H            SPI_CS_H


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

#define     MPU6000_INT_STATUS             0X3A
#define     MPU6000_INT_ENABLE             0X38
#define     MPU6000_INT_PIN_CFG            0X37

// /*������������,DPS-��/��*/
// #define     MPU6000_GYRO_FULL_SCALE_250DPS    250.0f
// #define     MPU6000_GYRO_FULL_SCALE_500DPS    500.0f
// #define     MPU6000_GYRO_FULL_SCALE_1000DPS    1000.0f
// #define     MPU6000_GYRO_FULL_SCALE_2000DPS    2000.0f

/*������������,LSB/(DPS)*/
#define     MPU6000_GYRO_SENSITIVITY_250DPS    131.0f
#define     MPU6000_GYRO_SENSITIVITY_500DPS    65.5f
#define     MPU6000_GYRO_SENSITIVITY_1000DPS    32.8f
#define     MPU6000_GYRO_SENSITIVITY_2000DPS    16.4f   
/*������ת��ϵ��(ԭʼ����ת��Ϊ��ʵ����(����/s))*/
#define     MPU6000_GYRO_CONVERSION_FACTOR    0.01745329f/MPU6000_GYRO_SENSITIVITY_2000DPS

// /*���ٶȼ�������*/
// #define     MPU6000_ACCEL_FULL_SCALE_2G    2.0f
// #define     MPU6000_ACCEL_FULL_SCALE_4G    4.0f
// #define     MPU6000_ACCEL_FULL_SCALE_8G    8.0f

/*���ٶȼ�������,LSB/(g)*/
#define     MPU6000_ACCEL_SENSITIVITY_2G    16384.0f
#define     MPU6000_ACCEL_SENSITIVITY_4G    8192.0f
#define     MPU6000_ACCEL_SENSITIVITY_8G    4096.0f
#define     MPU6000_ACCEL_SENSITIVITY_16G    2048.0f
/*���ٶȼ�ת��ϵ��(ԭʼ����ת��Ϊ��ʵ����(m/s^2))*/
#define     MPU6000_ACCEL_CONVERSION_FACTOR    9.80665f/MPU6000_ACCEL_SENSITIVITY_2G


 /* Exported types ------------------------------------------------------------*/

// /*MPU6000���ݽṹ��*/
// #define     MPU6000_RawData_t            Axis3i16_t
// #define     MPU6000_CalData_t            Axis3f_t

/* Exported variables --------------------------------------------------------*/

 /* Exported functions ------------------------------------------------------- */
void mpu6000_init(void);
void MPU6000_GetData(Axis3i16_t *acc, Axis3i16_t *gyro);
#endif /* __BSP_MPU6000_H */
 
