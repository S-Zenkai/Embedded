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
 #include "pro_config.h"
 #include "bsp_spi.h"

 /* Exported macro ------------------------------------------------------------*/

 /*CS引脚操作*/
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

// /*陀螺仪满量程,DPS-度/秒*/
// #define     MPU6000_GYRO_FULL_SCALE_250DPS    250.0f
// #define     MPU6000_GYRO_FULL_SCALE_500DPS    500.0f
// #define     MPU6000_GYRO_FULL_SCALE_1000DPS    1000.0f
// #define     MPU6000_GYRO_FULL_SCALE_2000DPS    2000.0f

/*陀螺仪灵敏度,LSB/(DPS)*/
#define     MPU6000_GYRO_SENSITIVITY_250DPS    131.0f
#define     MPU6000_GYRO_SENSITIVITY_500DPS    65.5f
#define     MPU6000_GYRO_SENSITIVITY_1000DPS    32.8f
#define     MPU6000_GYRO_SENSITIVITY_2000DPS    16.4f   
/*陀螺仪转换系数(原始数据转换为真实数据(弧度/s))*/
#define     MPU6000_GYRO_CONVERSION_FACTOR    0.01745329f/MPU6000_GYRO_SENSITIVITY_2000DPS

// /*加速度计满量程*/
// #define     MPU6000_ACCEL_FULL_SCALE_2G    2.0f
// #define     MPU6000_ACCEL_FULL_SCALE_4G    4.0f
// #define     MPU6000_ACCEL_FULL_SCALE_8G    8.0f

/*加速度计灵敏度,LSB/(g)*/
#define     MPU6000_ACCEL_SENSITIVITY_2G    16384.0f
#define     MPU6000_ACCEL_SENSITIVITY_4G    8192.0f
#define     MPU6000_ACCEL_SENSITIVITY_8G    4096.0f
#define     MPU6000_ACCEL_SENSITIVITY_16G    2048.0f
/*加速度计转换系数(原始数据转换为真实数据(m/s^2))*/
#define     MPU6000_ACCEL_CONVERSION_FACTOR    9.80665f/MPU6000_ACCEL_SENSITIVITY_2G


 /* Exported types ------------------------------------------------------------*/

// /*MPU6000数据结构体*/
// #define     MPU6000_RawData_t            Axis3i16_t
// #define     MPU6000_CalData_t            Axis3f_t

/* Exported variables --------------------------------------------------------*/

 /* Exported functions ------------------------------------------------------- */
void mpu6000_init(void);
void MPU6000_GetData(Axis3_i16_u *acc, Axis3_i16_u *gyro);
void MPU6000_Data_Conversion(Axis3_f_u *acc, Axis3_f_u *gyro,Axis3_i16_u *acc_raw, Axis3_i16_u *gyro_raw);
#endif /* __BSP_MPU6000_H */
 
