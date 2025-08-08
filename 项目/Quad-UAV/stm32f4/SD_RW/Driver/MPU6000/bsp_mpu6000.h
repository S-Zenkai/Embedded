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

 /*CS引脚操作*/
#define     MPU6000_SPI_CS_L            SPI_CS_L
#define     MPU6000_SPI_CS_H            SPI_CS_H

/*MPU6000数据结构体*/
#define     MPU6000_RawData_t            Axis3i16_t
#define     MPU6000_CalData_t            Axis3f_t

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

/*校准采样值*/
#define     gyro_calibration_simple           1024
#define     acc_calibration_simple            400

/*MPU6000量程转换*/
#define     gyro_range_2000          (32768.0f/2000.0f)
#define     acc_range_2g             (32768.0f/2.0f)
 /* Exported types ------------------------------------------------------------*/



/*MPU8000校准结构体*/
typedef struct
{
    bool gyro_calibration_done;/*是否校准flag*/
    bool acc_calibration_done;
    Axis3i64_t bias_sum;/*偏差和*/
    Axis3i64_t bias_sumsq;/*偏差平方和*/
    float acc_sumsq;/*根号下的acc三轴平方和*/
    float acc_scale;/*重力缩放因子*/
    uint32_t sample_count;/*采样计数*/
} MPU6000Calibration_t;

/* Exported variables --------------------------------------------------------*/

 /* Exported functions ------------------------------------------------------- */
void MPU6000_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6000_ReadReg(uint8_t reg);
void mpu6000_init(void);
void MPU6000_GetData(MPU6000_RawData_t *acc, MPU6000_RawData_t *gyro);
void MPU6000_ProcessData(Axis3i16_t *acc_raw, Axis3i16_t *gyro_raw, Axis3f_t *acc_cal, Axis3f_t *gyro_cal);
#endif /* __BSP_MPU6000_H */
 
