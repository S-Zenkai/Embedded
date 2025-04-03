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
 #include <stdbool.h>
 
 /* Exported macro ------------------------------------------------------------*/
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

 typedef struct
{
    int16_t ACCEL_XOUT;
    int16_t ACCEL_YOUT;
    int16_t ACCEL_ZOUT;
    int16_t GYRO_XOUT;
    int16_t GYRO_YOUT;
    int16_t GYRO_ZOUT;
} MPU6000_DataTypedef;

/*利用联合体共用一块内存特性，x=acc_arr[0],y=acc_arr[1],z=acc_arr[2]*/
typedef union
{
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis[3];
} MPU60003axis16_t;

typedef union
{
    struct
    {
        int64_t x;
        int64_t y;
        int64_t z;
    };
    int64_t axis[3];
} MPU60003axis64_t;

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis[3];
} MPU60003axisf_t;

/*MPU6000存储数据结构体*/
typedef struct
{
    MPU60003axis16_t AccRaw;/*原始数据*/
    MPU60003axis16_t GyroRaw;
    MPU60003axisf_t Acc;/*经处理的数据*/
    MPU60003axisf_t Gyro;
} MPU6000Data_t;

/*MPU8000校准结构体*/
typedef struct
{
    bool gyro_calibration_done;/*是否校准flag*/
    bool acc_calibration_done;
    MPU60003axis64_t bias_sum;/*偏差和*/
    MPU60003axis64_t bias_sumsq;/*偏差平方和*/
    float acc_sumsq;/*根号下的acc三轴平方和*/
    float acc_scale;/*重力缩放因子*/
    uint32_t sample_count;/*采样计数*/
} MPU6000Calibration_t;

/* Exported variables --------------------------------------------------------*/

 /* Exported functions ------------------------------------------------------- */
void MPU6000_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6000_ReadReg(uint8_t reg);
void mpu6000_init(void);
void MPU6000_GetData(MPU6000Data_t *MPU6000_Data);
void MPU6000_ProcessData(MPU6000Data_t *MPU6000_Data);
#endif /* __BSP_MPU6000_H */
 
