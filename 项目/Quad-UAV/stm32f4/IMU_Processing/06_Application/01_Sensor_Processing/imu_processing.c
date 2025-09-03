/**
 ******************************************************************************
 * @file    imu_processing.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/08/08
 * @brief   IMU数据处理
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/

#include "imu_processing.h"
#include <math.h>

/** @addtogroup Drone_F427
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*陀螺仪偏置采样数阈值*/
#define GYRO_BIAS_SAMPLE_COUNT_THRESHOLD 1024
/*陀螺仪偏置方差阈值*/
#define GYRO_BIAS_VAR_THRESHOLD 1
/*加速度计缩放因子采样数阈值*/
#define ACC_SCALE_FACTOR_SAMPLE_COUNT_THRESHOLD 1024

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*imu数据处理结构体*/
static IMU_Processing_t imu_processing;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  校准陀螺仪数据，在飞机静止状态下获取陀螺仪平均偏置
 * @note
 * @param  无
 * @retval 无
 */
static bool IMU_CalibrateGyro(IMU_RawData_t *imu_raw_data)
{
    /*计算静止状态时采样和及平方和*/
    imu_processing.gyro_bias_mean.x += imu_raw_data->gyro.x;
    imu_processing.gyro_bias_mean.y += imu_raw_data->gyro.y;
    imu_processing.gyro_bias_mean.z += imu_raw_data->gyro.z;
    imu_processing.gyro_square_sum.x += imu_raw_data->gyro.x * imu_raw_data->gyro.x;
    imu_processing.gyro_square_sum.y += imu_raw_data->gyro.y * imu_raw_data->gyro.y;
    imu_processing.gyro_square_sum.z += imu_raw_data->gyro.z * imu_raw_data->gyro.z;
    imu_processing.gyro_sample_count++;

    /*判断采样数是否达到采样数阈值*/
    if (imu_processing.gyro_sample_count < GYRO_BIAS_SAMPLE_COUNT_THRESHOLD)
    {
        return false;
    }

    /*计算方差(样本方差)的平方*/
    imu_processing.gyro_bias_var.x = (float)(imu_processing.gyro_square_sum.x -
                                             imu_processing.gyro_bias_mean.x * imu_processing.gyro_bias_mean.x /
                                                 GYRO_BIAS_SAMPLE_COUNT_THRESHOLD) /
                                     (GYRO_BIAS_SAMPLE_COUNT_THRESHOLD - 1);
    imu_processing.gyro_bias_var.y = (float)(imu_processing.gyro_square_sum.y -
                                             imu_processing.gyro_bias_mean.y * imu_processing.gyro_bias_mean.y /
                                                 GYRO_BIAS_SAMPLE_COUNT_THRESHOLD) /
                                     (GYRO_BIAS_SAMPLE_COUNT_THRESHOLD - 1);
    imu_processing.gyro_bias_var.z = (float)(imu_processing.gyro_square_sum.z -
                                             imu_processing.gyro_bias_mean.z * imu_processing.gyro_bias_mean.z /
                                                 GYRO_BIAS_SAMPLE_COUNT_THRESHOLD) /
                                     (GYRO_BIAS_SAMPLE_COUNT_THRESHOLD - 1);

    /*计算均值*/
    imu_processing.gyro_bias_mean.x /= GYRO_BIAS_SAMPLE_COUNT_THRESHOLD;
    imu_processing.gyro_bias_mean.y /= GYRO_BIAS_SAMPLE_COUNT_THRESHOLD;
    imu_processing.gyro_bias_mean.z /= GYRO_BIAS_SAMPLE_COUNT_THRESHOLD;

    /*以方差判断飞机是否静止，判断方差是否在阈值范围内*/
    if (imu_processing.gyro_bias_var.x > GYRO_BIAS_VAR_THRESHOLD ||
        imu_processing.gyro_bias_var.y > GYRO_BIAS_VAR_THRESHOLD ||
        imu_processing.gyro_bias_var.z > GYRO_BIAS_VAR_THRESHOLD)
    {
        imu_processing.gyro_bias_mean.x = 0;
        imu_processing.gyro_bias_mean.y = 0;
        imu_processing.gyro_bias_mean.z = 0;
        imu_processing.gyro_bias_var.x = 0;
        imu_processing.gyro_bias_var.y = 0;
        imu_processing.gyro_bias_var.z = 0;
        imu_processing.gyro_sample_count = 0;
        imu_processing.gyro_square_sum.x = 0;
        imu_processing.gyro_square_sum.y = 0;
        imu_processing.gyro_square_sum.z = 0;
        IMU_PROCESSING_DEBUG("飞机未静止，陀螺仪校准失败\r\n");
        return false;
    }
    /*校准完成*/
    imu_processing.gyro_is_calibrated = true;
    IMU_PROCESSING_DEBUG("陀螺仪校准完成\r\n");
    return true;
}

/**
 * @brief  校准加速度数据，计算加速度缩放因子
 * @note
 * @param  无
 * @retval 无
 */
static bool IMU_CalibrateAcc(IMU_RawData_t *imu_raw_data)
{
    /*校准同样需要在飞机静止状态下进行*/
    if (!imu_processing.gyro_is_calibrated)
    {
        return false;
    }
    /*三轴数据平方和*/
    float acc_scale_factor_temp = sqrtf(imu_raw_data->acc.x * imu_raw_data->acc.x +
                                        imu_raw_data->acc.y * imu_raw_data->acc.y +
                                        imu_raw_data->acc.z * imu_raw_data->acc.z) /
                                  MPU6000_ACCEL_SENSITIVITY_2G;
    /*计算缩放因子*/
    imu_processing.acc_scale_factor += acc_scale_factor_temp;
    imu_processing.acc_sample_count++;
    /*判断采样数是否达到采样数阈值*/
    if (imu_processing.acc_sample_count < ACC_SCALE_FACTOR_SAMPLE_COUNT_THRESHOLD)
    {
        return false;
    }
    /*计算均值*/
    imu_processing.acc_scale_factor /= ACC_SCALE_FACTOR_SAMPLE_COUNT_THRESHOLD;
    // /*判断缩放因子是否在阈值范围内*/
    // if (imu_processing.acc_scale_factor > 1.0f || imu_processing.acc_scale_factor < 0.9f)
    // {
    //     return false;
    // }
    /*校准完成*/
    imu_processing.acc_is_calibrated = true;
    IMU_PROCESSING_DEBUG("加速度计校准完成\r\n");
    return true;
}


/**
  * @brief  坐标转换，x指向机头，z向下，右手坐标系，对应mpu6000坐标系需要进行旋转
  * @note   
  * @param  before_x: 坐标转换前x
  * @param  before_y: 坐标转换前y
  * @param  before_z: 坐标转换前z
  * @param  after: 转换后坐标
  * @retval 无
  */
static void IMU_Coordinate_Conversion(float before_x,float before_y,float before_z,Axis3f_t *after)
{
    after->x = before_y;
    after->y = -before_x;
    after->z = before_z;
}



/**
 * @brief  imu数据处理，包括校准imu数据，转换数据
 * @note
 * @param  imu_raw_data: 原始imu数据
 * @param  imu_cal_data: 校准imu数据
 * @retval 无
 */
void IMU_Processing(IMU_RawData_t *imu_raw_data, IMU_CalData_t *imu_cal_data)
{
    /*校准、转换数据*/
    /*陀螺仪*/
    imu_cal_data->gyro.x = ((float)imu_raw_data->gyro.x - imu_processing.gyro_bias_mean.x) * MPU6000_GYRO_CONVERSION_FACTOR;
    imu_cal_data->gyro.y = ((float)imu_raw_data->gyro.y - imu_processing.gyro_bias_mean.y) * MPU6000_GYRO_CONVERSION_FACTOR;
    imu_cal_data->gyro.z = ((float)imu_raw_data->gyro.z - imu_processing.gyro_bias_mean.z) * MPU6000_GYRO_CONVERSION_FACTOR;
    IMU_Coordinate_Conversion(imu_cal_data->gyro.x, imu_cal_data->gyro.y, imu_cal_data->gyro.z, &imu_cal_data->gyro);

    /*加速度计*/
    imu_cal_data->acc.x = ((float)imu_raw_data->acc.x * MPU6000_ACCEL_CONVERSION_FACTOR) / imu_processing.acc_scale_factor;
    imu_cal_data->acc.y = ((float)imu_raw_data->acc.y * MPU6000_ACCEL_CONVERSION_FACTOR) / imu_processing.acc_scale_factor;
    imu_cal_data->acc.z = ((float)imu_raw_data->acc.z * MPU6000_ACCEL_CONVERSION_FACTOR) / imu_processing.acc_scale_factor;
    IMU_Coordinate_Conversion(imu_cal_data->acc.x, imu_cal_data->acc.y, imu_cal_data->acc.z, &imu_cal_data->acc);
    /*后面对数据滤波时，又对角速度数据y、z轴作了取反，不知道为什么*/
    // printf("%f,", imu_cal_data->acc.x);
    // printf("%f,", imu_cal_data->acc.y);
    // printf("%f,", imu_cal_data->acc.z);
    // printf("%f,", imu_cal_data->gyro.x);
    // printf("%f,", imu_cal_data->gyro.y);
    // printf("%f\n", imu_cal_data->gyro.z);
}

/**
 * @brief  imu数据处理相关参数初始化
 * @note
 * @param  无
 * @retval 无
 */
static void IMU_Processing_Param_Init(void)
{
    imu_processing.gyro_bias_mean.x = 0;
    imu_processing.gyro_bias_mean.y = 0;
    imu_processing.gyro_bias_mean.z = 0;
    imu_processing.gyro_bias_var.x = 0;
    imu_processing.gyro_bias_var.y = 0;
    imu_processing.gyro_bias_var.z = 0;
    imu_processing.gyro_sample_count = 0;
    imu_processing.gyro_is_calibrated = false;

    imu_processing.gyro_square_sum.x = 0;
    imu_processing.gyro_square_sum.y = 0;
    imu_processing.gyro_square_sum.z = 0;

    imu_processing.acc_scale_factor = 0;
    imu_processing.acc_sample_count = 0;
    imu_processing.acc_is_calibrated = false;
}

/**
 * @brief  imu数据初始化
 * @note
 * @param  无
 * @retval 无
 */
void IMU_Processing_Init(IMU_RawData_t *raw_data)
{
    IMU_Processing_Param_Init();
    /*获取imu数据校验值*/
    while (!imu_processing.gyro_is_calibrated || !imu_processing.acc_is_calibrated)
    {
        MPU6000_GetData(&raw_data->acc, &raw_data->gyro);
        /*获取imu数据校验值*/
        /*获取陀螺仪偏置*/
        if (!imu_processing.gyro_is_calibrated)
        {
            /*获取陀螺仪偏置*/
            IMU_CalibrateGyro(raw_data);
        }
        /*获取加速度计缩放因子*/
        if (imu_processing.gyro_is_calibrated && !imu_processing.acc_is_calibrated)
        {
            /*获取加速度计缩放因子*/
            IMU_CalibrateAcc(raw_data);
        }
    }
}

/**
 * @}
 */
