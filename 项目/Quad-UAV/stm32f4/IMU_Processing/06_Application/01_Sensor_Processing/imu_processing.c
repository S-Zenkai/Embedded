/**
 ******************************************************************************
 * @file    imu_processing.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/08/08
 * @brief   IMU���ݴ���
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
/*������ƫ�ò�������ֵ*/
#define GYRO_BIAS_SAMPLE_COUNT_THRESHOLD 1024
/*������ƫ�÷�����ֵ*/
#define GYRO_BIAS_VAR_THRESHOLD 1
/*���ٶȼ��������Ӳ�������ֵ*/
#define ACC_SCALE_FACTOR_SAMPLE_COUNT_THRESHOLD 1024

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*imu���ݴ���ṹ��*/
static IMU_Processing_t imu_processing;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  У׼���������ݣ��ڷɻ���ֹ״̬�»�ȡ������ƽ��ƫ��
 * @note
 * @param  ��
 * @retval ��
 */
static bool IMU_CalibrateGyro(IMU_RawData_t *imu_raw_data)
{
    /*���㾲ֹ״̬ʱ�����ͼ�ƽ����*/
    imu_processing.gyro_bias_mean.x += imu_raw_data->gyro.x;
    imu_processing.gyro_bias_mean.y += imu_raw_data->gyro.y;
    imu_processing.gyro_bias_mean.z += imu_raw_data->gyro.z;
    imu_processing.gyro_square_sum.x += imu_raw_data->gyro.x * imu_raw_data->gyro.x;
    imu_processing.gyro_square_sum.y += imu_raw_data->gyro.y * imu_raw_data->gyro.y;
    imu_processing.gyro_square_sum.z += imu_raw_data->gyro.z * imu_raw_data->gyro.z;
    imu_processing.gyro_sample_count++;

    /*�жϲ������Ƿ�ﵽ��������ֵ*/
    if (imu_processing.gyro_sample_count < GYRO_BIAS_SAMPLE_COUNT_THRESHOLD)
    {
        return false;
    }

    /*���㷽��(��������)��ƽ��*/
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

    /*�����ֵ*/
    imu_processing.gyro_bias_mean.x /= GYRO_BIAS_SAMPLE_COUNT_THRESHOLD;
    imu_processing.gyro_bias_mean.y /= GYRO_BIAS_SAMPLE_COUNT_THRESHOLD;
    imu_processing.gyro_bias_mean.z /= GYRO_BIAS_SAMPLE_COUNT_THRESHOLD;

    /*�Է����жϷɻ��Ƿ�ֹ���жϷ����Ƿ�����ֵ��Χ��*/
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
        IMU_PROCESSING_DEBUG("�ɻ�δ��ֹ��������У׼ʧ��\r\n");
        return false;
    }
    /*У׼���*/
    imu_processing.gyro_is_calibrated = true;
    IMU_PROCESSING_DEBUG("������У׼���\r\n");
    return true;
}

/**
 * @brief  У׼���ٶ����ݣ�������ٶ���������
 * @note
 * @param  ��
 * @retval ��
 */
static bool IMU_CalibrateAcc(IMU_RawData_t *imu_raw_data)
{
    /*У׼ͬ����Ҫ�ڷɻ���ֹ״̬�½���*/
    if (!imu_processing.gyro_is_calibrated)
    {
        return false;
    }
    /*��������ƽ����*/
    float acc_scale_factor_temp = sqrtf(imu_raw_data->acc.x * imu_raw_data->acc.x +
                                        imu_raw_data->acc.y * imu_raw_data->acc.y +
                                        imu_raw_data->acc.z * imu_raw_data->acc.z) /
                                  MPU6000_ACCEL_SENSITIVITY_2G;
    /*������������*/
    imu_processing.acc_scale_factor += acc_scale_factor_temp;
    imu_processing.acc_sample_count++;
    /*�жϲ������Ƿ�ﵽ��������ֵ*/
    if (imu_processing.acc_sample_count < ACC_SCALE_FACTOR_SAMPLE_COUNT_THRESHOLD)
    {
        return false;
    }
    /*�����ֵ*/
    imu_processing.acc_scale_factor /= ACC_SCALE_FACTOR_SAMPLE_COUNT_THRESHOLD;
    // /*�ж����������Ƿ�����ֵ��Χ��*/
    // if (imu_processing.acc_scale_factor > 1.0f || imu_processing.acc_scale_factor < 0.9f)
    // {
    //     return false;
    // }
    /*У׼���*/
    imu_processing.acc_is_calibrated = true;
    IMU_PROCESSING_DEBUG("���ٶȼ�У׼���\r\n");
    return true;
}


/**
  * @brief  ����ת����xָ���ͷ��z���£���������ϵ����Ӧmpu6000����ϵ��Ҫ������ת
  * @note   
  * @param  before_x: ����ת��ǰx
  * @param  before_y: ����ת��ǰy
  * @param  before_z: ����ת��ǰz
  * @param  after: ת��������
  * @retval ��
  */
static void IMU_Coordinate_Conversion(float before_x,float before_y,float before_z,Axis3f_t *after)
{
    after->x = before_y;
    after->y = -before_x;
    after->z = before_z;
}



/**
 * @brief  imu���ݴ�������У׼imu���ݣ�ת������
 * @note
 * @param  imu_raw_data: ԭʼimu����
 * @param  imu_cal_data: У׼imu����
 * @retval ��
 */
void IMU_Processing(IMU_RawData_t *imu_raw_data, IMU_CalData_t *imu_cal_data)
{
    /*У׼��ת������*/
    /*������*/
    imu_cal_data->gyro.x = ((float)imu_raw_data->gyro.x - imu_processing.gyro_bias_mean.x) * MPU6000_GYRO_CONVERSION_FACTOR;
    imu_cal_data->gyro.y = ((float)imu_raw_data->gyro.y - imu_processing.gyro_bias_mean.y) * MPU6000_GYRO_CONVERSION_FACTOR;
    imu_cal_data->gyro.z = ((float)imu_raw_data->gyro.z - imu_processing.gyro_bias_mean.z) * MPU6000_GYRO_CONVERSION_FACTOR;
    IMU_Coordinate_Conversion(imu_cal_data->gyro.x, imu_cal_data->gyro.y, imu_cal_data->gyro.z, &imu_cal_data->gyro);

    /*���ٶȼ�*/
    imu_cal_data->acc.x = ((float)imu_raw_data->acc.x * MPU6000_ACCEL_CONVERSION_FACTOR) / imu_processing.acc_scale_factor;
    imu_cal_data->acc.y = ((float)imu_raw_data->acc.y * MPU6000_ACCEL_CONVERSION_FACTOR) / imu_processing.acc_scale_factor;
    imu_cal_data->acc.z = ((float)imu_raw_data->acc.z * MPU6000_ACCEL_CONVERSION_FACTOR) / imu_processing.acc_scale_factor;
    IMU_Coordinate_Conversion(imu_cal_data->acc.x, imu_cal_data->acc.y, imu_cal_data->acc.z, &imu_cal_data->acc);
    /*����������˲�ʱ���ֶԽ��ٶ�����y��z������ȡ������֪��Ϊʲô*/
    // printf("%f,", imu_cal_data->acc.x);
    // printf("%f,", imu_cal_data->acc.y);
    // printf("%f,", imu_cal_data->acc.z);
    // printf("%f,", imu_cal_data->gyro.x);
    // printf("%f,", imu_cal_data->gyro.y);
    // printf("%f\n", imu_cal_data->gyro.z);
}

/**
 * @brief  imu���ݴ�����ز�����ʼ��
 * @note
 * @param  ��
 * @retval ��
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
 * @brief  imu���ݳ�ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
void IMU_Processing_Init(IMU_RawData_t *raw_data)
{
    IMU_Processing_Param_Init();
    /*��ȡimu����У��ֵ*/
    while (!imu_processing.gyro_is_calibrated || !imu_processing.acc_is_calibrated)
    {
        MPU6000_GetData(&raw_data->acc, &raw_data->gyro);
        /*��ȡimu����У��ֵ*/
        /*��ȡ������ƫ��*/
        if (!imu_processing.gyro_is_calibrated)
        {
            /*��ȡ������ƫ��*/
            IMU_CalibrateGyro(raw_data);
        }
        /*��ȡ���ٶȼ���������*/
        if (imu_processing.gyro_is_calibrated && !imu_processing.acc_is_calibrated)
        {
            /*��ȡ���ٶȼ���������*/
            IMU_CalibrateAcc(raw_data);
        }
    }
}

/**
 * @}
 */
