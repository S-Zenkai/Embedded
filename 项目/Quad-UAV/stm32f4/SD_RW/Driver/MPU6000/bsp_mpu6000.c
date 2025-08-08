/**
 ******************************************************************************
 * @file    bsp_mpu6000.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/02/28
 * @brief   mpu6000�����ļ�
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "bsp_mpu6000.h"
#include "bsp_spi.h"
#include "bsp_systick.h"
#include "filter.h"
#include "bsp_usart.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*���׵�ͨ�˲�������*/
LPF2ordParam_t gyro_LPF2Param[3];
LPF2ordParam_t acc_LPF2Param[3];
/*MPU6000����У׼��ؽṹ�����*/
static MPU6000Calibration_t calibration;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  дMPU6000�Ĵ���
 * @note
 * @param  reg���Ĵ���
 *         data������
 * @retval ��
 */
void MPU6000_WriteReg(uint8_t reg, uint8_t data)
{
    MPU6000_SPI_CS_L;
    delay_us(1);
    SPI_TR_Byte(reg);
    SPI_TR_Byte(data);
    MPU6000_SPI_CS_H;
    delay_us(1);
}

/**
 * @brief  дMPU6000�Ĵ���
 * @note   ע����Ҫ���η��ͽ��պ���
 * @param  reg���Ĵ���
 * @retval ����������
 */
uint8_t MPU6000_ReadReg(uint8_t reg)
{
    uint8_t ret;
    MPU6000_SPI_CS_L;
    delay_us(1);
    SPI_TR_Byte(reg | 0x80); /*mpu6000��SPIʱ��0��д������1�Ƕ�����*/
    ret = SPI_TR_Byte(0xff); /*����һ����Ч����*/
    MPU6000_SPI_CS_H;
    delay_us(1); /*ע�����ﲻҪ���ˣ���Ȼ����������ȡmpu����ʱ�����޷����ͣ�����ʱ������޷���ȡGYRO����*/
    return ret;
}

/**
 * @brief  ������ȡMPU6000����(SPIʱ�򣬷����׵�ַ�󰴵�����ַ��ȡ����)
 * @note   ��ȡMPU6000���ʹ�øú����������ԽϿ��ٶȶ�ȡ���ݣ����ӳ���ʵʱ��
 * @param  buffer����ȡ���ݻ�����
 * @retval ��
 */
void MPU6000_ReadData(uint8_t *buf)
{
    uint8_t i;
    MPU6000_SPI_CS_L;
    SPI_TR_Byte(MPU6000_ACCEL_XOUT_H | 0x80); /*mpu6000��SPIʱ��0��д������1�Ƕ�����*/
    for (i = 0; i < 14; i++)
    {
        buf[i] = SPI_TR_Byte(0x00);
    }
    MPU6000_SPI_CS_H;
}

/**
 * @brief
 * @note
 * @param  ��
 * @retval ��
 */
// void mpu6000_csgpio_init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;
//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//     /*MPU6000��CS(PC2)*/
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//     GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
//     GPIO_Init(GPIOC, &GPIO_InitStructure);
//     MPU6000_SPI_CS_H;
// }

/**
 * @brief  MPU6000��ʼ������
 * @note
 * @param  ��
 * @retval ��
 */
void mpu6000_init(void)
{
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x80); /*��λMPU6000���мĴ���������ǰ����д��Դ����Ĵ�����*/
    delay_ms(100);                              /*��λ��Ҫһ��ʱ��*/
    /*����MPU6000*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x00);
    /*�������Լ죬+-2000������*/
    MPU6000_WriteReg(MPU6000_GYRO_CONFIG, 0x18);
    /*�������Լ죬+-2g�����̣�С����ʱʹ��2g���ɣ�*/
    MPU6000_WriteReg(MPU6000_ACCEL_CONFIG, 0x00);
    /*ʹ�ܼ��ٶȼƺ������Ǹ�����(�ɲ�ʹ�ܣ���λ��������ʹ����)*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_2, 0x00);
    /*ѡ��ʱ��ԴΪX��������*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x01);
    /*���ò�����*/
    /*�����ʹ�ʽ��f_T=���������Ƶ��/(1+MPU6000_SMPRT_DIV)*/
    /*��������1000hz*/
    MPU6000_WriteReg(MPU6000_SMPRT_DIV, 0);
    /*���������˲�������Ҫ������ũ�����������ô���,��������ʱ����أ�*/
    /*�����������������Ƶ��1khz������,��ʱ��acc:44,4.9,gyro:42,4.8*/
    MPU6000_WriteReg(MPU6000_CONFIG, 0x03);
    MPU6000_WriteReg(MPU6000_INT_PIN_CFG, 0x80);
    /*ʹ�����ݾ����ж�*/
    MPU6000_WriteReg(MPU6000_INT_ENABLE, 0x01);
    /*��ͨ�˲���������ʼ��*/
    /*��ͨ����ʼ��������Գ�ʼ�������е��ӳ�Ԫ�������Ⱦ*/
    for (uint8_t i = 0; i < 3; i++)
    {
        LPF2ord_Init(&gyro_LPF2Param[i], 1000, 80);
        LPF2ord_Init(&acc_LPF2Param[i], 1000, 30);
    }

    /*δУ׼ʱ��������������1*/
    calibration.acc_scale = 1.0f;
}

/**
 * @brief  ��ȡMPU6000��������
 * @note
 * @param  MPU6000_Data��MPU6000���ݽṹ��ָ��
 * @retval ��
 */
void MPU6000_GetData(MPU6000_RawData_t *acc, MPU6000_RawData_t *gyro)
{
    uint8_t raw_buf[14];
    /*ע��Ҫ������ȡmpu6000����*/
    MPU6000_ReadData(raw_buf);
    acc->x = (int16_t)(raw_buf[0] << 8 | raw_buf[1]);
    acc->y = (int16_t)(raw_buf[2] << 8 | raw_buf[3]);
    acc->z = (int16_t)(raw_buf[4] << 8 | raw_buf[5]);
    gyro->x = (int16_t)(raw_buf[8] << 8 | raw_buf[9]);
    gyro->y = (int16_t)(raw_buf[10] << 8 | raw_buf[11]);
    gyro->z = (int16_t)(raw_buf[12] << 8 | raw_buf[13]);
}

/**
 * @brief  У׼������ƫ������ȡƫ��������ֵ��ֵ�ͷ���
 * @note   ͨ�������С�жϷɻ��Ƿ��ھ�ֹ״̬
 * @param  data��MPU6000���ݽṹ��
 * @retval ����ֵ���Ƿ�У׼�ɹ���false or true
 */
static bool calculate_gyro(MPU6000_RawData_t *gyro)
{
    //    float mean;/*ƽ��ֵ*/
    MPU6000_RawData_t var; /*����*/
    /*�������ֵ�ͼ�ƽ����*/
    calibration.bias_sum.x += gyro->x;
    calibration.bias_sum.y += gyro->y;
    calibration.bias_sum.z += gyro->z;
    calibration.bias_sumsq.x += gyro->x * gyro->x;
    calibration.bias_sumsq.y += gyro->y * gyro->y;
    calibration.bias_sumsq.z += gyro->z * gyro->z;
    calibration.sample_count++;
    /*δ���趨����������false*/
    if (calibration.sample_count < gyro_calibration_simple)
        return false;
    /*�����ֵ�ͷ���趨������ֵ�жϷɻ���ֹ״̬*/
    var.x = ((float)calibration.bias_sumsq.x -
             (float)(calibration.bias_sum.x * calibration.bias_sum.x) / gyro_calibration_simple) /
            (gyro_calibration_simple - 1);
    var.y = ((float)calibration.bias_sumsq.y -
             (float)(calibration.bias_sum.y * calibration.bias_sum.y) / gyro_calibration_simple) /
            (gyro_calibration_simple - 1);
    var.z = ((float)calibration.bias_sumsq.z -
             (float)(calibration.bias_sum.z * calibration.bias_sum.z) / gyro_calibration_simple) /
            (gyro_calibration_simple - 1);
    /*�������ֵ�������ģ�����ʵ����֤*/
    if (var.x > 4 || var.y > 4 || var.z > 4)
    {
        calibration.sample_count = 0;
        calibration.bias_sum.x = 0;
        calibration.bias_sum.y = 0;
        calibration.bias_sum.z = 0;
        calibration.bias_sumsq.x = 0;
        calibration.bias_sumsq.y = 0;
        calibration.bias_sumsq.z = 0;
        return false;
    }
    calibration.sample_count = 0; /*֮�����ڼ��ٶ�У׼*/
    return true;
}

/**
 * @brief  У׼���ٶȼ�����ȡ������������
 * @note   ��Ҫ�ڷɻ���ֹ�����У׼
 * @param  data��MPU6000���ݽṹ��
 * @retval ����ֵ���Ƿ�У׼�ɹ���false or true
 */
static bool calculate_acc(MPU6000_RawData_t *acc)
{
    float acc_sum;
    /*��������ƽ���ͣ����������Ӧ=g��ͨ����������ƽ���͸���ƽ��ֵ�õ��������ӣ�У׼���ٶȼ�*/
    acc_sum = (acc->x / acc_range_2g) * (acc->x / acc_range_2g)
        + (acc->y / acc_range_2g) * (acc->y / acc_range_2g)
        + (acc->z / acc_range_2g) * (acc->z / acc_range_2g);
    calibration.acc_sumsq += sqrtf(acc_sum);
    calibration.sample_count++;
    if (calibration.sample_count < acc_calibration_simple)
        return false;
    calibration.acc_scale = calibration.acc_sumsq / acc_calibration_simple;
    return true;
}
/**
 * @brief  ����MPU6000ԭʼ����
 * @note
 * @param  MPU6000_Data��MPU6000���ݽṹ��ָ��
 * @retval ��
 */
void MPU6000_ProcessData(Axis3i16_t *acc_raw, Axis3i16_t *gyro_raw, Axis3f_t *acc_cal, Axis3f_t *gyro_cal)
{
    /*У׼�����Ǻͼ��ٶȼ�*/
    if (!calibration.gyro_calibration_done)
    {
        if (calculate_gyro(gyro_raw))
        {
            calibration.gyro_calibration_done = true;
            calibration.acc_sumsq = 0;
        }
    }
    if (calibration.gyro_calibration_done && (!calibration.acc_calibration_done))
    {
        if (calculate_acc(acc_raw))
        {
            calibration.acc_calibration_done = true;
        }
    }
    /*�����趨����ת������*/
    acc_cal->x = (acc_raw->x / acc_range_2g) / calibration.acc_scale;
    acc_cal->y = (acc_raw->y / acc_range_2g) / calibration.acc_scale;
    acc_cal->z = (acc_raw->z / acc_range_2g) / calibration.acc_scale;
    gyro_cal->x = (gyro_raw->x - calibration.bias_sum.x / gyro_calibration_simple) / gyro_range_2000;
    gyro_cal->y = (gyro_raw->y - calibration.bias_sum.y / gyro_calibration_simple) / gyro_range_2000;
    gyro_cal->z = (gyro_raw->z - calibration.bias_sum.z / gyro_calibration_simple) / gyro_range_2000;

    // printf("%f,", MPU6000_Data->Acc.x);
    // printf("%f,", MPU6000_Data->Acc.y);
    // printf("%f,", MPU6000_Data->Acc.z);
    // /*���ٶ�����*/
    // printf("%f,", MPU6000_Data->Gyro.x);
    // printf("%f,", MPU6000_Data->Gyro.y);
    // printf("%f,", MPU6000_Data->Gyro.z);
    /*�˲�*/
    acc_cal->x = LPF2ord(&acc_LPF2Param[0], acc_cal->x);
    acc_cal->y = LPF2ord(&acc_LPF2Param[1], acc_cal->y);
    acc_cal->z = LPF2ord(&acc_LPF2Param[2], acc_cal->z);
    gyro_cal->x = LPF2ord(&gyro_LPF2Param[0], gyro_cal->x);
    gyro_cal->y = LPF2ord(&gyro_LPF2Param[1], gyro_cal->y);
    gyro_cal->z = LPF2ord(&gyro_LPF2Param[2], gyro_cal->z);
    // for (index = 0; index < 3; index++)
    // {
    //     MPU6000_Data->Gyro.axis[index] = LPF2ord(&gyro_LPF2Param[index], MPU6000_Data->Gyro.axis[index]);
    // }
    // printf("%f,", MPU6000_Data->Acc.x);
    // printf("%f,", MPU6000_Data->Acc.y);
    // printf("%f,", MPU6000_Data->Acc.z);
    // /*���ٶ�����*/
    // printf("%f,", MPU6000_Data->Gyro.x);
    // printf("%f,", MPU6000_Data->Gyro.y);
    // printf("%f\n", MPU6000_Data->Gyro.z);
}
