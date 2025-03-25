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

/*mpu6000ԭʼ���ݻ�����*/
int16_t MPU6000Data_Buff[14];
/*���׵�ͨ�˲�������*/
LPF2ordParam_t LPF2ordParam[6];
/*MPU6000����У׼��ؽṹ�����*/
MPU6000CalibrationParam_t calibration_structure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  дMPU6000�Ĵ���
  * @note   
  * @param  reg���Ĵ���
  *         data������
  * @retval ��
  */
void MPU6000_WriteReg(uint8_t reg,uint8_t data)
{
    SPI_CS_L;
    delay_ms(10);
    SPI_TansmissionReceiveByte(reg);
    SPI_TansmissionReceiveByte(data);
    SPI_CS_H;
    delay_ms(10);
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
    SPI_CS_L;
    delay_ms(10);
    SPI_TansmissionReceiveByte(reg|0x80);/*mpu6000��SPIʱ��0��д������1�Ƕ�����*/
    ret = SPI_TansmissionReceiveByte(0xff);/*����һ����Ч����*/
    SPI_CS_H;
    delay_ms(10);/*ע�����ﲻҪ���ˣ���Ȼ����������ȡmpu����ʱ�����޷����ͣ�����ʱ������޷���ȡGYRO����*/
    return ret;
}

/**
  * @brief  ������ȡMPU6000����(SPIʱ�򣬷����׵�ַ�󰴵�����ַ��ȡ����)
  * @note   ��ȡMPU6000���ʹ�øú����������ԽϿ��ٶȶ�ȡ���ݣ����ӳ���ʵʱ��
  * @param  buffer����ȡ���ݻ�����
  * @retval ��
  */
void MPU6000_ReadData(int16_t* buffer)
{
    uint8_t i;
    SPI_CS_L;
    SPI_TansmissionReceiveByte(MPU6000_ACCEL_XOUT_H|0x80);/*mpu6000��SPIʱ��0��д������1�Ƕ�����*/
    for (i = 0; i < 14;i++)
    {
        buffer[i] = SPI_TansmissionReceiveByte(0x00);
    }
    SPI_CS_H;
}

/**
  * @brief  MPU6000��ʼ������
  * @note   
  * @param  ��
  * @retval ��
  */
void mpu6000_init(void)
{
    /*spi��ʼ��*/
    spi_init();
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x80);/*��λMPU6000���мĴ���������ǰ����д��Դ����Ĵ�����*/
    delay_ms(100);/*��λ��Ҫһ��ʱ��*/
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
    /*��������100hz*/
    MPU6000_WriteReg(MPU6000_SMPRT_DIV, 0);
    /*���������˲�������Ҫ������ũ�����������ô���,��������ʱ����أ�*/
    /*�����������������Ƶ��1khz������,��ʱ��acc:44,4.9,gyro:42,4.8*/
    MPU6000_WriteReg(MPU6000_CONFIG, 0x03);

    MPU6000_WriteReg(MPU6000_INT_PIN_CFG, 0x80);
    /*ʹ�����ݾ����ж�*/
    MPU6000_WriteReg(MPU6000_INT_ENABLE, 0x01);
    /*��ͨ�˲���������ʼ��*/
    /*��ͨ����ʼ��������Գ�ʼ�������е��ӳ�Ԫ�������Ⱦ*/
    for (uint8_t i = 0; i < 6;i++)
    {
        LPF2ord_Init(&LPF2ordParam[i], 1000, 30);
    }
    calibration_structure.buffer_pointer = calibration_structure.gyro_bias_buffer;
    calibration_structure.acc_scale_factor = 1;
}



/**
  * @brief  
  * @note   
  * @param  ��
  * @retval ��
  */
static void add_measurement_to_buffer(MPU6000Raw3axis_t* gyro,MPU6000CalibrationParam_t* calibration_structure)
{
    uint16_t ii;
    for (ii = 0; ii < 3;ii++)
    {
        calibration_structure->buffer_pointer->axis[ii] = gyro->axis[ii];
    }
    calibration_structure->buffer_pointer++;
    /*����û����ɣ�*/
    /*���ݴ���ʱ��ͷ��ʼ��*/
    if(calibration_structure->buffer_pointer >= &calibration_structure->gyro_bias_buffer[gyro_bias_buffer_size])
    {
       calibration_structure->buffer_full_flag = true;
        calibration_structure->buffer_pointer = calibration_structure->gyro_bias_buffer;
    }
}

/**
  * @brief  �����׼��ͷ���
  * @note   
  * @param  ��
  * @retval ��
  */

static void calculate_standard_deviation_and_variance(MPU6000CalibrationParam_t* calibration_structure)
{
    uint16_t ii;
    uint8_t jj;
    static float sum[3];
    static float sumsq[3];
    /*�������������ݱ�׼��ͷ���*/
    if(calibration_structure->buffer_full_flag == true)
    {
        for (ii = 0; ii < gyro_bias_buffer_size; ii++)
        {
            for (jj = 0; jj < 3; jj++)
            {
                sum[jj] += calibration_structure->gyro_bias_buffer[ii].axis[jj];/*������*/
                sumsq[jj] += calibration_structure->gyro_bias_buffer[ii].axis[jj] *
                             calibration_structure->gyro_bias_buffer[ii].axis[jj];/*����ƽ����*/
            }
        }
        for (jj = 0; jj < 3;jj++)
        {
            calibration_structure->bias_mean.axis[jj] = sum[jj] / gyro_bias_buffer_size;/*ƽ��ֵ����*/
            calibration_structure->bias_variance.axis[jj] = (sumsq[jj] - (sum[jj] * sum[jj]) / gyro_bias_buffer_size) / (gyro_bias_buffer_size - 1);/*����*/
        }

        if(calibration_structure->bias_variance.x<400&&calibration_structure->bias_variance.y<400&&calibration_structure->bias_variance.z<400)
        {
            calibration_structure->mean_effective_flag = true;/*flagΪtrueʱ������ɻ����ã�����ƽ��ֵ��Ч*/
            calibration_structure->iscalibration_flag = true;
        }
        /*��������������Ա�֤�ڲ�÷ɻ�δ��ֹʱ�����������ظ������ֵ��������ڻ�����ԭ���ݱ����Ǻ��ټ���*/
        else
        {
            calibration_structure->buffer_full_flag = false;
            sum[0] = sum[1] = sum[2] = 0;
            sumsq[0] = sumsq[1] = sumsq[2] = 0;
        }
    }
}

/**
  * @brief  ������ٶ���������
  * @note   ��Ҫ�ڷɻ���ֹʱ������������
  * @param  ��
  * @retval ��
  */
static void calculate_Scale_factor(MPU6000Raw3axis_t* gyro,MPU6000CalibrationParam_t* calibration_structure)
{
    uint8_t jj;
    float acc_sum = 0;
    static float acc_sumsq = 0;
    static uint16_t counter = 0;
    if(counter<400)
    {
        for (jj = 0; jj < 3;jj++)
        {
            acc_sum += ((gyro->axis[jj] / 32768.0f) * 2.0f) * ((gyro->axis[jj] / 32768.0f) * 2.0f);
        }
        acc_sumsq += sqrtf(acc_sum);
        counter++;
    }
    if(counter==400)
    {
        calibration_structure->acc_scale_factor = acc_sumsq / 400;
        acc_sum = 0;
        acc_sumsq = 0;
        counter++;
    }
}

static void process_gyro_bias(MPU6000Data_t *MPU6000_Data,MPU6000CalibrationParam_t* calibration_structure)
{
    /*���²���ֵ���뻺����*/
    add_measurement_to_buffer(&MPU6000_Data->GyroRaw, calibration_structure);
    /*�������������ݱ�׼��ͷ���*/
    if(calibration_structure->iscalibration_flag==false)
    {
        calculate_standard_deviation_and_variance(calibration_structure);
    }
    /*����ƽ��ֵ������Чʱ��������������*/
    if(calibration_structure->mean_effective_flag == true)
    {
        calculate_Scale_factor(&MPU6000_Data->AccRaw, calibration_structure);
    }
}

/**
  * @brief  ��ȡMPU6000��������
  * @note   
  * @param  MPU6000_Data��MPU6000���ݽṹ��ָ��
  * @retval ��
  */
 void MPU6000_GetData(MPU6000Data_t *MPU6000_Data)
 {
     uint8_t i;
     uint8_t j = 0;
     /*ע��Ҫ������ȡmpu6000����*/
     MPU6000_ReadData(MPU6000Data_Buff);
     for (i = 0; i < 6; i++)
     {
         MPU6000_Data->AccRaw.axis[j] = MPU6000Data_Buff[i];
         MPU6000_Data->AccRaw.axis[j] <<= 8;
         i++;
         MPU6000_Data->AccRaw.axis[j] |= MPU6000Data_Buff[i];
         j++;
     }
     j = 0;
     for (i = 8; i < 14; i++)
     {
         MPU6000_Data->GyroRaw.axis[j] = MPU6000Data_Buff[i];
         MPU6000_Data->GyroRaw.axis[j] <<= 8;
         i++;
         MPU6000_Data->GyroRaw.axis[j] |= MPU6000Data_Buff[i];
         j++;
     }
 }

/**
  * @brief  ����MPU6000ԭʼ����
  * @note   
  * @param  MPU6000_Data��MPU6000���ݽṹ��ָ��
  * @retval ��
  */
void MPU6000_ProcessData(MPU6000Data_t *MPU6000_Data)
{
    uint8_t index;
    /*����������ƫ�����*/
    process_gyro_bias(MPU6000_Data,&calibration_structure);

    /*�����趨����ת������*/
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Acc.axis[index] = ((MPU6000_Data->AccRaw.axis[index]/calibration_structure.acc_scale_factor) / 32768.0f) * 2.0f;
        // MPU6000_Data->Acc.axis[index] = ((MPU6000_Data->AccRaw.axis[index]) / 32768.0f) * 2.0f;

    }
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Gyro.axis[index] = ((MPU6000_Data->GyroRaw.axis[index]-calibration_structure.bias_mean.axis[index]) / 32768.0f) * 2000.0f;
        // MPU6000_Data->Gyro.axis[index] = ((MPU6000_Data->GyroRaw.axis[index]) / 32768.0f) * 2000.0f;

    }
    printf("%f,", MPU6000_Data->Acc.x);
    printf("%f,", MPU6000_Data->Acc.y);
    printf("%f,", MPU6000_Data->Acc.z);
    /*���ٶ�����*/
    printf("%f,", MPU6000_Data->Gyro.x);
    printf("%f,", MPU6000_Data->Gyro.y);
    printf("%f,", MPU6000_Data->Gyro.z);
    /*�˲�*/
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Acc.axis[index] = LPF2ord(&LPF2ordParam[index], MPU6000_Data->Acc.axis[index]);
    }
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Gyro.axis[index] = LPF2ord(&LPF2ordParam[index+3], MPU6000_Data->Gyro.axis[index]);
    }
    printf("%f,", MPU6000_Data->Acc.x);
    printf("%f,", MPU6000_Data->Acc.y);
    printf("%f,", MPU6000_Data->Acc.z);
    /*���ٶ�����*/
    printf("%f,", MPU6000_Data->Gyro.x);
    printf("%f,", MPU6000_Data->Gyro.y);
    printf("%f\n", MPU6000_Data->Gyro.z);
}
