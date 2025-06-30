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
void MPU6000_ReadData(uint8_t* buf)
{
    uint8_t i;
    SPI_CS_L;
    SPI_TansmissionReceiveByte(MPU6000_ACCEL_XOUT_H|0x80);/*mpu6000��SPIʱ��0��д������1�Ƕ�����*/
    for (i = 0; i < 14;i++)
    {
        buf[i] = SPI_TansmissionReceiveByte(0x00);
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
    for (uint8_t i = 0; i < 3;i++)
    {
        LPF2ord_Init(&gyro_LPF2Param[i], 1000, 80);
        LPF2ord_Init(&acc_LPF2Param[i], 1000, 30);
    }

    calibration.acc_scale = 1.0f;
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
     uint8_t raw_buf[14];
     /*ע��Ҫ������ȡmpu6000����*/
     MPU6000_ReadData(raw_buf);
     for (i = 0; i < 6; i++)
     {
         MPU6000_Data->AccRaw.axis[j] = raw_buf[i];
         MPU6000_Data->AccRaw.axis[j] <<= 8;
         i++;
         MPU6000_Data->AccRaw.axis[j] |= raw_buf[i];
         j++;
     }
     j = 0;
     for (i = 8; i < 14; i++)
     {
          MPU6000_Data->GyroRaw.axis[j] = raw_buf[i];
         MPU6000_Data->GyroRaw.axis[j] <<= 8;
          i++;
          MPU6000_Data->GyroRaw.axis[j] |= raw_buf[i];
         j++;
     }
		 printf("%hd,",MPU6000_Data->AccRaw.x);
     printf("%hd,",MPU6000_Data->AccRaw.y);
     printf("%hd,",MPU6000_Data->AccRaw.z);
     /*���ٶ�����*/
     printf("%hd,",MPU6000_Data->GyroRaw.x);
     printf("%hd,",MPU6000_Data->GyroRaw.y);
     printf("%hd\n",MPU6000_Data->GyroRaw.z);
 }

static bool calculate_gyro(MPU6000Data_t *data)
{
    uint8_t i;
    float mean;/*ƽ��ֵ*/
    float var;/*����*/
    for (i = 0; i < 3;i++)
    {
        calibration.bias_sum.axis[i] += data->GyroRaw.axis[i];
        calibration.bias_sumsq.axis[i] += data->GyroRaw.axis[i] * data->GyroRaw.axis[i];
    }
    calibration.sample_count++;
    if(calibration.sample_count<gyro_calibration_simple)
        return false;
    for (i = 0; i < 3; i++)
    {
        mean = calibration.bias_sum.axis[i] / gyro_calibration_simple;
        var = ((float)calibration.bias_sumsq.axis[i] -
               (float)(calibration.bias_sum.axis[i] * calibration.bias_sum.axis[i]) / gyro_calibration_simple) /
              (gyro_calibration_simple - 1);
        if(var>400)
        {
            calibration.sample_count = 0;
            calibration.bias_sum.axis[i] = 0;
            calibration.bias_sumsq.axis[i] = 0;
            return false;
        }
    }
    calibration.sample_count = 0;/*֮�����ڼ��ٶ�У׼*/
    return true;
}

static bool calculate_acc(MPU6000Data_t *data)
{
    uint8_t i;
    float acc_sum;
    for(i = 0; i < 3;i++)
    {
        acc_sum += (data->AccRaw.axis[i] / acc_range_2g) * (data->AccRaw.axis[i] / acc_range_2g);
    }
    calibration.acc_sumsq += sqrtf(acc_sum);
    calibration.sample_count++;
    if(calibration.sample_count<acc_calibration_simple)
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
void MPU6000_ProcessData(MPU6000Data_t *MPU6000_Data)
{
    uint8_t index;
    /*У׼�����Ǻͼ��ٶȼ�*/
    if(!calibration.gyro_calibration_done)
    {
        if(calculate_gyro(MPU6000_Data))
        {
            calibration.gyro_calibration_done = true;
            calibration.acc_sumsq = 0;
        }
    }
    if(calibration.gyro_calibration_done&&(!calibration.acc_calibration_done))
    {
        if(calculate_acc(MPU6000_Data))
        {
            calibration.acc_calibration_done = true;
        }
    }
    /*�����趨����ת������*/
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Acc.axis[index] = (MPU6000_Data->AccRaw.axis[index] / acc_range_2g) / calibration.acc_scale;
    }
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Gyro.axis[index] = (MPU6000_Data->GyroRaw.axis[index] - calibration.bias_sum.axis[index] / gyro_calibration_simple) / gyro_range_2000;
    }
//    printf("%f,", MPU6000_Data->Acc.x);
//    printf("%f,", MPU6000_Data->Acc.y);
//    printf("%f,", MPU6000_Data->Acc.z);
//    /*���ٶ�����*/
//    printf("%f,", MPU6000_Data->Gyro.x);
//    printf("%f,", MPU6000_Data->Gyro.y);
//    printf("%f,", MPU6000_Data->Gyro.z);
    /*�˲�*/
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Acc.axis[index] = LPF2ord(&acc_LPF2Param[index], MPU6000_Data->Acc.axis[index]);
        MPU6000_Data->Gyro.axis[index] = LPF2ord(&gyro_LPF2Param[index], MPU6000_Data->Gyro.axis[index]);
    }
    // for (index = 0; index < 3; index++)
    // {
    //     MPU6000_Data->Gyro.axis[index] = LPF2ord(&gyro_LPF2Param[index], MPU6000_Data->Gyro.axis[index]);
    // }
//    printf("%f,", MPU6000_Data->Acc.x);
//    printf("%f,", MPU6000_Data->Acc.y);
//    printf("%f,", MPU6000_Data->Acc.z);
//    /*���ٶ�����*/
//    printf("%f,", MPU6000_Data->Gyro.x);
//    printf("%f,", MPU6000_Data->Gyro.y);
//    printf("%f\n", MPU6000_Data->Gyro.z);
}
