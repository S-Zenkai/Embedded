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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*mpu6000ԭʼ���ݻ�����*/
int16_t MPU6000Data_Buff[14];
/*���׵�ͨ�˲�������*/
LPF2ordParam_t LPF2ordParam[6];

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
void MPU6000_ProcessingData(MPU6000Data_t *MPU6000_Data)
{
    uint8_t index;

    /*�����趨����ת������*/
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Acc.axis[index] = (MPU6000_Data->AccRaw.axis[index] / 32768.0f) * 2.0f;
    }
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Gyro.axis[index] = (MPU6000_Data->GyroRaw.axis[index] / 32768.0f) * 2000.0f;
    }
    // printf("%f,", MPU6000_Data->Acc.x);
    // printf("%f,", MPU6000_Data->Acc.y);
    // printf("%f,", MPU6000_Data->Acc.z);
    // /*���ٶ�����*/
    // printf("%f,", MPU6000_Data->Gyro.x);
    // printf("%f,", MPU6000_Data->Gyro.y);
    // printf("%f,", MPU6000_Data->Gyro.z);
    /*�˲�*/
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Acc.axis[index] = LPF2ord(&LPF2ordParam[index], MPU6000_Data->Acc.axis[index]);
    }
    for (index = 0; index < 3; index++)
    {
        MPU6000_Data->Gyro.axis[index] = LPF2ord(&LPF2ordParam[index+3], MPU6000_Data->Gyro.axis[index]);
    }
    // printf("%f,", MPU6000_Data->Acc.x);
    // printf("%f,", MPU6000_Data->Acc.y);
    // printf("%f,", MPU6000_Data->Acc.z);
    // /*���ٶ�����*/
    // printf("%f,", MPU6000_Data->Gyro.x);
    // printf("%f,", MPU6000_Data->Gyro.y);
    // printf("%f\n", MPU6000_Data->Gyro.z);
}
