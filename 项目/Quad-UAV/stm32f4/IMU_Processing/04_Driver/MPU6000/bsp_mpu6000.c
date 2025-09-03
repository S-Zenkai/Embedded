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


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  дMPU6000�Ĵ���
 * @note
 * @param  reg���Ĵ���
 *         data������
 * @retval ��
 */
static void MPU6000_WriteReg(uint8_t reg, uint8_t data)
{
    MPU6000_SPI_CS_L;
    delay_us(1);
    SPI_TR_Byte(reg);
    SPI_TR_Byte(data);
    MPU6000_SPI_CS_H;
    delay_us(1);
}

#if 0
/**
 * @brief  ��MPU6000�Ĵ���
 * @note   ע����Ҫ���η��ͽ��պ���
 * @param  reg���Ĵ���
 * @retval ����������
 */
static uint8_t MPU6000_ReadReg(uint8_t reg)
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
#endif

/**
 * @brief  ������ȡMPU6000����(SPIʱ�򣬷����׵�ַ�󰴵�����ַ��ȡ����)
 * @note   ��ȡMPU6000���ʹ�øú����������ԽϿ��ٶȶ�ȡ���ݣ����ӳ���ʵʱ��
 * @param  buffer����ȡ���ݻ�����
 * @retval ��
 */
static void MPU6000_ReadData(uint8_t *buf)
{
    uint8_t i;
    MPU6000_SPI_CS_L;
    delay_us(1);
    SPI_TR_Byte(MPU6000_ACCEL_XOUT_H | 0x80); /*mpu6000��SPIʱ��0��д������1�Ƕ�����*/
    for (i = 0; i < 14; i++)
    {
        buf[i] = SPI_TR_Byte(0x00);
    }
    MPU6000_SPI_CS_H;
    delay_us(1);
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
    /*�������Լ죬+-2000�����̣�������16.4LSB/��/s��������Խ�ߣ�����ԽС���ԽǶȱ仯Խ����*/
    /*������16.4LSB/��/s����õ����ݻ��ǽ����͵ģ�������ٶȼ�����ÿ�������㶼��仯*/
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
}

/**
 * @brief  ��ȡMPU6000��������
 * @note
 * @param  MPU6000_Data��MPU6000���ݽṹ��ָ��
 * @retval ��
 */
void MPU6000_GetData(Axis3i16_t *acc, Axis3i16_t *gyro)
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
