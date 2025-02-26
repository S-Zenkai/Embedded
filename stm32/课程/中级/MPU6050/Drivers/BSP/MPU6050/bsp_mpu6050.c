/**
 * *****************************************************************************
 * @file        bsp_mpu6050.c
 * @brief       
 * @author      
 * @date        2024-12-05
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:    stm32f103vet6
 * 
 * *****************************************************************************
 */
/*----------------------------------include-----------------------------------*/
#include "bsp_mpu6050.h"
#include "bsp_iic.h"
#include "bsp_systick.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief       ��MPU6050�Ĵ���дһ������
 * 
 * @param       reg �Ĵ�����ַ
 * @param       data Ҫд������
 */
void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    /*Ӧ��ʹ��һ����������δ�յ�Ӧ���źŵ��������������whileѭ�����������ʱ�����*/
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    IIC_SendByte(data);
    IIC_ReceiveAck();
    IIC_Stop();
}

/**
 * @brief       ��MPU6050�Ĵ�������д����
 * 
 * @param       reg �Ĵ�����ַ
 * @param       databuff Ҫд�����ݻ�����ָ��
 * @param       buffsz ���ݻ�������С
 */
void MPU6050_WriteRegContinue(uint8_t reg, uint8_t *databuff, uint16_t buffsz)
{
    uint16_t i;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    for (i = 0; i < buffsz;i++)
    {
        IIC_SendByte(databuff[i]);
        IIC_ReceiveAck();
    }
    IIC_Stop();
}

/**
 * @brief       ��MPU6050�Ĵ���
 * 
 * @param       reg �Ĵ�����ַ
 * @return      uint8_t ����������
 */
uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t ret;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    /*ע�⣬���ʹ��while����Ӧ���źţ��޷�����ͨ�ţ��²��������Ϊ�޷�����ʱ��Ҫ��*/
    // while (IIC_ReceiveAck() == IIC_ASK)
    //     ;
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    IIC_Start();
    IIC_SendByte(MPU6050_ADR);
    IIC_ReceiveAck();
    ret = IIC_ReceiveByte();
    IIC_SendAck(IIC_NASK);
    IIC_Stop();
    return ret;
}

/**
 * @brief       ������MPU6050�Ĵ���
 *
 * @param       reg �Ĵ�����ַ
 * @param       databuff    ���ݻ�����
 * @param       buffsz      ���ݻ�������С
 */
void MPU6050_ReadRegContinue(uint8_t reg, uint8_t* databuff,uint16_t buffsz)
{
    uint8_t i = 0;
    IIC_Start();
    IIC_SendByte(MPU6050_ADW);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    IIC_Start();
    IIC_SendByte(MPU6050_ADR);
    IIC_ReceiveAck();
    for (i = 0; i < buffsz-1;i++)
    {
        databuff[i] = IIC_ReceiveByte();
        IIC_SendAck(IIC_ASK);
    }
    databuff[buffsz - 1] = IIC_ReceiveByte();
    IIC_SendAck(IIC_NASK);
    IIC_Stop();
}

/**
 * @brief       MPU6050��ʼ��������IICͨ�����ų�ʼ��������MPU6050���������̡����ò����ʡ����������˲���
 * 
 */
// void MPU6050Init(void)
// {
//     IICInit();
//     MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);/*����MPU6050��ʱ��ԴΪX��������*/
//     MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); 
//     MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);/*�������Լ죬+-2000������*/
//     MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); /*�������Լ죬+-2000������*/
//     MPU6050_WriteReg(MPU6050_SMPRT_DIV, 100);/*���ò�����*/
//     MPU6050_WriteReg(MPU6050_CONFIG, 100);   /*���������˲���*/
// }

void MPU6050Init(void)
{
    IICInit();
    /*��λMPU6050���мĴ���������ǰ����д��Դ����Ĵ�����*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);/*��λMPU6050���мĴ���������ǰ����д��Դ����Ĵ�����*/
    delay_ms(100);/*��λ��Ҫһ��ʱ��*/
    /*����MPU6050*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    /*�������Լ죬+-2000������*/
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    /*�������Լ죬+-2g�����̣�С����ʱʹ��2g���ɣ�*/
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
    /*ʹ�ܼ��ٶȼƺ������Ǹ�����(�ɲ�ʹ�ܣ���λ��������ʹ����)*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    /*ѡ��ʱ��ԴΪX��������*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    /*���ò�����(MPU6050��������СΪ4HZ�����Ϊ8KHZ)*/
    /*�����ʹ�ʽ��f=...*/
    MPU6050_WriteReg(MPU6050_SMPRT_DIV, 9);
    /*���������˲�������Ҫ������ũ�����������ô���*/
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);
}

/**
 * @brief       ��ȡMPU6050�������ݣ�ע�⣬��ȡ������Ϊ2���Ʋ���ֵ��
 *
 * @param       MPU6050_DataStruct ָ��MPU6050_DataTypedef�ṹ���ָ�룬�ýṹ�����MPU6050��������
 */
void MPU6050GetData(MPU6050_DataTypedef* MPU6050_DataStruct)
{
    MPU6050_DataStruct->ACCEL_XOUT = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    MPU6050_DataStruct->ACCEL_XOUT <<= 8;
    MPU6050_DataStruct->ACCEL_XOUT |= MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);

    MPU6050_DataStruct->ACCEL_YOUT = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    MPU6050_DataStruct->ACCEL_YOUT <<= 8;
    MPU6050_DataStruct->ACCEL_YOUT |= MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);

    MPU6050_DataStruct->ACCEL_ZOUT = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    MPU6050_DataStruct->ACCEL_ZOUT <<= 8;
    MPU6050_DataStruct->ACCEL_ZOUT |= MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);

    MPU6050_DataStruct->GYRO_XOUT = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    MPU6050_DataStruct->GYRO_XOUT <<= 8;
    MPU6050_DataStruct->GYRO_XOUT |= MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);

    MPU6050_DataStruct->GYRO_YOUT = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    MPU6050_DataStruct->GYRO_YOUT <<= 8;
    MPU6050_DataStruct->GYRO_YOUT |= MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);

    MPU6050_DataStruct->GYRO_ZOUT = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    MPU6050_DataStruct->GYRO_ZOUT <<= 8;
    MPU6050_DataStruct->GYRO_ZOUT |= MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
}
    /*------------------------------------test------------------------------------*/
