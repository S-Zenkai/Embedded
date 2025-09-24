/**
 ******************************************************************************
 * @file    bsp_mpu6000.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/02/28
 * @brief   mpu6000驱动文件
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
 * @brief  写MPU6000寄存器
 * @note
 * @param  reg：寄存器
 *         data：数据
 * @retval 无
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
 * @brief  读MPU6000寄存器
 * @note   注意需要两次发送接收函数
 * @param  reg：寄存器
 * @retval 读到的数据
 */
static uint8_t MPU6000_ReadReg(uint8_t reg)
{
    uint8_t ret;
    MPU6000_SPI_CS_L;
    delay_us(1);
    SPI_TR_Byte(reg | 0x80); /*mpu6000的SPI时序：0是写操作，1是读操作*/
    ret = SPI_TR_Byte(0xff); /*发送一个无效数据*/
    MPU6000_SPI_CS_H;
    delay_us(1); /*注意这里不要忘了，不然后面连续读取mpu数据时引脚无法拉低，导致时序错误无法读取GYRO数据*/
    return ret;
}
#endif

/**
 * @brief  连续读取MPU6000数据(SPI时序，发送首地址后按递增地址读取数据)
 * @note   读取MPU6000最好使用该函数，可以以较快速度读取数据，增加程序实时性
 * @param  buffer：读取数据缓冲区
 * @retval 无
 */
static void MPU6000_ReadData(uint8_t *buf)
{
    uint8_t i;
    MPU6000_SPI_CS_L;
    delay_us(1);
    SPI_TR_Byte(MPU6000_ACCEL_XOUT_H | 0x80); /*mpu6000的SPI时序：0是写操作，1是读操作*/
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
 * @param  无
 * @retval 无
 */
// void mpu6000_csgpio_init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;
//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//     /*MPU6000：CS(PC2)*/
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//     GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
//     GPIO_Init(GPIOC, &GPIO_InitStructure);
//     MPU6000_SPI_CS_H;
// }

/**
 * @brief  MPU6000初始化函数
 * @note
 * @param  无
 * @retval 无
 */
void mpu6000_init(void)
{
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x80); /*复位MPU6000所有寄存器（唤醒前可以写电源管理寄存器）*/
    delay_ms(100);                              /*复位需要一定时间*/
    /*唤醒MPU6000*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x00);
    /*不启动自检，+-2000满量程，灵敏度16.4LSB/°/s，灵敏度越高，量程越小，对角度变化越敏感*/
    /*灵敏度16.4LSB/°/s，测得的数据会是阶梯型的，不像加速度计那样每个测量点都会变化*/
    MPU6000_WriteReg(MPU6000_GYRO_CONFIG, 0x18);
    /*不启动自检，+-2g满量程（小外力时使用2g即可）*/
    MPU6000_WriteReg(MPU6000_ACCEL_CONFIG, 0x00);
    /*使能加速度计和陀螺仪各个轴(可不使能，复位后各个轴便使能了)*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_2, 0x00);
    /*选择时钟源为X轴陀螺仪*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x01);
    /*设置采样率*/
    /*采样率公式：f_T=陀螺仪输出频率/(1+MPU6000_SMPRT_DIV)*/
    /*这里设置1000hz*/
    MPU6000_WriteReg(MPU6000_SMPRT_DIV, 0);
    /*设置数字滤波器（需要根据香农采样定理设置带宽,带宽与延时负相关）*/
    /*这里设置陀螺仪输出频率1khz；带宽,延时：acc:44,4.9,gyro:42,4.8*/
    MPU6000_WriteReg(MPU6000_CONFIG, 0x03);
    MPU6000_WriteReg(MPU6000_INT_PIN_CFG, 0x80);
    /*使能数据就绪中断*/
    MPU6000_WriteReg(MPU6000_INT_ENABLE, 0x01);
}

/**
 * @brief  获取MPU6000测量数据
 * @note
 * @param  MPU6000_Data：MPU6000数据结构体指针
 * @retval 无
 */
void MPU6000_GetData(Axis3_i16_u *acc, Axis3_i16_u *gyro)
{
    uint8_t raw_buf[14];
    /*注意要连续读取mpu6000数据*/
    MPU6000_ReadData(raw_buf);
    acc->x = (int16_t)(raw_buf[0] << 8 | raw_buf[1]);
    acc->y = (int16_t)(raw_buf[2] << 8 | raw_buf[3]);
    acc->z = (int16_t)(raw_buf[4] << 8 | raw_buf[5]);
    gyro->x = (int16_t)(raw_buf[8] << 8 | raw_buf[9]);
    gyro->y = (int16_t)(raw_buf[10] << 8 | raw_buf[11]);
    gyro->z = (int16_t)(raw_buf[12] << 8 | raw_buf[13]);
}

/**
 * @brief  将原始数据转换为实际数据(单位为g和弧度/s)
 * @note
 * @param  无
 * @retval 无
 */
void MPU6000_Data_Conversion(Axis3_f_u *acc, Axis3_f_u *gyro,Axis3_i16_u *acc_raw, Axis3_i16_u *gyro_raw)
{
    /*校准、转换数据*/
    /*陀螺仪*/
    gyro->x = (float)gyro_raw->x * MPU6000_GYRO_CONVERSION_FACTOR;
    gyro->y = (float)gyro_raw->y * MPU6000_GYRO_CONVERSION_FACTOR;
    gyro->z = (float)gyro_raw->z * MPU6000_GYRO_CONVERSION_FACTOR;

    /*加速度计*/
    acc->x = (float)acc_raw->x * MPU6000_ACCEL_CONVERSION_FACTOR;
    acc->y = (float)acc_raw->y * MPU6000_ACCEL_CONVERSION_FACTOR;
    acc->z = (float)acc_raw->z * MPU6000_ACCEL_CONVERSION_FACTOR;
}
