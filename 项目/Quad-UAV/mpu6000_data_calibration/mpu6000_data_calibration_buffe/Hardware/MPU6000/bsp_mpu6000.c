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

/*mpu6000原始数据缓冲区*/
int16_t MPU6000Data_Buff[14];
/*二阶低通滤波器参数*/
LPF2ordParam_t LPF2ordParam[6];
/*MPU6000数据校准相关结构体变量*/
MPU6000CalibrationParam_t calibration_structure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  写MPU6000寄存器
  * @note   
  * @param  reg：寄存器
  *         data：数据
  * @retval 无
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
  * @brief  写MPU6000寄存器
  * @note   注意需要两次发送接收函数
  * @param  reg：寄存器
  * @retval 读到的数据
  */
uint8_t MPU6000_ReadReg(uint8_t reg)
{
    uint8_t ret;
    SPI_CS_L;
    delay_ms(10);
    SPI_TansmissionReceiveByte(reg|0x80);/*mpu6000的SPI时序：0是写操作，1是读操作*/
    ret = SPI_TansmissionReceiveByte(0xff);/*发送一个无效数据*/
    SPI_CS_H;
    delay_ms(10);/*注意这里不要忘了，不然后面连续读取mpu数据时引脚无法拉低，导致时序错误无法读取GYRO数据*/
    return ret;
}

/**
  * @brief  连续读取MPU6000数据(SPI时序，发送首地址后按递增地址读取数据)
  * @note   读取MPU6000最好使用该函数，可以以较快速度读取数据，增加程序实时性
  * @param  buffer：读取数据缓冲区
  * @retval 无
  */
void MPU6000_ReadData(int16_t* buffer)
{
    uint8_t i;
    SPI_CS_L;
    SPI_TansmissionReceiveByte(MPU6000_ACCEL_XOUT_H|0x80);/*mpu6000的SPI时序：0是写操作，1是读操作*/
    for (i = 0; i < 14;i++)
    {
        buffer[i] = SPI_TansmissionReceiveByte(0x00);
    }
    SPI_CS_H;
}

/**
  * @brief  MPU6000初始化函数
  * @note   
  * @param  无
  * @retval 无
  */
void mpu6000_init(void)
{
    /*spi初始化*/
    spi_init();
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x80);/*复位MPU6000所有寄存器（唤醒前可以写电源管理寄存器）*/
    delay_ms(100);/*复位需要一定时间*/
    /*唤醒MPU6000*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x00);
    /*不启动自检，+-2000满量程*/
    MPU6000_WriteReg(MPU6000_GYRO_CONFIG, 0x18);
    /*不启动自检，+-2g满量程（小外力时使用2g即可）*/
    MPU6000_WriteReg(MPU6000_ACCEL_CONFIG, 0x00);
    /*使能加速度计和陀螺仪各个轴(可不使能，复位后各个轴便使能了)*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_2, 0x00);
    /*选择时钟源为X轴陀螺仪*/
    MPU6000_WriteReg(MPU6000_PWR_MGMT_1, 0x01);
    /*设置采样率*/
    /*采样率公式：f_T=陀螺仪输出频率/(1+MPU6000_SMPRT_DIV)*/
    /*这里设置100hz*/
    MPU6000_WriteReg(MPU6000_SMPRT_DIV, 0);
    /*设置数字滤波器（需要根据香农采样定理设置带宽,带宽与延时负相关）*/
    /*这里设置陀螺仪输出频率1khz；带宽,延时：acc:44,4.9,gyro:42,4.8*/
    MPU6000_WriteReg(MPU6000_CONFIG, 0x03);

    MPU6000_WriteReg(MPU6000_INT_PIN_CFG, 0x80);
    /*使能数据就绪中断*/
    MPU6000_WriteReg(MPU6000_INT_ENABLE, 0x01);
    /*低通滤波器参数初始化*/
    /*分通道初始化，避免对初始化变量中的延迟元素耦合污染*/
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
  * @param  无
  * @retval 无
  */
static void add_measurement_to_buffer(MPU6000Raw3axis_t* gyro,MPU6000CalibrationParam_t* calibration_structure)
{
    uint16_t ii;
    for (ii = 0; ii < 3;ii++)
    {
        calibration_structure->buffer_pointer->axis[ii] = gyro->axis[ii];
    }
    calibration_structure->buffer_pointer++;
    /*这里没问题吧？*/
    /*数据存满时从头开始存*/
    if(calibration_structure->buffer_pointer >= &calibration_structure->gyro_bias_buffer[gyro_bias_buffer_size])
    {
       calibration_structure->buffer_full_flag = true;
        calibration_structure->buffer_pointer = calibration_structure->gyro_bias_buffer;
    }
}

/**
  * @brief  计算标准差和方差
  * @note   
  * @param  无
  * @retval 无
  */

static void calculate_standard_deviation_and_variance(MPU6000CalibrationParam_t* calibration_structure)
{
    uint16_t ii;
    uint8_t jj;
    static float sum[3];
    static float sumsq[3];
    /*计算陀螺仪数据标准差和方差*/
    if(calibration_structure->buffer_full_flag == true)
    {
        for (ii = 0; ii < gyro_bias_buffer_size; ii++)
        {
            for (jj = 0; jj < 3; jj++)
            {
                sum[jj] += calibration_structure->gyro_bias_buffer[ii].axis[jj];/*样本和*/
                sumsq[jj] += calibration_structure->gyro_bias_buffer[ii].axis[jj] *
                             calibration_structure->gyro_bias_buffer[ii].axis[jj];/*样本平方和*/
            }
        }
        for (jj = 0; jj < 3;jj++)
        {
            calibration_structure->bias_mean.axis[jj] = sum[jj] / gyro_bias_buffer_size;/*平均值计算*/
            calibration_structure->bias_variance.axis[jj] = (sumsq[jj] - (sum[jj] * sum[jj]) / gyro_bias_buffer_size) / (gyro_bias_buffer_size - 1);/*方差*/
        }

        if(calibration_structure->bias_variance.x<400&&calibration_structure->bias_variance.y<400&&calibration_structure->bias_variance.z<400)
        {
            calibration_structure->mean_effective_flag = true;/*flag为true时，代表飞机静置，采样平均值有效*/
            calibration_structure->iscalibration_flag = true;
        }
        /*下面这种情况可以保证在测得飞机未静止时，不必立即重复计算均值方差，而是在缓冲区原数据被覆盖后再计算*/
        else
        {
            calibration_structure->buffer_full_flag = false;
            sum[0] = sum[1] = sum[2] = 0;
            sumsq[0] = sumsq[1] = sumsq[2] = 0;
        }
    }
}

/**
  * @brief  计算加速度缩放因子
  * @note   需要在飞机静止时计算缩放因子
  * @param  无
  * @retval 无
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
    /*将新测量值放入缓冲区*/
    add_measurement_to_buffer(&MPU6000_Data->GyroRaw, calibration_structure);
    /*计算陀螺仪数据标准差和方差*/
    if(calibration_structure->iscalibration_flag==false)
    {
        calculate_standard_deviation_and_variance(calibration_structure);
    }
    /*测量平均值数据有效时，计算缩放因子*/
    if(calibration_structure->mean_effective_flag == true)
    {
        calculate_Scale_factor(&MPU6000_Data->AccRaw, calibration_structure);
    }
}

/**
  * @brief  获取MPU6000测量数据
  * @note   
  * @param  MPU6000_Data：MPU6000数据结构体指针
  * @retval 无
  */
 void MPU6000_GetData(MPU6000Data_t *MPU6000_Data)
 {
     uint8_t i;
     uint8_t j = 0;
     /*注意要连续读取mpu6000数据*/
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
  * @brief  处理MPU6000原始数据
  * @note   
  * @param  MPU6000_Data：MPU6000数据结构体指针
  * @retval 无
  */
void MPU6000_ProcessData(MPU6000Data_t *MPU6000_Data)
{
    uint8_t index;
    /*处理陀螺仪偏置误差*/
    process_gyro_bias(MPU6000_Data,&calibration_structure);

    /*根据设定量程转换数据*/
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
    /*角速度数据*/
    printf("%f,", MPU6000_Data->Gyro.x);
    printf("%f,", MPU6000_Data->Gyro.y);
    printf("%f,", MPU6000_Data->Gyro.z);
    /*滤波*/
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
    /*角速度数据*/
    printf("%f,", MPU6000_Data->Gyro.x);
    printf("%f,", MPU6000_Data->Gyro.y);
    printf("%f\n", MPU6000_Data->Gyro.z);
}
