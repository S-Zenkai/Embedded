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

/*二阶低通滤波器参数*/
LPF2ordParam_t gyro_LPF2Param[3];
LPF2ordParam_t acc_LPF2Param[3];
/*MPU6000数据校准相关结构体变量*/
static MPU6000Calibration_t calibration;

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
void MPU6000_ReadData(uint8_t* buf)
{
    uint8_t i;
    SPI_CS_L;
    SPI_TansmissionReceiveByte(MPU6000_ACCEL_XOUT_H|0x80);/*mpu6000的SPI时序：0是写操作，1是读操作*/
    for (i = 0; i < 14;i++)
    {
        buf[i] = SPI_TansmissionReceiveByte(0x00);
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
    for (uint8_t i = 0; i < 3;i++)
    {
        LPF2ord_Init(&gyro_LPF2Param[i], 1000, 80);
        LPF2ord_Init(&acc_LPF2Param[i], 1000, 30);
    }

    calibration.acc_scale = 1.0f;
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
     uint8_t raw_buf[14];
     /*注意要连续读取mpu6000数据*/
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
     /*角速度数据*/
     printf("%hd,",MPU6000_Data->GyroRaw.x);
     printf("%hd,",MPU6000_Data->GyroRaw.y);
     printf("%hd\n",MPU6000_Data->GyroRaw.z);
 }

static bool calculate_gyro(MPU6000Data_t *data)
{
    uint8_t i;
    float mean;/*平均值*/
    float var;/*方差*/
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
    calibration.sample_count = 0;/*之后用于加速度校准*/
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
  * @brief  处理MPU6000原始数据
  * @note   
  * @param  MPU6000_Data：MPU6000数据结构体指针
  * @retval 无
  */
void MPU6000_ProcessData(MPU6000Data_t *MPU6000_Data)
{
    uint8_t index;
    /*校准陀螺仪和加速度计*/
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
    /*根据设定量程转换数据*/
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
//    /*角速度数据*/
//    printf("%f,", MPU6000_Data->Gyro.x);
//    printf("%f,", MPU6000_Data->Gyro.y);
//    printf("%f,", MPU6000_Data->Gyro.z);
    /*滤波*/
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
//    /*角速度数据*/
//    printf("%f,", MPU6000_Data->Gyro.x);
//    printf("%f,", MPU6000_Data->Gyro.y);
//    printf("%f\n", MPU6000_Data->Gyro.z);
}
