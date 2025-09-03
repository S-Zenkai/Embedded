/**
  ******************************************************************************
  * @file    bsp_ms5611.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/18
  * @brief   MS5611驱动文件
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_ms5611.h"
#include "bsp_spi.h"
#include "bsp_systick.h"
#include <math.h>
#include "filter.h"
#include "bsp_usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*ms5611prom缓冲区，存放读取到的出场数据和设置（地址0）、校准系数（地址1-6）、串行码和CRC（地址7）*/
uint16_t prom_buff[8];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  ms5611 cs引脚初始化
  * @note   
  * @param  无
  * @retval 无
  */
static void ms5611_cs_configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /*MS5611：CS(PD7)*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    MS5611_SPI_CS_H;
}

/**
  * @brief  
  * @note   
  * @param  无
  * @retval 无
  */
static void ms5611_reset(void)
{
    MS5611_SPI_CS_L;
    // delay_ms(1);
    SPI_TR_Byte(CMD_RESET);
    delay_ms(3);/*官方代码推荐*/
    MS5611_SPI_CS_H;
    // delay_ms(1);
}

/**
  * @brief  读取ms5611的prom，得到出场数据和设置（地址0）、校准系数（地址1-6）串行码和CRC（地址7）
  * @note   
  * @param  prom_buff：缓冲区
  * @retval 无
  */
static void ms5611_Readprom(ms5611Prom_t* prom_buff)
{
    uint8_t i = 0;
    for (i = 0; i < 8;i++)
    {
        MS5611_SPI_CS_L;
        // delay_ms(1);
        SPI_TR_Byte(CMD_PROM_RD + 2 * i);
        prom_buff->prom[i]=SPI_TR_Byte(0x00);/*MSB*/
        prom_buff->prom[i] <<= 8;
        prom_buff->prom[i] |= SPI_TR_Byte(0x00); /*LSB*/
        MS5611_SPI_CS_H;
        // delay_ms(1);
    }
}

/**
  * @brief  ms5611初始化
  * @note   
  * @param  param：ms5611参数结构体
  * @retval 无
  */
void ms5611_init(ms5611param_t* param)
{
    ms5611_cs_configure();
    ms5611_reset();
    ms5611_Readprom(&param->prom_data);
}


/**
  * @brief  
  * @note   
  * @param  无
  * @retval 无
  */
static void osr_delay(uint8_t osr)
{
    switch (osr)
    {
    case CMD_ADC_256:
        delay_us(900);
        break;
    case CMD_ADC_512:
        delay_ms(3);
        break;
    case CMD_ADC_1024:
        delay_ms(4);
        break;
    case CMD_ADC_2048:
        delay_ms(6);
        break;
    case CMD_ADC_4096:
        delay_ms(10);
        break;
    default:
        break;
    }
}

/**
  * @brief  
  * @note   
  * @param  无
  * @retval 无
  */
static uint32_t ms5611_ReadADC(uint8_t adc, uint8_t osr)
{
    uint32_t ret;
    MS5611_SPI_CS_L;
    // delay_ms(1);
    /*使能D1(为补偿压力)或D2(未补偿温度)转换*/
    SPI_TR_Byte(CMD_ADC_CONV + adc + osr);
    /*依据过分辨率选择转换时间，下面的时间选择是官方例程用的，但时间要比最大转换时间要大*/
    osr_delay(osr);
    /*注意时序要求开始新的命令序列时必需重新拉低拉高cs引脚*/
    MS5611_SPI_CS_H;
    // delay_ms(1);
    MS5611_SPI_CS_L;
    // delay_ms(1);
    /*读取ADC数据*/
    SPI_TR_Byte(CMD_ADC_READ);
    ret = SPI_TR_Byte(0x00); /*MSB*/
    ret <<= 8;
    ret |= SPI_TR_Byte(0x00);
    ret <<= 8;
    ret |= SPI_TR_Byte(0x00);
    MS5611_SPI_CS_H;
    // delay_ms(1);
    return ret;
}

// /**
//   * @brief  
//   * @note   
//   * @param  无
//   * @retval 无
//   */
// void ms5611_GetData(ms5611param_t* param_s)
// {
//     /*Read digital pressure and temperature data*/
//     param_s->D1 = ms5611_ReadADC(CMD_ADC_D1, CMD_ADC_512);
//     param_s->D2 = ms5611_ReadADC(CMD_ADC_D2, CMD_ADC_512);
//     /*Calculate temperature*/
//     param_s->dT = param_s->D2 - param_s->prom_data.C5 * powf(2, 8);
//     // param_s->temp = 20.0f + (float)param_s->dT * param_s->prom_data.C6 / powf(2, 23);
//     // param_s->temp = (2000 + (param_s->dT * param_s->prom_data.C6) / powf(2, 23))/100;
//     param_s->temp = (2000 + (param_s->dT * param_s->prom_data.C6) / powf(2, 23));
//     /*Calculate temperature compensated pressure*/
//     // param_s->OFF = param_s->prom_data.C2 * powf(2, 17) + (float)(param_s->prom_data.C4 * param_s->dT) / powf(2, 6);
//     param_s->OFF = param_s->prom_data.C2 * powf(2, 17) + (param_s->prom_data.C4 * param_s->dT) / powf(2, 6);
//     // param_s->SENS = param_s->prom_data.C1 *powf(2, 16) + (float)(param_s->prom_data.C3 * param_s->dT) / powf(2, 7);
//     param_s->SENS = param_s->prom_data.C1 *powf(2, 16) + (param_s->prom_data.C3 * param_s->dT) / powf(2, 7);
//     // param_s->P = (((param_s->D1 * param_s->SENS) / powf(2, 21) - param_s->OFF) / powf(2, 15))/100;
//     param_s->P = (((param_s->D1 * param_s->SENS) / powf(2, 21) - param_s->OFF) / powf(2, 15));
// }

/**
 * @brief
 * @note
 * @param  无
 * @retval 无
 */
void ms5611_GetData(ms5611param_t *param_s)
{
    /*Read digital pressure and temperature data*/
    param_s->D1 = ms5611_ReadADC(CMD_ADC_D1, CMD_ADC_512);
    param_s->D2 = ms5611_ReadADC(CMD_ADC_D2, CMD_ADC_512);

    /*Calculate temperature*/
    param_s->dT = param_s->D2 - ((uint32_t)param_s->prom_data.C5 << 8);
    // temp = (2000 + (param_s->dT * param_s->prom_data.C6) / powf(2, 23)) / 100.0f;
    int32_t temp = (2000 + ((int64_t)param_s->dT * param_s->prom_data.C6) / (1 << 23));/*这里定义为int32原因是/2^23,本质上是移位操作，使用整数精度更高*/

    param_s->OFF = ((int64_t)param_s->prom_data.C2 <<17) + (((int64_t)param_s->prom_data.C4 * param_s->dT) / (1<<6));
    param_s->SENS = ((int64_t)param_s->prom_data.C1 <<16) + (((int64_t)param_s->prom_data.C3 * param_s->dT) / (1<<7));


    /*温度>20度时*/
    param_s->T2 = param_s->OFF2 = param_s->SENS2 = 0;
    /*温度<20度*/
    if (temp < 2000)
    {
        param_s->T2 = ((int64_t)param_s->dT * param_s->dT) / (1LL<<31);
        param_s->OFF2 = 5 * ((temp - 2000LL) * (temp - 2000LL)) / 2;
        param_s->SENS2 = 5 * ((temp - 2000LL) * (temp - 2000LL)) / 4;

        /*温度<15*/
        if (temp < -1500)
        {
            param_s->OFF2 = param_s->OFF2 + 7 * ((temp + 1500LL) * (temp + 1500LL));
            param_s->SENS2 = param_s->SENS2 + (11 * ((temp + 1500LL) * (temp + 1500LL))) / 2;
        }
    }
    temp = (temp - param_s->T2);
    // printf("%f,", temp);
    /*Calculate temperature compensated pressure*/
    
    param_s->OFF = param_s->OFF - param_s->OFF2;
    param_s->SENS = param_s->SENS - param_s->SENS2;

    int32_t P = (int32_t)((((int64_t)param_s->D1 * param_s->SENS) / (1 << 21) - param_s->OFF) / (1 << 15));
    param_s->P = P / 100.0f;
    printf("%f,", param_s->P);
    param_s->temp = temp / 100.0f;
    printf("%f,", param_s->temp);
		#if 0
    param_s->P = moving_average(param_s->P);
		#endif
    printf("%f,", param_s->P);
}

float reference_pressure = 1013.25;
void ms5611_CalculateAltitude(ms5611param_t *param_s)
{
    /*现找到了如下几个计算高度的公式*/
    /*此处的标准大气压1024.1为查得的深圳当地海平面标准气压，一般是使用标准大气压1013.2，这里要测试下是否合适*/
    /*1 barometric公式*/
    // param_s->AH = 44300.0f * (1 - powf(param_s->P / 1024.1f, 1.0f / 5.255f));
    param_s->AH = 44330.0f * (1.0f - powf(reference_pressure / 1013.25f, 1.0f / 5.255f));
    param_s->RH = 44330.0f * (1.0f - powf(param_s->P / 1013.25f, 1.0f / 5.255f)) - param_s->AH;
    printf("%f\n", param_s->RH);
    // param_s->AH = moving_average(param_s->AH);
    // printf("%f\n", param_s->AH);
    /*2 hypsometric公式,考虑了温度的影响*/
    // param_s->AH = (powf(1024.1f / param_s->P, 1.0f / 5.255f) - 1.0f) * (param_s->temp + 273.15f) / 0.0065f;
    // param_s->AH = (powf(1022.65f / param_s->P, 1.0f / 5.255f) - 1.0f) * (param_s->temp + 273.15f) / 0.0065f;
    // printf("%f\n", param_s->AH);
    // param_s->AH = moving_average(param_s->AH);
    // printf("%f\n", param_s->AH);
}
