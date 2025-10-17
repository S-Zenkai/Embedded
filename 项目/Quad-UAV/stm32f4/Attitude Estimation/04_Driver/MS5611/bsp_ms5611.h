/**
  ******************************************************************************
  * @file    bsp_ms5611.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/18
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_MS5611_H
#define __BSP_MS5611_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/* Exported define ------------------------------------------------------------*/
 /*CS引脚操作*/
 #define     MS5611_SPI_CS_L            GPIO_ResetBits(GPIOD, GPIO_Pin_7)
 #define     MS5611_SPI_CS_H            GPIO_SetBits(GPIOD, GPIO_Pin_7)

/*ms5611命令*/
#define     CMD_RESET           0x1E // ADC reset command
#define     CMD_ADC_READ        0x00 // ADC read command
#define     CMD_ADC_CONV        0x40 // ADC conversion command
#define     CMD_ADC_D1          0x00 // ADC D1 conversion
#define     CMD_ADC_D2          0x10 // ADC D2 conversion
#define     CMD_ADC_256         0x00 // ADC OSR=256
#define     CMD_ADC_512         0x02 // ADC OSR=512
#define     CMD_ADC_1024        0x04 // ADC OSR=1024
#define     CMD_ADC_2048        0x06 // ADC OSR=2056
#define     CMD_ADC_4096        0x08 // ADC OSR=4096
#define     CMD_PROM_RD         0xA0 // Prom read command
/* Exported types ------------------------------------------------------------*/

typedef struct
{
    

} ms5611data_t;

typedef union
{
    struct
    {
        uint16_t AD0;                     /*存放读取到的出场数据和设置(地址0)*/
        uint16_t C1, C2, C3, C4, C5, C6; /*校准系数*/
        uint16_t AD7;                     /*串行码和CRC(地址7)*/
    };
    uint16_t prom[8];
} ms5611Prom_t;


typedef struct
{
    ms5611Prom_t prom_data;
    uint32_t D1;/*未补偿数字压力值*/
    uint32_t D2;/*未补偿数字温度值*/
    int32_t dT;
    float temp;/*补偿温度*/
    int64_t OFF;
    int64_t SENS;
    float P;/*补偿压力*/
    int32_t T2;
    int64_t OFF2;
    int64_t SENS2;
    float AH;/*绝对高度*/
    float RH;/*相对高度*/
} ms5611param_t;

// typedef struct
// {
//     ms5611Prom_t prom_data;
//     uint32_t D1;/*未补偿数字压力值*/
//     uint32_t D2;/*未补偿数字温度值*/
//     int32_t dT;
//     int32_t temp;/*补偿温度*/
//     int64_t OFF;
//     int64_t SENS;
//     int64_t P;/*补偿压力*/
// } ms5611param_t;

/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ms5611_init(ms5611param_t *param);
void ms5611_GetData(ms5611param_t *param_s);
void ms5611_CalculateAltitude(ms5611param_t *param_s);

#endif /* __BSP_MS5611_H */

