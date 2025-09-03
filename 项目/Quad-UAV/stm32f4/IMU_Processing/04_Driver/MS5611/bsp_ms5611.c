/**
  ******************************************************************************
  * @file    bsp_ms5611.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/18
  * @brief   MS5611�����ļ�
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

/*ms5611prom����������Ŷ�ȡ���ĳ������ݺ����ã���ַ0����У׼ϵ������ַ1-6�����������CRC����ַ7��*/
uint16_t prom_buff[8];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  ms5611 cs���ų�ʼ��
  * @note   
  * @param  ��
  * @retval ��
  */
static void ms5611_cs_configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /*MS5611��CS(PD7)*/
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
  * @param  ��
  * @retval ��
  */
static void ms5611_reset(void)
{
    MS5611_SPI_CS_L;
    // delay_ms(1);
    SPI_TR_Byte(CMD_RESET);
    delay_ms(3);/*�ٷ������Ƽ�*/
    MS5611_SPI_CS_H;
    // delay_ms(1);
}

/**
  * @brief  ��ȡms5611��prom���õ��������ݺ����ã���ַ0����У׼ϵ������ַ1-6���������CRC����ַ7��
  * @note   
  * @param  prom_buff��������
  * @retval ��
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
  * @brief  ms5611��ʼ��
  * @note   
  * @param  param��ms5611�����ṹ��
  * @retval ��
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
  * @param  ��
  * @retval ��
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
  * @param  ��
  * @retval ��
  */
static uint32_t ms5611_ReadADC(uint8_t adc, uint8_t osr)
{
    uint32_t ret;
    MS5611_SPI_CS_L;
    // delay_ms(1);
    /*ʹ��D1(Ϊ����ѹ��)��D2(δ�����¶�)ת��*/
    SPI_TR_Byte(CMD_ADC_CONV + adc + osr);
    /*���ݹ��ֱ���ѡ��ת��ʱ�䣬�����ʱ��ѡ���ǹٷ������õģ���ʱ��Ҫ�����ת��ʱ��Ҫ��*/
    osr_delay(osr);
    /*ע��ʱ��Ҫ��ʼ�µ���������ʱ����������������cs����*/
    MS5611_SPI_CS_H;
    // delay_ms(1);
    MS5611_SPI_CS_L;
    // delay_ms(1);
    /*��ȡADC����*/
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
//   * @param  ��
//   * @retval ��
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
 * @param  ��
 * @retval ��
 */
void ms5611_GetData(ms5611param_t *param_s)
{
    /*Read digital pressure and temperature data*/
    param_s->D1 = ms5611_ReadADC(CMD_ADC_D1, CMD_ADC_512);
    param_s->D2 = ms5611_ReadADC(CMD_ADC_D2, CMD_ADC_512);

    /*Calculate temperature*/
    param_s->dT = param_s->D2 - ((uint32_t)param_s->prom_data.C5 << 8);
    // temp = (2000 + (param_s->dT * param_s->prom_data.C6) / powf(2, 23)) / 100.0f;
    int32_t temp = (2000 + ((int64_t)param_s->dT * param_s->prom_data.C6) / (1 << 23));/*���ﶨ��Ϊint32ԭ����/2^23,����������λ������ʹ���������ȸ���*/

    param_s->OFF = ((int64_t)param_s->prom_data.C2 <<17) + (((int64_t)param_s->prom_data.C4 * param_s->dT) / (1<<6));
    param_s->SENS = ((int64_t)param_s->prom_data.C1 <<16) + (((int64_t)param_s->prom_data.C3 * param_s->dT) / (1<<7));


    /*�¶�>20��ʱ*/
    param_s->T2 = param_s->OFF2 = param_s->SENS2 = 0;
    /*�¶�<20��*/
    if (temp < 2000)
    {
        param_s->T2 = ((int64_t)param_s->dT * param_s->dT) / (1LL<<31);
        param_s->OFF2 = 5 * ((temp - 2000LL) * (temp - 2000LL)) / 2;
        param_s->SENS2 = 5 * ((temp - 2000LL) * (temp - 2000LL)) / 4;

        /*�¶�<15*/
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
    /*���ҵ������¼�������߶ȵĹ�ʽ*/
    /*�˴��ı�׼����ѹ1024.1Ϊ��õ����ڵ��غ�ƽ���׼��ѹ��һ����ʹ�ñ�׼����ѹ1013.2������Ҫ�������Ƿ����*/
    /*1 barometric��ʽ*/
    // param_s->AH = 44300.0f * (1 - powf(param_s->P / 1024.1f, 1.0f / 5.255f));
    param_s->AH = 44330.0f * (1.0f - powf(reference_pressure / 1013.25f, 1.0f / 5.255f));
    param_s->RH = 44330.0f * (1.0f - powf(param_s->P / 1013.25f, 1.0f / 5.255f)) - param_s->AH;
    printf("%f\n", param_s->RH);
    // param_s->AH = moving_average(param_s->AH);
    // printf("%f\n", param_s->AH);
    /*2 hypsometric��ʽ,�������¶ȵ�Ӱ��*/
    // param_s->AH = (powf(1024.1f / param_s->P, 1.0f / 5.255f) - 1.0f) * (param_s->temp + 273.15f) / 0.0065f;
    // param_s->AH = (powf(1022.65f / param_s->P, 1.0f / 5.255f) - 1.0f) * (param_s->temp + 273.15f) / 0.0065f;
    // printf("%f\n", param_s->AH);
    // param_s->AH = moving_average(param_s->AH);
    // printf("%f\n", param_s->AH);
}
