/**
 * *****************************************************************************
 * @file        bsp_adc.c
 * @brief       ����ģʽ��ͨ��adc��������ʹ�ù����飬DMAת������
 * @author      
 * @date        2024-12-05
 * @version     
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:    stm32f103vet6
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_adc.h"
/*----------------------------------function----------------------------------*/
/**
 * @brief       GPIO����
 * 
 */
static void ADC_GPIOConfigure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADCx_GPIO_ClockFun(ADCx_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; /*ģ������ģʽ�ǶϿ�ָ��GPIO�����ⲿ�������*/
    GPIO_InitStructure.GPIO_Pin = ADCx_IN10_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADCx_IN10_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ADCx_IN11_Pin;
    GPIO_Init(ADCx_IN11_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ADCx_IN12_Pin;
    GPIO_Init(ADCx_IN12_Port, &GPIO_InitStructure);
}

/**
 * @brief       ADC����,����ת����
 * 
 */
static void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADCx_ClockFun(ADCx_CLK, ENABLE);
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;/*����ת��*/
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;/*���ݶ��뷽ʽ*/
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;/*������ʽ*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;/*�Ƿ�˫ADC*/
    ADC_InitStructure.ADC_NbrOfChannel = NbrOfChannel; /*�������õ���Ҫת����ͨ��������ͨ����������������������*/
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;       /*�����Ƿ�ʹ��ɨ�裬��ͨ��ADCʹ��DISABLE����ͨ��ʹ��ENABLE*/
    ADC_Init(ADCx, &ADC_InitStructure);
}

/**
 * @brief       ADC��ʼ��
 *
 */
void ADCInit(void)
{
    ADC_GPIOConfigure();
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);/*ADCʱ��Ƶ�����14M����������Ϊ72M/6=12M*/
    ADC_Configure();
    /*ADC��ת��ʱ��=����ʱ��+12.5��ADC����*/
    /*ADC��ת��ʱ��=(1.5+12.5)*ADC����*/
    ADC_RegularChannelConfig(ADCx, ADC_Channel_10, 1, ADC_SampleTime_1Cycles5); /*ͨ��10��PC0��������1��1.5ADC����*/
    ADC_RegularChannelConfig(ADCx, ADC_Channel_11, 2, ADC_SampleTime_1Cycles5); /*ͨ��11��PC1��������1��1.5ADC����*/
    ADC_RegularChannelConfig(ADCx, ADC_Channel_12, 3, ADC_SampleTime_1Cycles5); /*ͨ��12��PC2��������1��1.5ADC����*/
    ADC_DMACmd(ADCx, ENABLE);
    ADC_Cmd(ADCx, ENABLE);
    /*У׼*/
    ADC_ResetCalibration(ADCx);
    while (ADC_GetResetCalibrationStatus(ADCx) == SET)
        ;
    ADC_StartCalibration(ADCx);
    while (ADC_GetCalibrationStatus(ADCx) == SET)
        ;
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);
}




/*------------------------------------test------------------------------------*/
