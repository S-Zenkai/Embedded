/**
 * *****************************************************************************
 * @file        bsp_adc.c
 * @brief       
 * @author      
 * @date        2024-11-28
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
    ADC_GPIO_ClockFun(ADC_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; /*ģ������ģʽ�ǶϿ�ָ��GPIO�����ⲿ�������*/
    GPIO_InitStructure.GPIO_Pin = ADC_GPIO_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADC_GPIO_Port, &GPIO_InitStructure);
}

/**
 * @brief       ADC����,����ת����
 * 
 */
static void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_ClockFun(ADC_CLK, ENABLE);
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;/*����ת��*/
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;/*���ݶ��뷽ʽ*/
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;/*������ʽ*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;/*�Ƿ�˫ADC*/
    ADC_InitStructure.ADC_NbrOfChannel = 1;/*�������õ���Ҫת����ͨ��������ͨ����������������������*/
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; /*�����Ƿ�ʹ��ɨ�裬��ͨ��ADCʹ��DISABLE����ͨ��ʹ��ENABLE*/
    ADC_Init(ADC, &ADC_InitStructure);
}

void ADCInit(void)
{
    ADC_GPIOConfigure();
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);/*ADCʱ��Ƶ�����14M����������Ϊ72M/6=12M*/
    ADC_Configure();
    /*ADC��ת��ʱ��=����ʱ��+12.5��ADC����*/
    /*ADC��ת��ʱ��=(1.5+12.5)*ADC����*/
    ADC_RegularChannelConfig(ADC, ADC_Channel_11, 1, ADC_SampleTime_1Cycles5); /*ͨ��11������1��1.5ADC����*/
    ADC_Cmd(ADC, ENABLE);
    /*У׼*/
    ADC_ResetCalibration(ADC);
    while (ADC_GetResetCalibrationStatus(ADC) == SET)
        ;
    ADC_StartCalibration(ADC);
    while (ADC_GetCalibrationStatus(ADC) == SET)
        ;
}

uint16_t GetConvValue(void)
{
    ADC_SoftwareStartConvCmd(ADC, ENABLE);
    while (ADC_GetFlagStatus(ADC, ADC_FLAG_EOC)==RESET)
        ;
    return ADC_GetConversionValue(ADC);
}


/*------------------------------------test------------------------------------*/
