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
 * 实验平台:    stm32f103vet6
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_adc.h"
/*----------------------------------function----------------------------------*/
/**
 * @brief       GPIO配置
 * 
 */
static void ADC_GPIOConfigure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_GPIO_ClockFun(ADC_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; /*模拟输入模式是断开指定GPIO，由外部输入决定*/
    GPIO_InitStructure.GPIO_Pin = ADC_GPIO_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADC_GPIO_Port, &GPIO_InitStructure);
}

/**
 * @brief       ADC配置,单次转换，
 * 
 */
static void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_ClockFun(ADC_CLK, ENABLE);
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;/*单次转换*/
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;/*数据对齐方式*/
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;/*触发方式*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;/*是否双ADC*/
    ADC_InitStructure.ADC_NbrOfChannel = 1;/*这里配置的是要转换的通道个数，通道和序列由其他函数配置*/
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; /*配置是否使用扫描，单通道ADC使用DISABLE，多通道使用ENABLE*/
    ADC_Init(ADC, &ADC_InitStructure);
}

void ADCInit(void)
{
    ADC_GPIOConfigure();
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);/*ADC时钟频率最大14M，这里配置为72M/6=12M*/
    ADC_Configure();
    /*ADC中转换时间=采样时间+12.5个ADC周期*/
    /*ADC中转换时间=(1.5+12.5)*ADC周期*/
    ADC_RegularChannelConfig(ADC, ADC_Channel_11, 1, ADC_SampleTime_1Cycles5); /*通道11、序列1，1.5ADC周期*/
    ADC_Cmd(ADC, ENABLE);
    /*校准*/
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
