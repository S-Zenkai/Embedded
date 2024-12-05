/**
 * *****************************************************************************
 * @file        bsp_adc.c
 * @brief       独立模式多通道adc驱动程序，使用规则组，DMA转移数据
 * @author      
 * @date        2024-12-05
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
    ADCx_GPIO_ClockFun(ADCx_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; /*模拟输入模式是断开指定GPIO，由外部输入决定*/
    GPIO_InitStructure.GPIO_Pin = ADCx_IN10_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADCx_IN10_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ADCx_IN11_Pin;
    GPIO_Init(ADCx_IN11_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ADCx_IN12_Pin;
    GPIO_Init(ADCx_IN12_Port, &GPIO_InitStructure);
}

/**
 * @brief       ADC配置,单次转换，
 * 
 */
static void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADCx_ClockFun(ADCx_CLK, ENABLE);
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;/*连续转换*/
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;/*数据对齐方式*/
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;/*触发方式*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;/*是否双ADC*/
    ADC_InitStructure.ADC_NbrOfChannel = NbrOfChannel; /*这里配置的是要转换的通道个数，通道和序列由其他函数配置*/
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;       /*配置是否使用扫描，单通道ADC使用DISABLE，多通道使用ENABLE*/
    ADC_Init(ADCx, &ADC_InitStructure);
}

/**
 * @brief       ADC初始化
 *
 */
void ADCInit(void)
{
    ADC_GPIOConfigure();
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);/*ADC时钟频率最大14M，这里配置为72M/6=12M*/
    ADC_Configure();
    /*ADC中转换时间=采样时间+12.5个ADC周期*/
    /*ADC中转换时间=(1.5+12.5)*ADC周期*/
    ADC_RegularChannelConfig(ADCx, ADC_Channel_10, 1, ADC_SampleTime_1Cycles5); /*通道10（PC0）、序列1，1.5ADC周期*/
    ADC_RegularChannelConfig(ADCx, ADC_Channel_11, 2, ADC_SampleTime_1Cycles5); /*通道11（PC1）、序列1，1.5ADC周期*/
    ADC_RegularChannelConfig(ADCx, ADC_Channel_12, 3, ADC_SampleTime_1Cycles5); /*通道12（PC2）、序列1，1.5ADC周期*/
    ADC_DMACmd(ADCx, ENABLE);
    ADC_Cmd(ADCx, ENABLE);
    /*校准*/
    ADC_ResetCalibration(ADCx);
    while (ADC_GetResetCalibrationStatus(ADCx) == SET)
        ;
    ADC_StartCalibration(ADCx);
    while (ADC_GetCalibrationStatus(ADCx) == SET)
        ;
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);
}




/*------------------------------------test------------------------------------*/
