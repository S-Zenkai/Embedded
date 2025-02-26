/**
 * *****************************************************************************
 * @file        bsp_encoder.c
 * @brief       TIM������ģʽ�������ת��
 * @author      
 * @date        2024-11-22
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:    stm32f103vet6��tim������ģʽ��tim4CH1��CH2
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_encoder.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief       TIMʱ����ʼ����
 * 
 */
static void TimBaseInit(void)
{
    TIM_TimeBaseInitTypeDef TimBaseInitStructure;
    Timx_APBxClock_FUN(Timx_CLK, ENABLE);
    /*ѡ���ڲ�ʱ��*/
    TIM_InternalClockConfig(Timx);
    TimBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; /*��ʱ��ʱ��CK_INTƵ���������˲�������Ƶ��֮��ķ�Ƶ��*/
    TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*����ɲ����ã����ɱ����������йܣ�����ûʲô��*/
    TimBaseInitStructure.TIM_Period = 65535 - 1;             /*ARR��װ�ؼĴ���ֵ*/
    TimBaseInitStructure.TIM_Prescaler = 1 - 1;/*Ԥ��Ƶֵ*/
    TimBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(Timx, &TimBaseInitStructure);
}

/**
 * @brief       ��TIM_CH��ص�GPIO��ʼ������
 * 
 */
static void TimGPIOInit(void)
{
    GPIO_InitTypeDef GPIOInitStructuer;
    Timx_GPIO_APBxClock_FUN(Timx_GPIO_CLK, ENABLE);
    /*CH1������Ϊ���벶��ͨ��*/
    /*����gpio���ã����ⲿ����Ĭ�ϵ�ƽΪ�ߵ�ƽʱ������Ϊ��������*/
    /*���ⲿ����Ĭ�ϵ�ƽΪ�͵�ƽʱ������Ϊ��������*/
    /*��ȷ���ⲿ���룬������Ϊ�������룬ȱ������������������*/
    GPIOInitStructuer.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIOInitStructuer.GPIO_Pin = Timx_CH1_GPIO_Pin;
    GPIOInitStructuer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Timx_CH1_GPIO_Port, &GPIOInitStructuer);
    /*CH2����*/
    GPIOInitStructuer.GPIO_Pin = Timx_CH2_GPIO_Pin;
    GPIO_Init(Timx_CH2_GPIO_Port, &GPIOInitStructuer);
}

/**
 * @brief       ���벶���ʼ��
 * 
 */
static void ICInit(void)
{
    TIM_ICInitTypeDef TimICInitStructure;
    TimICInitStructure.TIM_Channel = TIM_Channel_1;/*����ͨ��ѡ��*/
    TimICInitStructure.TIM_ICFilter = 0xF;/*�����˲���*/
    /*���沢������һ�����벶�������ش��������Ǽ��Բ���ת���ɲ����ã��ڱ��������ú������ظ�����*/
    TimICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
    //TimICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;/*���벶��Ԥ��Ƶ����������ģʽδʹ��*/
    TimICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;/*ѡ��TIxFPx*/
    TIM_ICInit(Timx ,&TimICInitStructure);

    TimICInitStructure.TIM_Channel = TIM_Channel_2; /*����ͨ��ѡ��*/
    TimICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(Timx, &TimICInitStructure);
}

void EncoderInit(void)
{
    TimBaseInit();        /*ʱ����Ԫ��ʼ��*/
    TimGPIOInit();        /*GPIO�˿ڳ�ʼ��*/
    ICInit();             /*���벶���ʼ��*/
    /*���ñ�����ģʽ������TI1��TI2���Բ���ת*/
    TIM_EncoderInterfaceConfig(Timx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(Timx, ENABLE);
}

/**
 * @brief       ���������ź�Ƶ�ʣ�ʹ�ò��ܷ�����ʽ��fx=fc/N�����У�fcΪ��׼Ƶ�ʣ�Ҳ�Ǳ��ļ�ʱ���趨�Ķ�ʱ������Ƶ��(72M/PSC)
 * 
 * @return      uint32_t ,Ƶ��
 */
int16_t GetSpeed(void)
{
    uint16_t speed = 0;
    speed = TIM_GetCounter(Timx) / 1320;/*���һȦ11�����壬һ�������4����*/
    TIM_SetCounter(Timx, 0);
    return speed;
}
    /*------------------------------------test------------------------------------*/
