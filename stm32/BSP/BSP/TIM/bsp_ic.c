/**
 * *****************************************************************************
 * @file        bsp_ic.c
 * @brief       tim���벶��ʵ�飬ͨ�����벶���ܲ���pwm�ź�Ƶ�ʺ�ռ�ձ�
 * @author      
 * @date        2024-11-12
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:    stm32f103vet6�����벶��tim4������pwm����tim3
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_ic.h"
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
    TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/**/
    TimBaseInitStructure.TIM_Period = 65535 - 1;             /*ARR��װ�ؼĴ���ֵ*/
    TimBaseInitStructure.TIM_Prescaler = 72 - 1;/*Ԥ��Ƶֵ*/
    TimBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(Timx, &TimBaseInitStructure);
    /*����TIM_TimeBaseInit�����ز���һ�������¼���������ʱ��͸����ж�ͬʱ����������ʹ��TIM����������һ���ж�*/
    /*���������ʼʱ�����жϣ�ֻ����TIM_TimeBaseInit��TIM_ITConfig֮����������¼���־λ����*/
    TIM_ClearFlag(Timx, TIM_FLAG_Update); /*���д���ɱ�֤value��0��ʼ��*/
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
    GPIOInitStructuer.GPIO_Mode = GPIO_Mode_IPU;
    GPIOInitStructuer.GPIO_Pin = Timx_CH1_GPIO_Pin;
    GPIOInitStructuer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Timx_CH1_GPIO_Port, &GPIOInitStructuer);
}

/**
 * PWM���������㹫ʽ��
 *      1.PWMƵ�ʣ�F=CK_PSC/(ARR+1)(PSC+1);���У�CK_PSC�Ƿ����ʱ��Ƶ�ʣ�ARR����װ�ؼĴ����е�ֵ��PSC�Ƿ�Ƶϵ��
 *      2.ռ�ձȣ�Duty=CRR/(ARR+1);CRR�ǲ���ȽϼĴ����е�ֵ
 *      3.�ֱ��ʣ�Reso=1/(ARR+1)
 *
 */

/**
 * @brief       ���벶���ʼ��
 * 
 */
static void ICInit(void)
{
    TIM_ICInitTypeDef TimICInitStructure;
    TimICInitStructure.TIM_Channel = TIM_Channel_1;/*����ͨ��ѡ��*/
    TimICInitStructure.TIM_ICFilter = 0xF;/*�����˲���*/
    TimICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;/*���ش���ѡ��*/
    TimICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;/*���벶��Ԥ��Ƶ��*/
    TimICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;/*ѡ��TIxFPx*/
    TIM_ICInit(Timx ,&TimICInitStructure);

    /*PWMI����*/
    TIM_PWMIConfig(Timx, &TimICInitStructure);
    TIM_ICInit(Timx, &TimICInitStructure);

    /*��ģʽ����*/
    /*ʵ�ּ������Զ�����*/
    TIM_SelectInputTrigger(Timx, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(Timx, TIM_SlaveMode_Reset);
}

void TimICInit(void)
{
    TimBaseInit();        /*ʱ����Ԫ��ʼ��*/
    TimGPIOInit();        /*GPIO�˿ڳ�ʼ��*/
    ICInit();          /*���벶���ʼ��*/
    TIM_Cmd(Timx, ENABLE);
}

/**
 * @brief       ���������ź�Ƶ�ʣ�ʹ�ò��ܷ�����ʽ��fx=fc/N�����У�fcΪ��׼Ƶ�ʣ�Ҳ�Ǳ��ļ�ʱ���趨�Ķ�ʱ������Ƶ��(72M/PSC)
 * 
 * @return      uint32_t ,Ƶ��
 */
uint16_t GetFreq(void)
{
    return 1000000 / (TIM_GetCapture1(Timx)+1);
}

/**
 * @brief       Get the Duty object
 * 
 * @return      uint8_t ռ�ձ�
 */
uint8_t GetDuty(void)
{
    return ((TIM_GetCapture2(Timx) + 1) * 100) / (TIM_GetCapture1(Timx) + 1);
}

#if 0
/**
 * @brief       ��Ƶ������Ƶ�ʣ���ʽ��fc=T/N�����У�TΪբ��ʱ�䣨������ͨ����ʱ�жϹ���ʵ�֣���
 *              NΪTʱ���ڴ����ź�����������
 * @note        ʹ�ñ���������Ƶ�ʻ�������TIM4��ʱ�жϹ����Լ�������صı���
 * 
 */
void TIM4_IRQHandler(void)
{
    if (TIM_GetFlagStatus(Timx, TIM_FLAG_Update))
    {
        freq = counter / 6;
        counter = 0;
        OLED_Printf(0, 0, OLED_8X16, "freq=%d", freq);
        OLED_Update();
        TIM_ClearFlag(Timx, TIM_FLAG_Update);
    }
    if (TIM_GetFlagStatus(Timx, TIM_FLAG_CC1))
    {
        counter++;
        TIM_ClearFlag(Timx, TIM_FLAG_CC1);
    }
}
#endif


    /*------------------------------------test------------------------------------*/
