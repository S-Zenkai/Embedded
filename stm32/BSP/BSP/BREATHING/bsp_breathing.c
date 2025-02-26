#include "bsp_pwm.h"
#include "bsp_usart.h"
#include <stdlib.h>

/*��ѹ��ֵ�ȼ���*/
#define AMPLITUDE_CLASS 256

/* LED���ȵȼ� PWM��,ָ������ ���˱�ʹ�ù���Ŀ¼�µ�python�ű�index_wave.py����*/
const uint16_t indexWave[] = {
    1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
    2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5,
    5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10,
    11, 12, 12, 13, 14, 15, 17, 18,
    19, 20, 22, 23, 25, 27, 29, 31,
    33, 36, 38, 41, 44, 47, 51, 54,
    58, 63, 67, 72, 77, 83, 89, 95,
    102, 110, 117, 126, 135, 145, 156,
    167, 179, 192, 206, 221, 237, 254,
    272, 292, 313, 336, 361, 387, 415,
    445, 477, 512, 512, 477, 445, 415,
    387, 361, 336, 313, 292, 272, 254,
    237, 221, 206, 192, 179, 167, 156,
    145, 135, 126, 117, 110, 102, 95,
    89, 83, 77, 72, 67, 63, 58, 54, 51,
    47, 44, 41, 38, 36, 33, 31, 29, 27,
    25, 23, 22, 20, 19, 18, 17, 15, 14,
    13, 12, 12, 11, 10, 9, 9, 8, 8, 7, 7,
    6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3,
    3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1};

uint16_t indexWaveSZ = sizeof(indexWave) / sizeof(indexWave[0]);

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*�������ȼ����飬ֻ������������������һ��*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; /*��ռ���ȼ�*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        /*�����ȼ�*/
    NVIC_Init(&NVIC_InitStructure);
}


static void TimBaseInit(void)
{
   TIM_TimeBaseInitTypeDef TimBaseInitStructure;
   PWM_Timx_APBxClock_FUN(PWM_Timx_CLK, ENABLE);
   /*ѡ���ڲ�ʱ��*/
   TIM_InternalClockConfig(PWM_Timx);
   TimBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TimBaseInitStructure.TIM_Period=512-1;
   TimBaseInitStructure.TIM_Prescaler=5-1;
   TimBaseInitStructure.TIM_RepetitionCounter = 0;
   TIM_TimeBaseInit(PWM_Timx, &TimBaseInitStructure);
   TIM_ClearFlag(PWM_Timx, TIM_FLAG_Update);
}



static void GPIOTimInit(void)
{
    GPIO_InitTypeDef GPIOInitStructuer;
    PWM_GPIO_APBxClock_FUN(PWM_GPIO_CLK, ENABLE);
    

    GPIOInitStructuer.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIOInitStructuer.GPIO_Pin = PWM_GPIOG_Pin;
    GPIOInitStructuer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_GPIO_Port, &GPIOInitStructuer);

    GPIOInitStructuer.GPIO_Pin = PWM_GPIOR_Pin;
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    GPIO_Init(PWM_GPIO_Port, &GPIOInitStructuer);

    GPIOInitStructuer.GPIO_Pin = PWM_GPIOB_Pin;
    GPIO_Init(PWM_GPIO_Port, &GPIOInitStructuer);
}


/**
 * PWM���������㹫ʽ��
 *      1.PWMƵ�ʣ�F=CK_PSC/(ARR+1)(PSC+1);���У�CK_PSC�Ƿ����ʱ��Ƶ�ʣ�ARR����װ�ؼĴ����е�ֵ��PSC�Ƿ�Ƶϵ��
 *      2.ռ�ձȣ�Duty=CRR/(ARR+1);CRR�ǲ���ȽϼĴ����е�ֵ
 *      3.�ֱ��ʣ�Reso=1/(ARR+1)
 * 
 */
static void TimOCxInit(void)
{
    TIM_OCInitTypeDef TimOCxInitStructuer;

    TIM_OCStructInit(&TimOCxInitStructuer);
    // TimOCxInitStructuer.TIM_OCIdleState;
    TimOCxInitStructuer.TIM_OCMode = TIM_OCMode_PWM1;
    // TimOCxInitStructuer.TIM_OCNIdleState;
    // TimOCxInitStructuer.TIM_OCNPolarity;
    TimOCxInitStructuer.TIM_OCPolarity = TIM_OCPolarity_Low; /*�����ʲô�ã�������һ��*/
    // TimOCxInitStructuer.TIM_OutputNState;
    TimOCxInitStructuer.TIM_OutputState = TIM_OutputState_Enable;
    TimOCxInitStructuer.TIM_Pulse=0;/**/
    TIM_OC3Init(PWM_Timx, &TimOCxInitStructuer);

    TIM_OC4Init(PWM_Timx, &TimOCxInitStructuer);

    TIM_OC2Init(PWM_Timx, &TimOCxInitStructuer);
}


void BreathingInit(void)
{
    NVIC_Configuration();/*NVIC�ж����ȼ�����*/
    TimBaseInit(); /*ʱ����Ԫ��ʼ��*/
    GPIOTimInit();/*GPIO�˿ڳ�ʼ��*/
    TimOCxInit();/*����Ƚϳ�ʼ��*/
    TIM_ITConfig(PWM_Timx, TIM_IT_Update, ENABLE);
    TIM_Cmd(PWM_Timx, ENABLE);
}

/**
 *  �������ڼ��㣺
 *              1.�ж�ʱ�䣺T1=(ARR*PSC)/72M;
 *              2.PWM��ÿ��Ԫ��ʹ��ʱ�䣨�������Ƚϲ���Ĵ��������ʱ�䣩��
 *               T2=AMPLITUDE_CLASS*2*T1��
 *                ���У�*2����Ϊif (pwm_counter>1)����pwm_period>AMPLITUDE_CLASSʱ��pwm_period���������¼���AMPLITUDE_CLASS
 *              3.��������T3=indexWaveSZ*T2;
 *
 * �жϺ������˼·��
 *                 ����RGB888��ɫ��ʽ��ͨ������������ɫͨ����256�����ڵ���������ռ�ȼ��ɿ���
 */


__IO uint32_t RGB888;/*24λ��ɫ��ʽ��max=0xFFFFFF*/
__IO uint16_t pwm_counter = 0;
uint16_t pwm_period;
__IO uint16_t pwm_index = 0;
uint32_t i;
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(PWM_Timx, TIM_IT_Update)==SET)
    {
        if (pwm_period < AMPLITUDE_CLASS)
        {
            if (pwm_period <= ((RGB888 & 0XFF0000) >> 16))/*R*/
                TIM_SetCompare2(PWM_Timx, indexWave[pwm_index]);
            else
                TIM_SetCompare2(PWM_Timx, 0);
            if (pwm_period <= ((RGB888 & 0X00FF00) >> 8))
                TIM_SetCompare3(PWM_Timx, indexWave[pwm_index]);
            else
                TIM_SetCompare3(PWM_Timx, 0);
            if (pwm_period <= (RGB888 & 0X0000FF))
                TIM_SetCompare4(PWM_Timx, indexWave[pwm_index]);
            else
                TIM_SetCompare4(PWM_Timx, 0);
            pwm_period++;
        }
        else
        {
            pwm_counter++;
            pwm_period = 0;
            if (pwm_counter>1)
            {
                pwm_counter = 0;
                pwm_index++;
                if (pwm_index >= indexWaveSZ)
                {
                    pwm_index = 0;
                    i = rand() % (0xFFFFFF + 1);
                    printf("%x\n", i);
                    RGB888 = i;
                }
            }
        }
        TIM_ClearFlag(PWM_Timx, TIM_FLAG_Update);
    }
}


