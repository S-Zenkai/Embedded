#include "bsp_pwm.h"
#include "bsp_usart.h"
#include <stdlib.h>

/*电压幅值等级数*/
#define AMPLITUDE_CLASS 256

/* LED亮度等级 PWM表,指数曲线 ，此表使用工程目录下的python脚本index_wave.py生成*/
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
    /*配置优先级分组，只需在整个工程中配置一次*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; /*抢占优先级*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        /*子优先级*/
    NVIC_Init(&NVIC_InitStructure);
}


static void TimBaseInit(void)
{
   TIM_TimeBaseInitTypeDef TimBaseInitStructure;
   PWM_Timx_APBxClock_FUN(PWM_Timx_CLK, ENABLE);
   /*选择内部时钟*/
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
 * PWM各参数计算公式：
 *      1.PWM频率：F=CK_PSC/(ARR+1)(PSC+1);其中，CK_PSC是分配后时钟频率，ARR是重装载寄存器中的值，PSC是分频系数
 *      2.占空比：Duty=CRR/(ARR+1);CRR是捕获比较寄存器中的值
 *      3.分辨率：Reso=1/(ARR+1)
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
    TimOCxInitStructuer.TIM_OCPolarity = TIM_OCPolarity_Low; /*这个有什么用？试试另一个*/
    // TimOCxInitStructuer.TIM_OutputNState;
    TimOCxInitStructuer.TIM_OutputState = TIM_OutputState_Enable;
    TimOCxInitStructuer.TIM_Pulse=0;/**/
    TIM_OC3Init(PWM_Timx, &TimOCxInitStructuer);

    TIM_OC4Init(PWM_Timx, &TimOCxInitStructuer);

    TIM_OC2Init(PWM_Timx, &TimOCxInitStructuer);
}


void BreathingInit(void)
{
    NVIC_Configuration();/*NVIC中断优先级配置*/
    TimBaseInit(); /*时基单元初始化*/
    GPIOTimInit();/*GPIO端口初始化*/
    TimOCxInit();/*输出比较初始化*/
    TIM_ITConfig(PWM_Timx, TIM_IT_Update, ENABLE);
    TIM_Cmd(PWM_Timx, ENABLE);
}

/**
 *  亮灭周期计算：
 *              1.中断时间：T1=(ARR*PSC)/72M;
 *              2.PWM表每个元素使用时间（包括将比较捕获寄存器置零的时间）：
 *               T2=AMPLITUDE_CLASS*2*T1；
 *                其中，*2是因为if (pwm_counter>1)，当pwm_period>AMPLITUDE_CLASS时，pwm_period会置零重新加至AMPLITUDE_CLASS
 *              3.亮灭周期T3=indexWaveSZ*T2;
 *
 * 中断函数编程思路：
 *                 根据RGB888颜色格式，通过控制三个颜色通道在256个周期的亮灭周期占比即可控制
 */


__IO uint32_t RGB888;/*24位颜色格式，max=0xFFFFFF*/
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


