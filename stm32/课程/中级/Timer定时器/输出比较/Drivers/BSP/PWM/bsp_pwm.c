#include "bsp_pwm.h"

/* LED亮度等级 PWM表,指数曲线 ，此表使用工程目录下的python脚本index_wave.py生成*/
uint16_t indexWave[] = {
    1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4,
    4, 5, 5, 6, 7, 8, 9, 10, 11, 13,
    15, 17, 19, 22, 25, 28, 32, 36,
    41, 47, 53, 61, 69, 79, 89, 102,
    116, 131, 149, 170, 193, 219, 250,
    284, 323, 367, 417, 474, 539, 613,
    697, 792, 901, 1024, 1024, 901, 792,
    697, 613, 539, 474, 417, 367, 323,
    284, 250, 219, 193, 170, 149, 131,
    116, 102, 89, 79, 69, 61, 53, 47, 41,
    36, 32, 28, 25, 22, 19, 17, 15, 13,
    11, 10, 9, 8, 7, 6, 5, 5, 4, 4, 3, 3,
    2, 2, 2, 2, 1, 1, 1, 1

};

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
   TimBaseInitStructure.TIM_Period=1024-1;
   TimBaseInitStructure.TIM_Prescaler=200-1;
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

    // GPIOInitStructuer.GPIO_Pin = PWM_GPIOR_Pin;
    // GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    // GPIO_Init(PWM_GPIO_Port, &GPIOInitStructuer);

    // GPIOInitStructuer.GPIO_Pin = PWM_GPIOB_Pin;
    // GPIO_Init(PWM_GPIO_Port, &GPIOInitStructuer);
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

    // TIM_OC4Init(PWM_Timx, &TimOCxInitStructuer);

    // TIM_OC2Init(PWM_Timx, &TimOCxInitStructuer);

    TIM_Cmd(PWM_Timx, ENABLE);
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

__IO uint16_t pwm_counter = 0;
uint16_t pwm_period = 10;
__IO uint16_t pwm_index = 0;
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(PWM_Timx, TIM_IT_Update)==SET)
    {
        if (pwm_counter < pwm_period)
        {
            TIM_SetCompare3(PWM_Timx, indexWave[pwm_index]);
            pwm_counter++;
            if (pwm_counter == pwm_period)
            {
                pwm_index++;
                pwm_counter = 0;
            }
            if (pwm_index==indexWaveSZ)
            {
                pwm_index = 0;
            }
        }
    }
    TIM_ClearFlag(PWM_Timx, TIM_FLAG_Update);
}
