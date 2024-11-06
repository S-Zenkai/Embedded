#include "bsp_pwm.h"


static void TimBaseInit(void)
{
   TIM_TimeBaseInitTypeDef TimBaseInitStructure;
   PWM_Timx_APBxClock_FUN(PWM_Timx_CLK, ENABLE);
   /*选择内部时钟*/
   TIM_InternalClockConfig(PWM_Timx);
   TimBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TimBaseInitStructure.TIM_Period=100-1;
   TimBaseInitStructure.TIM_Prescaler=720-1;
   TimBaseInitStructure.TIM_RepetitionCounter = 0;
   TIM_TimeBaseInit(PWM_Timx, &TimBaseInitStructure);
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
void TimOCxInit(void)
{
    TIM_OCInitTypeDef TimOCxInitStructuer;

    TimBaseInit();/*时基单元初始化*/

    GPIOTimInit();

    TIM_OCStructInit(&TimOCxInitStructuer);
    // TimOCxInitStructuer.TIM_OCIdleState;
    TimOCxInitStructuer.TIM_OCMode = TIM_OCMode_PWM1;
    // TimOCxInitStructuer.TIM_OCNIdleState;
    // TimOCxInitStructuer.TIM_OCNPolarity;
    TimOCxInitStructuer.TIM_OCPolarity = TIM_OCPolarity_High;/*这个有什么用？试试另一个*/
    // TimOCxInitStructuer.TIM_OutputNState;
    TimOCxInitStructuer.TIM_OutputState = TIM_OutputState_Enable;
    TimOCxInitStructuer.TIM_Pulse=100;/**/
    TIM_OC3Init(PWM_Timx, &TimOCxInitStructuer);

    TIM_OC4Init(PWM_Timx, &TimOCxInitStructuer);

    TIM_OC2Init(PWM_Timx, &TimOCxInitStructuer);

    TIM_Cmd(PWM_Timx, ENABLE);
}
