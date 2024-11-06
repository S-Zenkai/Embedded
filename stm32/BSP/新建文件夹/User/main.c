#include "stm32f10x.h"
#include "bsp_led.h"
//#include "bsp_tim.h"
//#include "OLED.h"
//#include "bsp_pwm.h"
#include "bsp_systick.h"

uint16_t value = 0;
int16_t i;
int main(void)
{
//    TimOCxInit();
	SysTick_Init(72);
	LED_Init();
    while (1)
    {
//        for (i = 0; i <= 100;i++)
//        {
//            TIM_SetCompare3(PWM_Timx, i);
//            delay_ms(10);
//            TIM_SetCompare2(PWM_Timx, i);
//            delay_ms(10);
//            TIM_SetCompare4(PWM_Timx, i);
//            delay_ms(10);
//        }
//        for (i = 100; i >= 0; i--)
//        {
//            TIM_SetCompare3(PWM_Timx, i);
//            delay_ms(10);
//            TIM_SetCompare2(PWM_Timx, i);
//            delay_ms(10);
//            TIM_SetCompare4(PWM_Timx, i);
//            delay_ms(10);
//        }
			LEDG_TOGGLE;
			delay_ms(1000);
    }
}
