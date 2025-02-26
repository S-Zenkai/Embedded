#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_tim.h"
#include "OLED.h"
#include "bsp_pwm.h"
#include "bsp_systick.h"

extern uint16_t indexWave[];

uint16_t value = 0;
int16_t i;
int main(void)
{
  BreathingInit();
  SysTick_Init(72);
  while (1)
  {
    // for (i = 0; i < 110; i++)
    // {
      // TIM_SetCompare3(PWM_Timx, indexWave[i]);
      // delay_ms(28);
      // TIM_SetCompare2(PWM_Timx, 0);
      //  delay_ms(10);
      // TIM_SetCompare4(PWM_Timx, 0);
      //  delay_ms(10);
    // }
    //  for (i = 110-1; i >= 0; i--)
    //  {
    //    TIM_SetCompare3(PWM_Timx, indexWave[i]);
    //    delay_ms(28);
    //    TIM_SetCompare2(PWM_Timx, 1024);
    //    //  delay_ms(10);
    //    TIM_SetCompare4(PWM_Timx, 1024);
    //    //  delay_ms(10);
    //  }
    }
}
