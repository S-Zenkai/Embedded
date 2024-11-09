#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_tim.h"
#include "OLED.h"
#include "bsp_pwm.h"
#include "bsp_systick.h"

#if 1
extern uint16_t indexWave[];
extern __IO uint32_t RGB888;

uint16_t value = 0;
int32_t i;
void Delay(__IO uint32_t nCount) // 简单的延时函数
{
  for (; nCount != 0; nCount--)
    ;
}
int main(void)
{
  BreathingInit();
  SysTick_Init(72);
  while (1)
  {
    for (i = 0; i < 0XFFFFFF;i=i+1000)
    {
      RGB888 = i;
      delay_ms(300);
    }
//     RGB888 = 0xFF00FF;
// //    delay_ms(3000);
// 		Delay(0x1FFFFFF);
// 		RGB888 = 0x8080ff;
// //    delay_ms(3000);
// 		Delay(0x1FFFFFF);
// 		RGB888 = 0xff8000;
// //    delay_ms(3000);
// 		Delay(0x1FFFFFF);
// 		RGB888 = 0xffc90e;
// //    delay_ms(3000);
// 		Delay(0x1FFFFFF);
  }
}
#endif

#if 0
__IO uint32_t rgb_color = 0xFF00FF;

//#define SOFT_DELAY() Delay(0x1FFFFFF);

void Delay(__IO u32 nCount);

/**
 * @brief  主函数
 * @param  无
 * @retval 无
 */
int main(void)
{
  /* 初始化呼吸灯 */
  BreathingInit();

  while (1)
  {

    // 可动态修改颜色，使用各种颜色的呼吸灯
    rgb_color = 0xFF00FF;
    Delay(0x1FFFFFF);

    rgb_color = 0x8080ff;
    Delay(0x1FFFFFF);

    rgb_color = 0xff8000;
    Delay(0x1FFFFFF);

    rgb_color = 0xffc90e;
    Delay(0x1FFFFFF);
  }
}

void Delay(__IO uint32_t nCount) // 简单的延时函数
{
  for (; nCount != 0; nCount--)
    ;
}
/*********************************************END OF FILE**********************/
#endif

