#include "stm32f10x.h"
#include "bsp_sys.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "bsp_tim.h"
#include "OLED.h"

uint16_t value = 0;
int main(void)
{
    LED_Init();
    OLED_Init();
    SysTick_Init(72);
    Timer_Init();
    // OLED_ShowNum(10, 0, value, 5, OLED_8X16);
    OLED_Printf(0, 0, OLED_8X16, "%s", "value=");
    while (1)
    {
        OLED_ShowNum(50, 20, value, 5, OLED_8X16);
        OLED_Update();
    }
}
