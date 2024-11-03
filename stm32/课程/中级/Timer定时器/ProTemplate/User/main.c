#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_TiMbase.h"


uint16_t value = 0;
int main(void)
{
    LED_GPIO_Config();
    BASIC_TIM_Init();
    // OLED_ShowNum(10, 0, value, 5, OLED_8X16);
//    OLED_Printf(0, 0, OLED_8X16, "%s", "value=");
    while (1)
    {
//        OLED_ShowNum(50, 20, value, 5, OLED_8X16);
//        LEDG_TOGGLE;
//        delay_ms(1000);
//        OLED_Update();
    }
    
}




