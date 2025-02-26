#include "stm32f10x.h"
#include "bsp_oled.h"
#include "bsp_delay.h"

int main(void)
{
	
	delay_init(72);
	OLED_IIC_init();
	OLED_DisplayChar(0,0,'5',H16W8);
	OLED_DisplayChar(16,0,'a',H16W8);
	OLED_DisplayChar(32,0,'a',H16W8);
	OLED_DisplayChar(48,0,'a',H16W8);
	OLED_DisplayCharString(0,0,"abcdef",H16W8);
	OLED_DisplayNumber(16,0,123456,6,H16W8);
    OLED_DisplaySignNumber(32,0,-123456,6,H16W8);
	OLED_DisplayHexNum(48,0,0x18AEF,5,H16W8);
	OLED_DisplayBinNum(0,56,0x18,8,H16W8);
	OLED_DisplayFloat(16,56,123.456, 3, 2, H16W8);
	while(1);
}




