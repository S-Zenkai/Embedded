#include "stm32f10x.h"
#include <stdio.h>
#include "bsp_usart.h"
#include "bsp.h"
#include "./I2C_Software/bsp_i2c.h"
#if 1
int main(void)
{
	uint8_t ret;
	USART_Config();
	ret=CheckDevice(0x78);
	if(ret)
		printf("≤‚ ‘“Ï≥£\n");
	else
		printf("≤‚ ‘’˝≥£\n");
	while(1);
}


#endif


