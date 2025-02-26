#include "stm32f10x.h"


int main(void)
{
	while(1)
	{
	//打开GPIOB的时钟
	RCC_APB2ENR |= (1 << 3);
	//设置GPIOB为输出
	GPIOB_CRL &= ~(0x0f << (4*1));
	GPIOB_CRL |= (1 << (4*1));
	//设置GPIOB端口输出
	GPIOB_ODR &= ~(1 << 1);
	Sleep(1000);
	}
}

void SystemInit(void)
{
	// 函数体为空，目的是为了骗过编译器不报错
}

