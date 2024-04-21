
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"


//int main(void)
//{
//	//打开GPIOB的时钟
//	RCC -> APB2ENR |= (0x01 << 3);
//	//配置端口为输出
//	GPIOB -> CRL &= ~(0x0f << (4*5));
//	GPIOB -> CRL |= (0x01 << (4*5));
//	//控制ODR寄存器
//	GPIOB -> ODR &= ~(0x01 << 5);
//	//GPIOB -> ODR |= (0x01 << 5);
//}


int main(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct;
	RCC -> APB2ENR |= (0x01 << 3);
	GPIOB -> CRL &= ~(0x0f << (4*0));
	GPIOB -> CRL |= (0x01 << (4*0));
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin0;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB,&GPIO_InitStruct);
	GPIO_ResetBits(GPIOB,GPIO_Pin0);
}

void SystemInit(void)
{
	// 函数体为空，目的是为了骗过编译器不报错
}



