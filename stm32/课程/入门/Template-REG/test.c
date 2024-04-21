
#if 0
#include "stm32f10x.h"


int main(void)
{
	
}

void  SystemInit (void)
{
	//使编译器不报错
}
#endif

int main(void)
{
	//打开GPIOB端口的时钟
	*(unsigned int*) 0x40021018 |= (1 << 3);
	//配置I/O口为输出
	*(unsigned int*) 0x40010C00 |= (1 << (4*5));
	//*(unsigned int*) 0x40010C00 |= (1 << (4*1));
	*(unsigned int*) 0x40010C00 |= (1 << (4*0));
	//控制ODR寄存器
	*(unsigned int*) 0x40010C0C &= ~(1 << 5);

}

void  SystemInit (void)
{
	//使编译器不报错
}




