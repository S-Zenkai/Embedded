#include <reg51.h>	   //此文件中定义了51的一些特殊功能寄存器

void delay(unsigned int i);  //声明延时函数

void main()
{ 
   
  P1  = 0x00; //置P0口为低电平 
  delay(600); // 调用延时程序
  P1  = 0xff; //置P0口为高电平
  delay(600); // 调用延时程序

}

/*******延时函数*************/
void delay(unsigned int i)
{
    unsigned char j;
    for(i; i > 0; i--)   
        for(j = 255; j > 0; j--);
}












