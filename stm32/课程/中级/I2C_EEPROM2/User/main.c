#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp.h"
#include "bsp_i2c_ee.h"
#if 1
int main(void)
{
	uint8_t i=0;
	uint8_t DataArr[80]={1,2,3,4,5,6,7,8,9};
	uint8_t ReceiveArr[80]={0,1,1,1,1,1,1,1,1};
	uint8_t rbyte;
	USART_Config();
	I2C_EE_Config();
	for(i=0;i<80;i++)
	{
		DataArr[i]=i;
	}
	I2C_EE_Byte_Write(0, 10);
	Wait_Write();
	I2C_EE_Random_Read(0,&rbyte);
	printf("%d\n",rbyte);
//	I2C_EE_Page_Write(0, DataArr,80);
	I2C_EE_BufferWrite(3, DataArr,80);
	Wait_Write();
	I2C_EE_Sequential_Read(3,ReceiveArr,80);
	for(i=0;i<80;i++)
	{
		printf("%d ",ReceiveArr[i]);
	}
	for(i=0;i<80;i++)
	{
		if(ReceiveArr[i]!=DataArr[i])
			break;
	}
	if(i==80)
		printf("¶ÁÐ´²âÊÔ³É¹¦");
	while(1);
}


#endif


