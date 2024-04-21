 /**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   FATFS文件系统移植
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "./usart/bsp_usart.h"
#include "./led/bsp_led.h"
#include "./flash/bsp_spi_flash.h"
#include "ff.h"			


FATFS fsObject;
FIL fp; 
const char wData[]="欢迎使用秉火开发板！";
char rData[4096]="";


UINT bw;
UINT br;

/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
int main(void)
{ 	
	FRESULT res;
	
	LED_GPIO_Config();
	LED_BLUE;
	
	/* 配置串口为：115200 8-N-1 */
	USART_Config();
	printf("\r\n 这是一个FATFS文件系统移植实验 \r\n");
	
	//挂载文件系统
	res = f_mount(&fsObject,"1:",1);

	printf("\r\nf_mount res =%d",res);
	
	if(res == FR_NO_FILESYSTEM)
	{
		res = f_mkfs("1:",0,0);
		printf("\r\nf_mkfs res =%d",res);
		//格式化后要取消挂载再重新挂载文件系统
		res = f_mount(NULL,"1:",1);
		res = f_mount(&fsObject,"1:",1);
		
		printf("\r\nsecond f_mount res =%d",res);
	}

	
	res = f_open(&fp,"1:中文文件名abcdefgadfasd.txt",FA_OPEN_ALWAYS|FA_READ|FA_WRITE);
	printf("\r\nf_open res =%d",res);
	
	if(res == FR_OK)
	{
		res = f_write(&fp,wData,sizeof(wData),&bw);
		printf ("\r\nbw= %d",bw);		

		if(res == FR_OK)
		{
			f_lseek(&fp,0);
			res = f_read (&fp,rData,f_size(&fp),&br);
			if(res == FR_OK)
				printf ("\r\n文件内容：%s br= %d",rData,br);		
		}	
		
		f_close(&fp);
	
	}
	

	while(1);  
}


void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/
