#ifndef __ILI9341_LCD_H
#define __ILI9341_LCD_H

#include "stm32f10x.h"
#include <stdio.h>


#define USE_ZNZ 1

/*LCD控制信号****************************/
#ifdef USE_ZNZ

	#define     FSMC_BANK_NORSRAMx				FSMC_Bank1_NORSRAM1
	
	#define 		ILI9341_CMD_ADDR  		(__IO uint16_t*)(0x60000000)
  #define 		ILI9341_DATA_ADDR  		(__IO uint16_t*)(0x60020000)
 

	#define      ILI9341_CS_CLK           RCC_APB2Periph_GPIOD
	#define      ILI9341_CS_PORT          GPIOD
	#define      ILI9341_CS_PIN           GPIO_Pin_7

	#define      ILI9341_RST_CLK           RCC_APB2Periph_GPIOE
	#define      ILI9341_RST_PORT          GPIOE
	#define      ILI9341_RST_PIN           GPIO_Pin_1


	#define      ILI9341_BK_CLK           RCC_APB2Periph_GPIOD
	#define      ILI9341_BK_PORT          GPIOD
	#define      ILI9341_BK_PIN           GPIO_Pin_12


	#define      ILI9341_RD_CLK           RCC_APB2Periph_GPIOD
	#define      ILI9341_RD_PORT          GPIOD
	#define      ILI9341_RD_PIN           GPIO_Pin_4

	#define      ILI9341_WR_CLK           RCC_APB2Periph_GPIOD
	#define      ILI9341_WR_PORT          GPIOD
	#define      ILI9341_WR_PIN           GPIO_Pin_5

	#define      ILI9341_DC_CLK           RCC_APB2Periph_GPIOD
	#define      ILI9341_DC_PORT          GPIOD
	#define      ILI9341_DC_PIN           GPIO_Pin_11

#else

	#define     FSMC_BANK_NORSRAMx				FSMC_Bank1_NORSRAM4
	
	#define 		ILI9341_CMD_ADDR  		(__IO uint16_t*)(0x6c000000)
  #define 		ILI9341_DATA_ADDR  		(__IO uint16_t*)(0x6d000000)

	#define      ILI9341_CS_CLK           RCC_APB2Periph_GPIOG
	#define      ILI9341_CS_PORT          GPIOG
	#define      ILI9341_CS_PIN           GPIO_Pin_12

	#define      ILI9341_RST_CLK           RCC_APB2Periph_GPIOG
	#define      ILI9341_RST_PORT          GPIOG
	#define      ILI9341_RST_PIN           GPIO_Pin_11


	#define      ILI9341_BK_CLK           RCC_APB2Periph_GPIOG
	#define      ILI9341_BK_PORT          GPIOG
	#define      ILI9341_BK_PIN           GPIO_Pin_6


	#define      ILI9341_RD_CLK           RCC_APB2Periph_GPIOD
	#define      ILI9341_RD_PORT          GPIOD
	#define      ILI9341_RD_PIN           GPIO_Pin_4

	#define      ILI9341_WR_CLK           RCC_APB2Periph_GPIOD
	#define      ILI9341_WR_PORT          GPIOD
	#define      ILI9341_WR_PIN           GPIO_Pin_5

	#define      ILI9341_DC_CLK           RCC_APB2Periph_GPIOE
	#define      ILI9341_DC_PORT          GPIOE
	#define      ILI9341_DC_PIN           GPIO_Pin_2
	
#endif


#define      ILI9341_D0_CLK                RCC_APB2Periph_GPIOD   
#define      ILI9341_D0_PORT               GPIOD
#define      ILI9341_D0_PIN                GPIO_Pin_14

#define      ILI9341_D1_CLK                RCC_APB2Periph_GPIOD   
#define      ILI9341_D1_PORT               GPIOD
#define      ILI9341_D1_PIN                GPIO_Pin_15

#define      ILI9341_D2_CLK                RCC_APB2Periph_GPIOD   
#define      ILI9341_D2_PORT               GPIOD
#define      ILI9341_D2_PIN                GPIO_Pin_0

#define      ILI9341_D3_CLK                RCC_APB2Periph_GPIOD  
#define      ILI9341_D3_PORT               GPIOD
#define      ILI9341_D3_PIN                GPIO_Pin_1

#define      ILI9341_D4_CLK                RCC_APB2Periph_GPIOE   
#define      ILI9341_D4_PORT               GPIOE
#define      ILI9341_D4_PIN                GPIO_Pin_7

#define      ILI9341_D5_CLK                RCC_APB2Periph_GPIOE   
#define      ILI9341_D5_PORT               GPIOE
#define      ILI9341_D5_PIN                GPIO_Pin_8

#define      ILI9341_D6_CLK                RCC_APB2Periph_GPIOE   
#define      ILI9341_D6_PORT               GPIOE
#define      ILI9341_D6_PIN                GPIO_Pin_9

#define      ILI9341_D7_CLK                RCC_APB2Periph_GPIOE  
#define      ILI9341_D7_PORT               GPIOE
#define      ILI9341_D7_PIN                GPIO_Pin_10

#define      ILI9341_D8_CLK                RCC_APB2Periph_GPIOE   
#define      ILI9341_D8_PORT               GPIOE
#define      ILI9341_D8_PIN                GPIO_Pin_11

#define      ILI9341_D9_CLK                RCC_APB2Periph_GPIOE   
#define      ILI9341_D9_PORT               GPIOE
#define      ILI9341_D9_PIN                GPIO_Pin_12

#define      ILI9341_D10_CLK                RCC_APB2Periph_GPIOE   
#define      ILI9341_D10_PORT               GPIOE
#define      ILI9341_D10_PIN                GPIO_Pin_13

#define      ILI9341_D11_CLK                RCC_APB2Periph_GPIOE   
#define      ILI9341_D11_PORT               GPIOE
#define      ILI9341_D11_PIN                GPIO_Pin_14

#define      ILI9341_D12_CLK                RCC_APB2Periph_GPIOE   
#define      ILI9341_D12_PORT               GPIOE
#define      ILI9341_D12_PIN                GPIO_Pin_15

#define      ILI9341_D13_CLK                RCC_APB2Periph_GPIOD   
#define      ILI9341_D13_PORT               GPIOD
#define      ILI9341_D13_PIN                GPIO_Pin_8

#define      ILI9341_D14_CLK                RCC_APB2Periph_GPIOD   
#define      ILI9341_D14_PORT               GPIOD
#define      ILI9341_D14_PIN                GPIO_Pin_9

#define      ILI9341_D15_CLK                RCC_APB2Periph_GPIOD   
#define      ILI9341_D15_PORT               GPIOD
#define      ILI9341_D15_PIN                GPIO_Pin_10



/*************************************** 调试预用 ******************************************/

#define      DEBUG_DELAY()                



/******************************* 定义 ILI934 常用命令 ********************************/
#define      CMD_SetCoordinateX		 		    0x2A	     //设置X坐标
#define      CMD_SetCoordinateY		 		    0x2B	     //设置Y坐标
#define      CMD_SetPixel		 		          0x2C	     //填充像素



#define RGB888_2_RGB565(R,G,B)		 (uint16_t)(((R & 0x1F) << 11) | ((G & 0x3F) << 5) | (B & 0x1F) )

/*SPI接口定义-结尾****************************/

/*等待超时时间*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

/*信息输出*/
#define FLASH_DEBUG_ON         1

#define FLASH_INFO(fmt,arg...)           printf("<<-FLASH-INFO->> "fmt"\n",##arg)
#define FLASH_ERROR(fmt,arg...)          printf("<<-FLASH-ERROR->> "fmt"\n",##arg)
#define FLASH_DEBUG(fmt,arg...)          do{\
                                          if(FLASH_DEBUG_ON)\
                                          printf("<<-FLASH-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

																					
																					
void ILI9341_Init(void);
void ILI9341_Draw_Rec(void);																					

#endif /* __ILI9341_LCD_H */

