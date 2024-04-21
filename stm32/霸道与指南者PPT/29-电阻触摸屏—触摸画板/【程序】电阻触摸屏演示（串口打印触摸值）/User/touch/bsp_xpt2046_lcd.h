#ifndef __BSP_XPT2046_LCD_H
#define	__BSP_XPT2046_LCD_H


#include "stm32f10x.h"


/******************************* XPT2046 触摸屏触摸信号指示引脚定义(不使用中断) ***************************/
#define             XPT2046_PENIRQ_GPIO_CLK                        RCC_APB2Periph_GPIOE   
#define             XPT2046_PENIRQ_GPIO_PORT                       GPIOE
#define             XPT2046_PENIRQ_GPIO_PIN                        GPIO_Pin_4

//触屏信号有效电平
#define             XPT2046_PENIRQ_ActiveLevel                     0
#define             XPT2046_PENIRQ_Read()                          GPIO_ReadInputDataBit ( XPT2046_PENIRQ_GPIO_PORT, XPT2046_PENIRQ_GPIO_PIN )



/******************************* XPT2046 触摸屏模拟SPI引脚定义 ***************************/
#define             XPT2046_SPI_GPIO_CLK                         RCC_APB2Periph_GPIOE| RCC_APB2Periph_GPIOD

#define             XPT2046_SPI_CS_PIN		                        GPIO_Pin_13
#define             XPT2046_SPI_CS_PORT		                      GPIOD

#define	            XPT2046_SPI_CLK_PIN	                        GPIO_Pin_0
#define             XPT2046_SPI_CLK_PORT	                        GPIOE

#define	            XPT2046_SPI_MOSI_PIN	                        GPIO_Pin_2
#define	            XPT2046_SPI_MOSI_PORT	                      GPIOE

#define	            XPT2046_SPI_MISO_PIN	                        GPIO_Pin_3
#define	            XPT2046_SPI_MISO_PORT	                      GPIOE


#define             XPT2046_CS_ENABLE()                          GPIO_SetBits ( XPT2046_SPI_CS_PORT, XPT2046_SPI_CS_PIN )    
#define             XPT2046_CS_DISABLE()                         GPIO_ResetBits ( XPT2046_SPI_CS_PORT, XPT2046_SPI_CS_PIN )  

#define             XPT2046_CLK_HIGH()                           GPIO_SetBits ( XPT2046_SPI_CLK_PORT, XPT2046_SPI_CLK_PIN )    
#define             XPT2046_CLK_LOW()                            GPIO_ResetBits ( XPT2046_SPI_CLK_PORT, XPT2046_SPI_CLK_PIN ) 

#define             XPT2046_MOSI_1()                             GPIO_SetBits ( XPT2046_SPI_MOSI_PORT, XPT2046_SPI_MOSI_PIN ) 
#define             XPT2046_MOSI_0()                             GPIO_ResetBits ( XPT2046_SPI_MOSI_PORT, XPT2046_SPI_MOSI_PIN )

#define             XPT2046_MISO()                               GPIO_ReadInputDataBit ( XPT2046_SPI_MISO_PORT, XPT2046_SPI_MISO_PIN )




//检测 ：触摸屏的按下和释放

#define TOUCH_NOT_PRESSED  0
#define TOUCH_PRESSED 		1


typedef enum
{
	XPT2046_STATE_RELEASE = 0,
	XPT2046_STATE_WAITING,
	XPT2046_STATE_PRESSED,
}Touch_State;


void XP2046_Init(void);
void XPT2046_SendCMD(uint8_t cmd);
uint16_t XPT2046_ReceiveData(void)	;
uint8_t touch_detect(void);


#endif /* __BSP_TOUCH_H */

