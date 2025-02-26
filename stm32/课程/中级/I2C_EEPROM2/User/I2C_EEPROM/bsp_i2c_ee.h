#ifndef _BSP_I2C_EE_H
#define _BSP_I2C_EE_H
#include "stm32f10x.h"
#include <stdio.h>


//GPIO初始化相关宏
#define    RCC_I2C_GPIO_PeriphClockCmd     RCC_APB2PeriphClockCmd
#define    I2C_SCL_GPIO_CLC       RCC_APB2Periph_GPIOB
#define    I2C_SDA_GPIO_CLC       RCC_APB2Periph_GPIOB
#define    I2C_SCL_GPIO_Pin       GPIO_Pin_6
#define    I2C_SDA_GPIO_Pin       GPIO_Pin_7
#define    I2C_SCL_GPIO       GPIOB
#define    I2C_SDA_GPIO       GPIOB

//I2C相关宏定义
#define    I2Cx_EE     I2C1
#define    RCC_I2C_PeriphClockCmd     RCC_APB1PeriphClockCmd
#define    I2Cx_CLC     RCC_APB1Periph_I2C1
#define    I2Cx_OwnAddr     0X00
#define    Event_Waiting_Time     0X1000
#define    I2Cx_Slave_Addr     0XA0
#define    EEPROM_PageByteNum     8
//#define    I2Cx_Word_Addr     0X01

/*信息输出*/
#define EEPROM_DEBUG_ON         1

#define EEPROM_INFO(fmt,arg...)           printf("<<-EEPROM-INFO->> "fmt"\n",##arg)
#define EEPROM_ERROR(fmt,arg...)          printf("<<-EEPROM-ERROR->> "fmt"\n",##arg)
#define EEPROM_DEBUG(fmt,arg...)          do{\
                                          if(EEPROM_DEBUG_ON)\
                                          printf("<<-EEPROM-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

void I2C_EE_Config(void);
static  uint32_t I2C_TIMEOUT_UserCallback(uint8_t errorCode);
uint8_t I2C_EE_Byte_Write(uint32_t I2Cx_Word_Addr, uint8_t Data);
uint8_t I2C_EE_Page_Write(uint32_t I2Cx_Word_Addr, uint8_t* Data,uint8_t DataNum);
uint8_t I2C_EE_BufferWrite(uint32_t I2Cx_Word_Addr, uint8_t* Data,uint8_t DataNum);
void I2C_EE_Random_Read(uint32_t I2Cx_Word_Addr,uint8_t* ReceiverData);
void I2C_EE_Sequential_Read(uint32_t I2Cx_Word_Addr,uint8_t* ReceiverData,uint8_t DataNum);
void Wait_Write(void);
#endif /*_BSP_I2C_EE_H*/


