#ifndef _BSP_I2C_EE_H
#define _BSP_I2C_EE_H

#include "stm32f10x.h"


//GPIO宏定义
#define    I2C_EEPROM_GPIO_CLC_FUN              RCC_APB2PeriphClockCmd
#define    I2C_EEPROM_GPIO_CLC                  RCC_APB2Periph_GPIOB
#define    I2C_EEPROM_SCL_GPIO_Port             GPIOB
#define    I2C_EEPROM_SCL_GPIO_Pin              GPIO_Pin_6
#define    I2C_EEPROM_SDA_GPIO_Port             GPIOB
#define    I2C_EEPROM_SDA_GPIO_Pin              GPIO_Pin_7


//I2C宏定义
//时钟
#define    I2C_EEPROM_CLC_FUN                   RCC_APB1PeriphClockCmd
#define    I2C_EEPROM_CLC                       RCC_APB1Periph_I2C1
#define    I2C_Speed                            400000
//主机地址是这样设置的吗
#define    I2C_Host_Addr                        0x44

//从设备地址
#define    I2C_Slava_Addr                       0xA0
//待写入地址
#define    I2C_Slava_Wri_Addr                   0

//事件等待时间
#define    Time_Wait_EV                        (uint16_t)0x1000




/*信息输出*/
#define EEPROM_DEBUG_ON         0

#define EEPROM_INFO(fmt,arg...)           printf("<<-EEPROM-INFO->> "fmt"\n",##arg)
#define EEPROM_ERROR(fmt,arg...)          printf("<<-EEPROM-ERROR->> "fmt"\n",##arg)
#define EEPROM_DEBUG(fmt,arg...)          do{\
                                          if(EEPROM_DEBUG_ON)\
                                          printf("<<-EEPROM-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)
//这个EEPROM_DEBUG有什么用

void I2C_GPIO_Config(void);
uint32_t I2C_EE_WriteByte(uint8_t S_Addr,uint8_t Data);
uint32_t I2C_EE_PageWrite(uint8_t S_Addr,uint8_t* Data,uint32_t Data_Size);
uint32_t I2C_EE_ReadByte(uint8_t S_Addr,uint8_t* Data);
uint32_t I2C_EE_SequentialRead(uint8_t S_Addr,uint8_t* Data,uint32_t Data_Size);
void EEPROM_Read(uint8_t addr,uint8_t *data,uint8_t numByteToRead);
//void Wait_EV(uint16_t Count,ErrorStatus (*I2C_CheckEvent)(I2C_TypeDef* , uint32_t))
uint32_t Wait_EV(uint32_t I2C_EVENT,ErrorStatus (*Wait_EV_I2C_CheckEvent)(I2C_TypeDef* , uint32_t),uint8_t ErrorCode);
void Wait_Write_Complete(void);
																					
void EEPROM_Read(uint8_t addr,uint8_t *data,uint8_t numByteToRead);
void EEPROM_Page_Write(uint8_t addr,uint8_t *data,uint8_t numByteToWrite);
																					
static  uint32_t I2C_TIMEOUT_UserCallback(uint8_t errorCode);
																					
#endif /*_BSP_I2C_EE_H*/
