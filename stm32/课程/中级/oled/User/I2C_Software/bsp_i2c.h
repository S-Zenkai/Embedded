/**
 ****************************************************************************************************
 * @file        bsp_i2c.h
 * @author      
 * @version     
 * @date        2024-09-10
 * @brief       模拟IIC实验
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:stm32f103vet6
 *
 ****************************************************************************************************
 */
 

#ifndef _BSP_I2C_H
#define _BSP_I2C_H


#include "stm32f10x.h"
#include <stdio.h>

/** SCL与SDA相关GPIO
 *  SCL:GPIOA4  SDA:GPIOA5
 */
#define    I2C_SCL_GPIO           GPIOA
#define    I2C_SCL_GPIO_Pin       GPIO_Pin_4
#define    I2C_SDA_GPIO           GPIOA
#define    I2C_SDA_GPIO_Pin       GPIO_Pin_5
#define    I2C_GPIO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define    I2C_GPIO_SCK_CLK           RCC_APB2Periph_GPIOA

//操作SCL、SDA总线
#define    I2C_SCL_Set()             I2C_SCL_GPIO->BSRR=I2C_SCL_GPIO_Pin
#define    I2C_SCL_Reset()           I2C_SCL_GPIO->BRR=I2C_SCL_GPIO_Pin
#define    I2C_SDA_Set()             I2C_SDA_GPIO->BSRR=I2C_SDA_GPIO_Pin
#define    I2C_SDA_Reset()           I2C_SDA_GPIO->BRR=I2C_SDA_GPIO_Pin
#define    I2C_SDA_Read()            (I2C_SDA_GPIO->IDR)&I2C_SDA_GPIO_Pin




/*信息输出*/
#define EEPROM_DEBUG_ON         1

#define EEPROM_INFO(fmt,arg...)           printf("<<-EEPROM-INFO->> "fmt"\n",##arg)
#define EEPROM_ERROR(fmt,arg...)          printf("<<-EEPROM-ERROR->> "fmt"\n",##arg)
#define EEPROM_DEBUG(fmt,arg...)          do{\
                                          if(EEPROM_DEBUG_ON)\
                                          printf("<<-EEPROM-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)


void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
uint8_t I2C_WaitAck(void);
void I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(void);
uint8_t CheckDevice(uint8_t addr);


#endif /*_BSP_I2C_H*/


