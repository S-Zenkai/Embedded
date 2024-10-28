#ifndef _BSP_SPI_FLASH_H
#define _BSP_SPI_FLASH_H
#include "stm32f10x.h"
#include <stdio.h>


//GPIO初始化相关宏
#define    FLASH_SPI_GPIO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define    FLASH_SPI_GPIO_CLK               RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC
#define    FLASH_SPI_GPIO_CS_PORT           GPIOC
#define    FLASH_SPI_GPIO_CS_PIN            GPIO_Pin_0
#define    FLASH_SPI_GPIO_SCK_PORT          GPIOA
#define    FLASH_SPI_GPIO_SCK_PIN           GPIO_Pin_5
#define    FLASH_SPI_GPIO_MOSI_PORT         GPIOA
#define    FLASH_SPI_GPIO_MOSI_PIN          GPIO_Pin_7
#define    FLASH_SPI_GPIO_MISO_PORT         GPIOA
#define    FLASH_SPI_GPIO_MISO_PIN          GPIO_Pin_6

//SPI相关宏定义
#define    FLASH_SPIx                       SPI1
#define    FLASH_SPI_APBxClock_FUN          RCC_APB2PeriphClockCmd
#define    FLASH_SPI_CLK                    RCC_APB2Periph_SPI1
//#define    FLASH_SPI_Free                   GPIO_ResetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);

#define    SPI_Flag_WaitTime                0x1000
#define    SPI_Status_WaitTime              0x3000

#define    FLASH_Read_JEDEC_ID              0x9F
#define    Instruction_Write_Enable         0x06
#define    Instruction_Sector_Erase         0x20
#define    Instruction_Read_Data            0x03
#define    Flash_Read_Data_Addr             0x0005
#define    Flash_Sector_Erase_Addr          0x0005
#define    Flash_Page_Program_Addr          0x0005
#define    FLASH_Invali_Data                0x00
#define    Instruction_Read_Status_Register          0x05
#define    Instruction_Page_Program         0x02
#define    Flash_PageByteNum                256



/*信息输出*/
#define EEPROM_DEBUG_ON         1

#define EEPROM_INFO(fmt,arg...)           printf("<<-EEPROM-INFO->> "fmt"\n",##arg)
#define EEPROM_ERROR(fmt,arg...)          printf("<<-EEPROM-ERROR->> "fmt"\n",##arg)
#define EEPROM_DEBUG(fmt,arg...)          do{\
                                          if(EEPROM_DEBUG_ON)\
                                          printf("<<-EEPROM-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

void SPI_Flash_Init(void);
uint16_t SPI_Flash_WritByte(uint8_t Data);
void SPI_Flash_ReadID(uint32_t* ReData);
void SPI_Flash_Sector_Erase(uint32_t Addr);
void SPI_Flash_Read_Data(uint32_t Addr,uint8_t* ReData,uint32_t ByteNum);
uint32_t SPI_Flash_Waitfor(void);
void SPI_Flash_Write_Enable(void);
void SPI_Flash_Page_Program(uint32_t Addr,uint8_t* WriteBuff,uint32_t ByteNum);
void SPI_FLASH_BufferWrite(uint32_t Addr, uint8_t* WriteBuff,uint32_t ByteNum);

#endif /*_BSP_SPI_FLASH_H*/


