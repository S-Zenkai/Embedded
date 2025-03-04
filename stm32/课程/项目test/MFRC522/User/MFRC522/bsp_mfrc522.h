#ifndef _BSP_MFRC522_H
#define _BSP_MFRC522_H
#include "stm32f10x.h"
#include <stdio.h>


//GPIO初始化相关宏
#define    MFRC522_SPI_GPIO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define    MFRC522_SPI_GPIO_CLK               RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB
#define    MFRC522_SPI_GPIO_CS_PORT           GPIOA
#define    MFRC522_SPI_GPIO_CS_PIN            GPIO_Pin_4
#define    MFRC522_SPI_GPIO_SCK_PORT          GPIOA
#define    MFRC522_SPI_GPIO_SCK_PIN           GPIO_Pin_5
#define    MFRC522_SPI_GPIO_MOSI_PORT         GPIOA
#define    MFRC522_SPI_GPIO_MOSI_PIN          GPIO_Pin_7
#define    MFRC522_SPI_GPIO_MISO_PORT         GPIOA
#define    MFRC522_SPI_GPIO_MISO_PIN          GPIO_Pin_6
#define    MFRC522_SPI_GPIO_RST_PORT          GPIOA
#define    MFRC522_SPI_GPIO_RST_PIN           GPIO_Pin_2

//SPI相关宏定义
#define    MFRC522_SPIx                       SPI1
#define    MFRC522_SPI_APBxClock_FUN          RCC_APB2PeriphClockCmd
#define    MFRC522_SPI_CLK                    RCC_APB2Periph_SPI1
//#define    MFRC522_SPI_Free                   GPIO_ResetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);


//MFRC522寄存器相关宏定义
// PAGE 0
#define     RFU00                 0x00    
#define     CommandReg            0x01    
#define     ComIEnReg             0x02    
#define     DivlEnReg             0x03    
#define     ComIrqReg             0x04    
#define     DivIrqReg             0x05
#define     ErrorReg              0x06    
#define     Status1Reg            0x07    
#define     Status2Reg            0x08    
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     RFU0F                 0x0F
// PAGE 1     
#define     RFU10                 0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     RFU1A                 0x1A
#define     RFU1B                 0x1B
#define     RFU1C             	  0x1C
#define     RFU1D                 0x1D
#define     RFU1E                 0x1E
#define     SerialSpeedReg        0x1F
// PAGE 2    
#define     RFU20                 0x20  
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFU23                 0x23
#define     ModWidthReg           0x24
#define     RFU25                 0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// PAGE 3      
#define     RFU30                 0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39  
#define     TestDAC2Reg           0x3A   
#define     TestADCReg            0x3B   
#define     RFU3C                 0x3C   
#define     RFU3D                 0x3D   
#define     RFU3E                 0x3E   
#define     RFU3F		  		        0x3F

//MFRC522相关指令集
#define PCD_IDLE              0x00               //取消当前命令
#define PCD_AUTHENT           0x0E               //验证密钥
#define PCD_RECEIVE           0x08               //接收数据
#define PCD_TRANSMIT          0x04               //发送数据
#define PCD_TRANSCEIVE        0x0C               //发送并接收数据
#define PCD_RESETPHASE        0x0F               //复位
#define PCD_CALCCRC           0x03               //CRC计算

/////////////////////////////////////////////////////////////////////
//Mifare_One卡片命令字
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL           0x26               //寻天线区内未进入休眠状态
#define PICC_REQALL           0x52               //寻天线区内全部卡
#define PICC_ANTICOLL1        0x93               //防冲撞
#define PICC_ANTICOLL2        0x95               //防冲撞
#define PICC_AUTHENT1A        0x60               //验证A密钥
#define PICC_AUTHENT1B        0x61               //验证B密钥
#define PICC_READ             0x30               //读块
#define PICC_WRITE            0xA0               //写块
#define PICC_DECREMENT        0xC0               //扣款
#define PICC_INCREMENT        0xC1               //充值
#define PICC_RESTORE          0xC2               //调块数据到缓冲区
#define PICC_TRANSFER         0xB0               //保存缓冲区中数据
#define PICC_HALT             0x50               //休眠

#define MF522_RST(x)        (x) ? GPIO_SetBits(GPIOA,GPIO_Pin_2):GPIO_ResetBits(GPIOA,GPIO_Pin_2)

/////////////////////////////////////////////////////////////////////
//MF522 FIFO长度定义
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte
#define MAXRLEN  18

/////////////////////////////////////////////////////////////////////
//和MF522通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////
#define MI_OK                     0
#define MI_NOTAGERR               1
#define MI_ERR                    2
//
#define MAX_LEN					  18






#define    SPI_Flag_WaitTime                0x1000
#define    SPI_Status_WaitTime              0x3000

#define    MFRC522_Read_JEDEC_ID              0x9F
#define    Instruction_Write_Enable         0x06
#define    Instruction_Sector_Erase         0x20
#define    Instruction_Read_Data            0x03
#define    MFRC522_Read_Data_Addr             0x0005
#define    MFRC522_Sector_Erase_Addr          0x0005
#define    MFRC522_Page_Program_Addr          0x0005
#define    MFRC522_Invali_Data                0x00
#define    Instruction_Read_Status_Register          0x05
#define    Instruction_Page_Program         0x02
#define    MFRC522_PageByteNum                256


#define          RC522_CS_Enable()         GPIO_ResetBits ( MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN )
#define          RC522_CS_Disable()        GPIO_SetBits ( MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN )

#define          RC522_Reset_Enable()      GPIO_ResetBits( MFRC522_SPI_GPIO_RST_PORT, MFRC522_SPI_GPIO_RST_PIN )
#define          RC522_Reset_Disable()     GPIO_SetBits ( MFRC522_SPI_GPIO_RST_PORT, MFRC522_SPI_GPIO_RST_PIN )


/*信息输出*/
#define EEPROM_DEBUG_ON         1

#define EEPROM_INFO(fmt,arg...)           printf("<<-EEPROM-INFO->> "fmt"\n",##arg)
#define EEPROM_ERROR(fmt,arg...)          printf("<<-EEPROM-ERROR->> "fmt"\n",##arg)
#define EEPROM_DEBUG(fmt,arg...)          do{\
                                          if(EEPROM_DEBUG_ON)\
                                          printf("<<-EEPROM-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

void SPI_MFRC522_Init(void);
void WriteRawRC(uint8_t Addr,uint8_t Data);
uint8_t ReadRawRC(uint8_t Addr);
																					
//uint8_t ReadRawRC ( uint8_t ucAddress );
//void WriteRawRC ( uint8_t ucAddress, uint8_t ucValue );
																					
void SetBitMask ( uint8_t ucReg, uint8_t ucMask );
void ClearBitMask ( uint8_t ucReg, uint8_t ucMask ) ;
void PcdAntennaOn ( void );
void PcdAntennaOff ( void );
void PcdReset ( void );
void M500PcdConfigISOType ( uint8_t ucType );
char PcdComMF522 ( uint8_t ucCommand,
                   uint8_t * pInData, 
                   uint8_t ucInLenByte, 
                   uint8_t * pOutData,
                   uint32_t * pOutLenBit )	;
char PcdRequest ( uint8_t ucReq_code, uint8_t * pTagType );
char PcdAnticoll ( uint8_t * pSnr );
void CalulateCRC ( uint8_t * pIndata, 
                 uint8_t ucLen, 
                 uint8_t * pOutData );
char PcdSelect ( uint8_t * pSnr );
char PcdAuthState ( uint8_t ucAuth_mode, 
                    uint8_t ucAddr, 
                    uint8_t * pKey,
                    uint8_t * pSnr );
char PcdWrite ( uint8_t ucAddr, uint8_t * pData );
char PcdRead ( uint8_t ucAddr, uint8_t * pData );
char PcdHalt( void );
char WriteAmount( uint8_t ucAddr, uint32_t pData );
char ReadAmount( uint8_t ucAddr, uint32_t *pData );

																					

#endif /*_BSP_MFRC522_H*/


