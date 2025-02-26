/**
 * *****************************************************************************
 * @file        bsp_oled.h
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2024-10-28
 * @copyright   
 * *****************************************************************************
 */

#ifndef _BSP_OLED_H
#define _BSP_OLED_H
#include "stm32f10x.h"
#include "stdio.h"


//oled(ssd1306)地址
#define   OLED_Addr    0x78

//control byte
#define   CmdControlByte  0x00
#define   DataControlByte  0x40

//字体大小
//高16宽8
#define    H16W8      8
//高8宽6
#define    H8W6       6

//字符字体大小
//高
#define    CharH16      16
#define    CharH8       8
//宽
#define    CharW8       8
#define    CharW6       6




void OLED_IIC_init(void);
void SendCommand(uint8_t Cmd);
void OLED_SendByte(uint8_t DataBuff);
void SendBytes(uint8_t* DataBuff,uint16_t num);
void SetCoordinate(uint8_t Page,uint8_t Column);
void OLED_Clear(void);
void OLED_ClearPart(uint16_t X,uint16_t Y,uint16_t Height,uint16_t Width);
void OLED_Invert(void);
void OLED_InvertPart(uint16_t X,uint16_t Y,uint16_t Height,uint16_t Width);
void OLED_Updata(void);
void OLED_UpdataPart(int16_t X,int16_t Y,int16_t Height,int16_t Width);
void OLED_DisplayChar(uint16_t X,uint16_t Y,char Char,uint8_t FontSize);
void OLED_DisplayCharString(uint16_t X,uint16_t Y,char* String,uint8_t FontSize);
void OLED_DisplayNumber(uint16_t X,uint16_t Y,uint32_t Num,uint8_t IntBit,uint8_t FontSize);
void OLED_DisplaySignNumber(uint16_t X,uint16_t Y,int32_t Num,uint8_t IntBit,uint8_t FontSize);
void OLED_DisplayHexNum(uint16_t X,uint16_t Y,int32_t Num,uint8_t IntBit,uint8_t FontSize);
void OLED_DisplayBinNum(uint16_t X,uint16_t Y,int32_t Num,uint8_t IntBit,uint8_t FontSize);
void OLED_DisplayFloat(uint16_t X,uint16_t Y,double Num,uint8_t IntBit,uint8_t DecBit,uint8_t FontSize);

#endif /*_BSP_OLED_H*/


