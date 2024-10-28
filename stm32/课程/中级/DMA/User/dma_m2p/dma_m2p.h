#ifndef _DMA_M2P_H
#define _DMA_M2P_H
#include "stm32f10x.h"
#include <stdio.h>
#define sz 32
//const uint32_t Asour[sz]={0x01020304,0x05060708,0x090A0B0C,0x0D0E0F10,
//                        0x11121314,0x15161718,0x191A1B1C,0x1D1E1F20,
//                        0x21222324,0x25262728,0x292A2B2C,0x2D2E2F30,
//                        0x31323334,0x35363738,0x393A3B3C,0x3D3E3F40,
//                        0x41424344,0x45464748,0x494A4B4C,0x4D4E4F50,
//                        0x51525354,0x55565758,0x595A5B5C,0x5D5E5F60,
//                        0x61626364,0x65666768,0x696A6B6C,0x6D6E6F70,
//                        0x71727374,0x75767778,0x797A7B7C,0x7D7E7F80};
////uint16_t sz=sizeof(Asour)/sizeof(Asour[1]);
//uint32_t Adest[sz]={0};

extern uint8_t Asour[sz];
#define   DMA_M2M_CLC                 RCC_AHBPeriph_DMA1
#define   DMA_M2M_Buff_Sz             sz
#define   DMA_M2M_Channelx            DMA1_Channel4
#define   DMA_M2M_TCFlag              DMA1_FLAG_TC5
#define   DMA_M2M_Sour                Asour
#define   DMA_M2M_Dest                (USART1_BASE+0x04)


#define    Debug_USART_GPIO           GPIOA
#define    Debug_USART_GPIO_TXPin     GPIO_Pin_9
#define    Debug_USART_GPIO_CLC       RCC_APB2Periph_GPIOA
#define    Debug_USART                USART1
#define    Debug_USART_CLC            RCC_APB2Periph_USART1
#define    Debug_USART_BaudRate       115200
#define    Debug_RCC_PeriphClockCmd     RCC_APB2PeriphClockCmd


void DMA_M2P_Config(void);
void USART_Config(void);

#endif /*_DMA_M2P_H*/
