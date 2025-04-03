/*********************************************************************************************************
*
* File                : usart.h
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, CKS
*                                       http://www.cksic.com
*                                          All Rights Reserved
*
*********************************************************************************************************/

#ifndef _USART_H
#define _USART_H

#include <stdio.h>
#include "cs32f10x.h"

#define REC_BUF_LEN 100
typedef struct
{
	u8 buf[REC_BUF_LEN];
	u8 state;
	u8 cnt;
}USART1_REC_STRUCT;
extern USART1_REC_STRUCT Rec;


void USART_Configuration(void);
 


#endif /*_USART_H*/
