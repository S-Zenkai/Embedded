/*********************************************************************************************************
*
* File                : systick.h
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

#ifndef __SYSTICK_H
#define __SYSTICK_H 

/* Includes ------------------------------------------------------------------*/	   
#include "cs32f10x.h"

/* Private function prototypes -----------------------------------------------*/
void Delay_Init(void);
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);

#endif
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/






























