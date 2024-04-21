#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f10x.h"


#define PASS_ID   ((uint32_t)0x1314)


#define CAN_TX_GPIO_PROT  		GPIOB
#define CAN_TX_GPIO_PIN   		GPIO_Pin_9

#define  CAN_RX_GPIO_PORT     GPIOB
#define  CAN_RX_GPIO_PIN      GPIO_Pin_8

#define CAN_GPIO_CLK					RCC_APB2Periph_GPIOB



/*ÐÅÏ¢Êä³ö*/
#define CAN_DEBUG_ON         1

#define CAN_INFO(fmt,arg...)           printf("<<-CAN-INFO->> "fmt"\n",##arg)
#define CAN_ERROR(fmt,arg...)          printf("<<-CAN-ERROR->> "fmt"\n",##arg)
#define CAN_DEBUG(fmt,arg...)          do{\
                                          if(CAN_DEBUG_ON)\
                                          printf("<<-CAN-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);

void CAN_Config(void)	;     


#endif /* __BSP_CAN_H */

