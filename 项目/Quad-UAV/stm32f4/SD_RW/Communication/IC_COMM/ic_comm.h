/**
  ******************************************************************************
  * @file    ic_comm.h
  * @author  kai
  * @version 
  * @data    2025/06/30
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
 /* Define to prevent recursive inclusion -------------------------------------*/
 #ifndef __IC_COMM_H
 #define __IC_COMM_H
 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"
 
 /* Exported define ------------------------------------------------------------*/
 
/*重新定ICC_DEBUG函数*/
//#define DEBUG_ICC_ENABLE
#ifdef DEBUG_ICC_ENABLE
#define ICC_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define ICC_DEBUG(format, ...)
#endif

 /* Exported types ------------------------------------------------------------*/





 /* Exported contants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
void IC_Comm_Init(void);
void ICC_Comm_Test(void);
void ICC_Send_Heartbeat(void);
void IC_Comm_Send_Motor_Value(void);
void IC_Comm_Task(void);
#endif /* __IC_COMM_H */
 
