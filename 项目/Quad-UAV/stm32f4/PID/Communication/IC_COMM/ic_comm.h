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
 
/*���¶�ICC_DEBUG����*/
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
#endif /* __IC_COMM_H */
 
