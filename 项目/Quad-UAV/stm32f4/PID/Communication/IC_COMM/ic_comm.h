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

/*遥控器数据(9通道)*/
typedef struct
{
    uint16_t channnel1;
    uint16_t channnel2;
    uint16_t channnel3;
    uint16_t channnel4;
    uint16_t channnel5;
    uint16_t channnel6;
    uint16_t channnel7;
    uint16_t channnel8;
    uint16_t channnel9;
} rc_data_t;



 /* Exported contants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
void IC_Comm_Init(void);
void ICC_Comm_Test(void);
void ICC_Send_Heartbeat(void);
void IC_Comm_Send_Motor_Value(void);
#endif /* __IC_COMM_H */
 
