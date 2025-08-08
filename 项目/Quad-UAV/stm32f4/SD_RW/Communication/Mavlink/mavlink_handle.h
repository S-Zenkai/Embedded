/**
  ******************************************************************************
  * @file    mavlink.c
  * @author  
  * @version V1.0.0
  * @data    2025/06/09
  * @brief   MAVlink”¶”√≤„¥˙¬Î
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAVLINK_HANDLE_H
#define __MAVLINK_HANDLE_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include "ringbuff.h"
#include "bsp_usart.h"
#include "pro_common.h"
#include "bsp_systick.h"

#include "mavlink_types.h"
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES MAVLINK_Send_Buffer
void MAVLINK_Send_Buffer(mavlink_channel_t chan, const uint8_t *buf, int length);  

#include "mavlink.h"
/* Exported define ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void mavlink_send_heartbeat(void);
void mavlink_send_raw_imu(void);
bool MAVLINK_Parse(void);
void MAVLINK_RB_Init(void);
#endif /* __MAVLINK_HANDLE_H */








