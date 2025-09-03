/**
  ******************************************************************************
  * @file    ic_protocol.h
  * @author  kai
  * @version 
  * @data    2025/06/30
  * @brief   Inter-Chip Communication Protocol，定义协议格式，分别:
  *         1. 定义帧头、检验码等常量
  *         2. 定义所有可能的命令ID
  *         3. 使用struct定义每个命令对应的负载数据结构
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/*数据包帧格式*/
/*******************************************************************************
* 帧头1 | 帧头2 | 消息长度 | 有效载荷(n) |    校验和    |
* 0xAA | 0x55  |  n     |    0xXX    | CK_A | CK_B |
*              |-----CRC校验区域------|
* 
* 有效载荷uint8_t payload[n]:
* payload[0]: 消息类型
* payload[1+]: 消息内容
* 
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IC_PROTOCOL_H
#define __IC_PROTOCOL_H
/* Includes ------------------------------------------------------------------*/


/* Exported define ------------------------------------------------------------*/
/*ICC(双机通信一帧)数据缓冲区大小*/
#define ICC_BUFFER_SIZE 128

/*数据包帧头*/
#define ICC_SYNC1 0xAA
#define ICC_SYNC2 0x55

/*消息类型*/
/*心跳包*/
#define ICC_MSG_TYPE_HEARTBEAT 0x00
/*GPS数据包*/
#define ICC_MSG_TYPE_GPS_DATA 0x01
/*IMU数据包*/
#define ICC_MSG_TYPE_IMU_DATA 0x02
/*遥控器数据包*/
#define ICC_MSG_TYPE_RC_DATA 0x03
/*PWM数据包*/
#define ICC_MSG_TYPE_PWM_DATA 0x04

/*有效载荷长度*/
#define ICC_PAYLOAD_PWM_LENGTH 9
#define ICC_PAYLOAD_RC_LENGTH 19
#define ICC_PAYLOAD_HEARTBEAT_LENGTH 2

/*消息包长度*/
#define ICC_MSG_PWM_LENGTH ICC_PAYLOAD_PWM_LENGTH+5
#define ICC_MSG_RC_LENGTH ICC_PAYLOAD_RC_LENGTH+5
#define ICC_MSG_HEARTBEAT_LENGTH ICC_PAYLOAD_HEARTBEAT_LENGTH+5

/* Exported types ------------------------------------------------------------*/

/*IIC协议解析器状态*/
typedef enum
{
    ICC_STATE_SYNC1=0,
    ICC_STATE_SYNC2,
    ICC_STATE_LENGTH,
    ICC_STATE_PAYLOAD,
    ICC_STATE_CCK_A,
    ICC_STATE_CCK_B
} icc_parser_state_t;



/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif /* __IC_PROTOCOL_H */



