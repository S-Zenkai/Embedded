/**
  ******************************************************************************
  * @file    ic_protocol.h
  * @author  kai
  * @version 
  * @data    2025/06/30
  * @brief   Inter-Chip Communication Protocol������Э���ʽ���ֱ�:
  *         1. ����֡ͷ��������ȳ���
  *         2. �������п��ܵ�����ID
  *         3. ʹ��struct����ÿ�������Ӧ�ĸ������ݽṹ
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/*���ݰ�֡��ʽ*/
/*******************************************************************************
* ֡ͷ1 | ֡ͷ2 | ��Ϣ���� | ��Ч�غ�(n) |    У���    |
* 0xAA | 0x55  |  n     |    0xXX    | CK_A | CK_B |
*              |-----CRCУ������------|
* 
* ��Ч�غ�uint8_t payload[n]:
* payload[0]: ��Ϣ����
* payload[1+]: ��Ϣ����
* 
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IC_PROTOCOL_H
#define __IC_PROTOCOL_H
/* Includes ------------------------------------------------------------------*/


/* Exported define ------------------------------------------------------------*/
/*ICC(˫��ͨ��һ֡)���ݻ�������С*/
#define ICC_BUFFER_SIZE 128

/*���ݰ�֡ͷ*/
#define ICC_SYNC1 0xAA
#define ICC_SYNC2 0x55

/*��Ϣ����*/
/*������*/
#define ICC_MSG_TYPE_HEARTBEAT 0x00
/*GPS���ݰ�*/
#define ICC_MSG_TYPE_GPS_DATA 0x01
/*IMU���ݰ�*/
#define ICC_MSG_TYPE_IMU_DATA 0x02
/*ң�������ݰ�*/
#define ICC_MSG_TYPE_RC_DATA 0x03
/*PWM���ݰ�*/
#define ICC_MSG_TYPE_PWM_DATA 0x04

/*��Ч�غɳ���*/
#define ICC_PAYLOAD_PWM_LENGTH 9
#define ICC_PAYLOAD_RC_LENGTH 19
#define ICC_PAYLOAD_HEARTBEAT_LENGTH 2

/*��Ϣ������*/
#define ICC_MSG_PWM_LENGTH ICC_PAYLOAD_PWM_LENGTH+5
#define ICC_MSG_RC_LENGTH ICC_PAYLOAD_RC_LENGTH+5
#define ICC_MSG_HEARTBEAT_LENGTH ICC_PAYLOAD_HEARTBEAT_LENGTH+5

/* Exported types ------------------------------------------------------------*/

/*IICЭ�������״̬*/
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



