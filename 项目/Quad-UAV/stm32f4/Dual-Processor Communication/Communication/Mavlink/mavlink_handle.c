/**
 ******************************************************************************
 * @file    my_mavlink.c
 * @author
 * @version V1.0.0
 * @data    2025/06/09
 * @brief   MAVlinkӦ�ò����
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/

#include "mavlink_handle.h"


/** @addtogroup MAVlink
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*������������*/
// MAVlink-���λ������������
ringbuff_t MAVLINK_RX_RingBuffMgr;
/*MAVlink�������ݻ�������С*/
#define MAVLINK_BUFFER_SIZE 128
/*MAVlink�������ݻ���������*/
uint8_t MAVLINK_RX_Buff[MAVLINK_BUFFER_SIZE * 10];
/*�洢������ɵ�MAVlink��Ϣ*/
mavlink_message_t MAVLINK_RX_Message;
/*�洢����״̬��Ϣ(�������ֽ�������Ϣ״̬�����������)*/
mavlink_status_t MAVLINK_RX_Status;

/*�ڡ�mavlink_helpers.h����Ҫʹ�á�*/
//mavlink_system_t mavlink_system =
//    {
//        1,
//        1}; // System ID, 1-255, Component/Subsystem ID, 1-255

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief  MAVlink-���λ�������ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
void MAVLINK_RB_Init(void)
{
    // ��m_MAVLINK_RX_Buff��MAVLINK_RX_RingBuffMgr�����н��й�������
    rb_init(MAVLINK_RX_Buff, sizeof(MAVLINK_RX_Buff), &MAVLINK_RX_RingBuffMgr);
}

/**
 * @brief  MAVlink-���λ��������
 * @note
 * @param  ��
 * @retval ��
 */
static void MAVLINK_RB_Clear(void)
{
    rb_clear(&MAVLINK_RX_RingBuffMgr);
}

/**
 * @brief  MAVlink-���λ���������ж�
 * @note
 * @param  ��
 * @retval ���ΪSET����֮ΪRESET
 */
static FlagStatus MAVLINK_RB_IsOverFlow(void)
{
    return MAVLINK_RX_RingBuffMgr.overflow;
}

/**
 * @brief  MAVlink-���λ���������д��
 * @note
 * @param  ��
 * @retval ��
 */
// static void MAVLINK_RB_Push(uint8_t data)
//{
//	rb_push(data, &MAVLINK_RX_RingBuffMgr);
// }

/**
 * @brief  MAVlink-���λ��������ݶ�ȡ
 * @note
 * @param  ��
 * @retval ��
 */
static uint8_t MAVLINK_RB_Pop(void)
{
    return rb_pop(&MAVLINK_RX_RingBuffMgr);
}

/**
 * @brief  MAVlink-���λ������Ƿ���������
 * @note
 * @param  ��
 * @retval ��������Ϊtrue����֮Ϊfalse
 */
static bool MAVLINK_RB_HasNew(void)
{
    return !rb_IsEmpty(&MAVLINK_RX_RingBuffMgr);
}

/**
 * @brief  MAVlink-��ȡ��ǰ������ʣ����������
 * @note
 * @param  ��
 * @retval ���ݼ���
 */
// static uint32_t MAVLINK_RB_DataCount(void)
//{
//	return rb_GetDataCounter(&MAVLINK_RX_RingBuffMgr);
// }

/**
 * @brief  MAVlink�շ���ʼ��(���ڳ�ʼ�������)
 * @note
 * @param  ��
 * @retval ��
 */
void MAVLINK_Init(void)
{
    MAVLINK_RB_Init();
}

/**
 * @brief  MAVlink���ͺ���
 * @note
 * @param  buff: ���ݻ�����
 * @param  length: ���ݳ���
 * @retval ���ͳɹ�Ϊtrue����֮Ϊfalse
 */
void MAVLINK_Send_Buffer(mavlink_channel_t chan, const uint8_t *buf, int length)
{
    USART_SendBytes(UART8, buf, length, 2000);
}


/**
  * @brief  ������յ���MAVlink��Ϣ
  * @note   
  * @param  ��
  * @retval ��
  */
void MAVLINK_Handle(mavlink_message_t msg)
{
    switch(msg.msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:
		printf("this is heartbeat from QGC/r/n");
		break;
	case MAVLINK_MSG_ID_SYS_STATUS:
//		  osd_vbat = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
//			osd_curr = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)
//			osd_battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
		break;
	default:
		break;
	}
}


/**
 * @brief  MAVlink��������
 * @note
 * @param  ��
 * @retval ��
 */
bool MAVLINK_Parse(void)
{
    bool handled = false;
    if (MAVLINK_RB_IsOverFlow() == SET)
    {
        MAVLINK_RB_Clear();
        return handled;
    }
    if (MAVLINK_RB_HasNew())
    {
        uint8_t data = MAVLINK_RB_Pop();
        if (mavlink_parse_char(MAVLINK_COMM_0, data, &MAVLINK_RX_Message, &MAVLINK_RX_Status))
        {
						MAVLINK_Handle(MAVLINK_RX_Message);
            handled = true;
        }
    }
    return handled;
}




/**
 * @brief  ����������
 * @note
 * @param  ��
 * @retval ��
 */
void mavlink_send_heartbeat(void)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    /*�жϵ�ǰͨ���Ƿ��֧��MAVlink1����*/
    /*MAVlink1Э����ϢIDֻ֧��С��256��˵����256��ͨ��ֻ���MAVlink1Э����ϢID���򲻷���*/
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256)
    {
        return;
    }
#endif

    mavlink_heartbeat_t packet = {
        963497464, 17, 84, 151, 218, 3};

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    /*�����ǰͨ����֧��MAVlink1���ͣ�����Ϣ�ֶ��й���MAVlink2����չ�ֶ�����*/
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet, 0, sizeof(packet) - MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
    }
#endif
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, packet.type, packet.autopilot, packet.base_mode, packet.custom_mode, packet.system_status);
}


//extern MPU6000Data_t MPU6000_Data;
/**
 * @brief  ����ԭʼIMU����
 * @note
 * @param  ��
 * @retval ��
 */
void mavlink_send_raw_imu(void)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_IMU >= 256)
    {
        return;
    }
#endif
    mavlink_raw_imu_t packet;
    packet.xacc = mpu6000_raw_acc.x;
    packet.yacc = mpu6000_raw_acc.y;
    packet.zacc = mpu6000_raw_acc.z;
    packet.xgyro = mpu6000_raw_gyro.x;
    packet.ygyro = mpu6000_raw_gyro.y;
    packet.zgyro = mpu6000_raw_gyro.z;
    packet.time_usec = GetTick();
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_RAW_IMU_MIN_LEN + (char *)&packet, 0, sizeof(packet) - MAVLINK_MSG_ID_RAW_IMU_MIN_LEN);
    }
#endif
    mavlink_msg_raw_imu_send(MAVLINK_COMM_0, packet.time_usec, packet.xacc, packet.yacc, packet.zacc, packet.xgyro, packet.ygyro, packet.zgyro, packet.xmag, packet.ymag, packet.zmag, packet.id, packet.temperature);
}



/*MAVlink���Դ���������*/
#if 0
static void mavlink_test_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    /*�жϵ�ǰͨ���Ƿ��֧��MAVlink1����*/
    /*MAVlink1Э����ϢIDֻ֧��С��256��˵����256��ͨ��ֻ���MAVlink1Э����ϢID���򲻷���*/
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256)
    {
        return;
    }
#endif
    /*mavlink_message_t��һ��ͨ�õ���Ϣ������������ MAVLink ������ת�ı�׼����ͨ�û��Ľṹ��*/
    /*��mavlink_heartbeat_tר�ýṹ�壬��ֻ���ڱ�ʾHEARTBEAT��һ����Ϣ�����ĳ�Ա������MAVlinkЭ���ж���������ֶ�*/
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t i;
    mavlink_heartbeat_t packet_in = {
        963497464, 17, 84, 151, 218, 3};
    mavlink_heartbeat_t packet1, packet2;
    memset(&packet1, 0, sizeof(packet1));
    packet1.custom_mode = packet_in.custom_mode;
    packet1.type = packet_in.type;
    packet1.autopilot = packet_in.autopilot;
    packet1.base_mode = packet_in.base_mode;
    packet1.system_status = packet_in.system_status;
    packet1.mavlink_version = packet_in.mavlink_version;

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    /*�����ǰͨ����֧��MAVlink1���ͣ�����Ϣ�ֶ��й���MAVlink2����չ�ֶ�����*/
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1) - MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
    }
#endif
    /*��������1*/
    memset(&packet2, 0, sizeof(packet2));
    /*mavlink_msg_heartbeat_encode������packet1����ΪMAVlink��Ϣ�����洢��msg��*/
    mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &packet1);
    /*mavlink_msg_heartbeat_decode������msg�е�MAVlink��Ϣ����Ϊpacket2*/
    mavlink_msg_heartbeat_decode(&msg, &packet2);
    /*���ԣ���֤����ͽ�����̵Ŀ���������*/
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
    /*��������2*/
    memset(&packet2, 0, sizeof(packet2));
    /*����*/
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, packet1.type, packet1.autopilot, packet1.base_mode, packet1.custom_mode, packet1.system_status);
    /*����*/
    mavlink_msg_heartbeat_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
    /*��������3*/
    memset(&packet2, 0, sizeof(packet2));
    /*���룬����MAVlink��Ϊ��ͬ��ͨ��ά��������״̬*/
    mavlink_msg_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.type, packet1.autopilot, packet1.base_mode, packet1.custom_mode, packet1.system_status);
    /*����*/
    mavlink_msg_heartbeat_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
    /*���ֽ�ͨ��*/
    memset(&packet2, 0, sizeof(packet2));
    /*��MAVlink��Ϣ����Ϊ�ֽ��������洢��buffer��*/
    mavlink_msg_to_send_buffer(buffer, &msg);
    /*comm_send_ch���ܻὫ����ͨ�����ڷ��ͳ�ȥ����ͨ��һ������������������������������ɺ󣬸���last_msgָ������*/
    for (i = 0; i < mavlink_msg_get_send_buffer_length(&msg); i++)
    {
        comm_send_ch(MAVLINK_COMM_0, buffer[i]);
    }
    mavlink_msg_heartbeat_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
    /*mavlink_msg_heartbeat_send��һ���߽ױ�ݺ���������������л������ͺ�Ϊһ��*/
    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_heartbeat_send(MAVLINK_COMM_1, packet1.type, packet1.autopilot, packet1.base_mode, packet1.custom_mode, packet1.system_status);
    mavlink_msg_heartbeat_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HEARTBEAT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HEARTBEAT) != NULL);
#endif
}
#endif

/**
 * @}
 */
