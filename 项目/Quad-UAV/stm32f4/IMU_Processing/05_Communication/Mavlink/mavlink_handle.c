/**
 ******************************************************************************
 * @file    my_mavlink.c
 * @author
 * @version V1.0.0
 * @data    2025/06/09
 * @brief   MAVlink应用层代码
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/

#include "mavlink_handle.h"
#include "com_data.h"

/** @addtogroup MAVlink
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*缓冲区管理器*/
// MAVlink-环形缓冲区管理变量
ringbuff_t MAVLINK_RX_RingBuffMgr;
/*MAVlink接收数据缓冲区大小*/
#define MAVLINK_BUFFER_SIZE 128
/*MAVlink接收数据缓冲区数组*/
uint8_t MAVLINK_RX_Buff[MAVLINK_BUFFER_SIZE * 10];
/*存储解析完成的MAVlink消息*/
mavlink_message_t MAVLINK_RX_Message;
/*存储解析状态信息(解析的字节数、消息状态、错误计数等)*/
mavlink_status_t MAVLINK_RX_Status;

/*在“mavlink_helper.h中需要使用”*/
mavlink_system_t mavlink_system =
    {
        1,
        1}; // System ID, 1-255, Component/Subsystem ID, 1-255

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief  MAVlink-环形缓冲区初始化
 * @note
 * @param  无
 * @retval 无
 */
void MAVLINK_RB_Init(void)
{
    // 将m_MAVLINK_RX_Buff和MAVLINK_RX_RingBuffMgr环队列进行关联管理。
    rb_init(MAVLINK_RX_Buff, sizeof(MAVLINK_RX_Buff), &MAVLINK_RX_RingBuffMgr);
}

/**
 * @brief  MAVlink-环形缓冲区清空
 * @note
 * @param  无
 * @retval 无
 */
static void MAVLINK_RB_Clear(void)
{
    rb_clear(&MAVLINK_RX_RingBuffMgr);
}

/**
 * @brief  MAVlink-环形缓冲区溢出判断
 * @note
 * @param  无
 * @retval 溢出为SET，反之为RESET
 */
static FlagStatus MAVLINK_RB_IsOverFlow(void)
{
    return MAVLINK_RX_RingBuffMgr.overflow;
}

/**
 * @brief  MAVlink-环形缓冲区数据写入
 * @note
 * @param  无
 * @retval 无
 */
// static void MAVLINK_RB_Push(uint8_t data)
//{
//	rb_push(data, &MAVLINK_RX_RingBuffMgr);
// }

/**
 * @brief  MAVlink-环形缓冲区数据读取
 * @note
 * @param  无
 * @retval 无
 */
static uint8_t MAVLINK_RB_Pop(void)
{
    uint8_t ret;
    rb_pop(&MAVLINK_RX_RingBuffMgr, &ret);
    return ret;
}

/**
 * @brief  MAVlink-环形缓冲区是否有新数据
 * @note
 * @param  无
 * @retval 有新数据为true，反之为false
 */
static bool MAVLINK_RB_HasNew(void)
{
    return !rb_IsEmpty(&MAVLINK_RX_RingBuffMgr);
}

/**
 * @brief  MAVlink-获取当前缓冲区剩余数据数量
 * @note
 * @param  无
 * @retval 数据计数
 */
// static uint32_t MAVLINK_RB_DataCount(void)
//{
//	return rb_GetDataCounter(&MAVLINK_RX_RingBuffMgr);
// }

/**
 * @brief  MAVlink收发初始化(串口初始化后调用)
 * @note
 * @param  无
 * @retval 无
 */
void MAVLINK_Init(void)
{
    MAVLINK_RB_Init();
}

/**
 * @brief  MAVlink发送函数
 * @note
 * @param  buff: 数据缓冲区
 * @param  length: 数据长度
 * @retval 发送成功为true，反之为false
 */
void MAVLINK_Send_Buffer(mavlink_channel_t chan, const uint8_t *buf, int length)
{
    USART_SendBytes(UART8, buf, length, 2000);
}

/**
 * @brief  处理接收到的MAVlink消息
 * @note
 * @param  无
 * @retval 无
 */
void MAVLINK_Handle(mavlink_message_t msg)
{
    switch (msg.msgid)
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
 * @brief  MAVlink解析函数
 * @note
 * @param  无
 * @retval 无
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
 * @brief  发送心跳包
 * @note
 * @param  无
 * @retval 无
 */
void mavlink_send_heartbeat(void)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    /*判断当前通道是否仅支持MAVlink1发送*/
    /*MAVlink1协议消息ID只支持小于256，说大于256且通道只输出MAVlink1协议消息ID，则不发送*/
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256)
    {
        return;
    }
#endif

    mavlink_heartbeat_t packet = {
        963497464, 17, 84, 151, 218, 3};

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    /*如果当前通道仅支持MAVlink1发送，则将消息字段中关于MAVlink2的扩展字段清零*/
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet, 0, sizeof(packet) - MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
    }
#endif
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, packet.type, packet.autopilot, packet.base_mode, packet.custom_mode, packet.system_status);
}

// extern MPU6000Data_t MPU6000_Data;
/**
 * @brief  发送原始IMU数据
 * @note
 * @param  无
 * @retval 无
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
    packet.xacc = imu_raw_data.acc.x;
    packet.yacc = imu_raw_data.acc.y;
    packet.zacc = imu_raw_data.acc.z;
    packet.xgyro = imu_raw_data.gyro.x;
    packet.ygyro = imu_raw_data.gyro.y;
    packet.zgyro = imu_raw_data.gyro.z;
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

/*MAVlink库自带测试用例*/
#if 0
static void mavlink_test_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    /*判断当前通道是否仅支持MAVlink1发送*/
    /*MAVlink1协议消息ID只支持小于256，说大于256且通道只输出MAVlink1协议消息ID，则不发送*/
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256)
    {
        return;
    }
#endif
    /*mavlink_message_t是一个通用的消息容器，在整个 MAVLink 库中流转的标准化、通用化的结构体*/
    /*而mavlink_heartbeat_t专用结构体，它只用于表示HEARTBEAT这一种消息，它的成员变量是MAVlink协议中定义的所有字段*/
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
    /*如果当前通道仅支持MAVlink1发送，则将消息字段中关于MAVlink2的扩展字段清零*/
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1) - MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
    }
#endif
    /*测试用例1*/
    memset(&packet2, 0, sizeof(packet2));
    /*mavlink_msg_heartbeat_encode函数将packet1编码为MAVlink消息，并存储在msg中*/
    mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &packet1);
    /*mavlink_msg_heartbeat_decode函数将msg中的MAVlink消息解码为packet2*/
    mavlink_msg_heartbeat_decode(&msg, &packet2);
    /*断言，验证编码和解码过程的可逆且无损*/
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
    /*测试用例2*/
    memset(&packet2, 0, sizeof(packet2));
    /*编码*/
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, packet1.type, packet1.autopilot, packet1.base_mode, packet1.custom_mode, packet1.system_status);
    /*解码*/
    mavlink_msg_heartbeat_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
    /*测试用例3*/
    memset(&packet2, 0, sizeof(packet2));
    /*编码，允许MAVlink库为不同的通道维护独立的状态*/
    mavlink_msg_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.type, packet1.autopilot, packet1.base_mode, packet1.custom_mode, packet1.system_status);
    /*解码*/
    mavlink_msg_heartbeat_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
    /*单字节通信*/
    memset(&packet2, 0, sizeof(packet2));
    /*将MAVlink消息编码为字节流，并存储在buffer中*/
    mavlink_msg_to_send_buffer(buffer, &msg);
    /*comm_send_ch可能会将数据通过串口发送出去，并通过一个解析器（解析函数），当发送完成后，更新last_msg指向内容*/
    for (i = 0; i < mavlink_msg_get_send_buffer_length(&msg); i++)
    {
        comm_send_ch(MAVLINK_COMM_0, buffer[i]);
    }
    mavlink_msg_heartbeat_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
    /*mavlink_msg_heartbeat_send是一个高阶便捷函数，将打包、序列化、发送合为一步*/
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
