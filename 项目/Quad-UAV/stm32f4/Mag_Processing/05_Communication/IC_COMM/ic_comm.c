/**
 ******************************************************************************
 * @file    ic_comm.c
 * @author  kai
 * @version
 * @data    2025/06/30
 * @brief   Inter-Chip Communication，实现协议打包和解包逻辑
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ic_comm.h"
#include "ic_protocol.h"
#include "ringbuff.h"
#include "pro_common.h"
#include "checksum.h"
#include <string.h>
#include "bsp_usart.h"
#include "bsp_systick.h"
#include "com_data.h"
/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* 环形缓冲区管理结构体 */
ringbuff_t ICC_RX_RingBuffMgr;

/*ICC-协议解析状态*/
icc_parser_state_t ICC_Parser_State = ICC_STATE_SYNC1;

/*发送完成标志*/
FlagStatus ICC_Send_Done_Flag = SET;
/*遥控器数据解码完成标志*/
FlagStatus RC_Data_Decode_Done = RESET;
/*心跳包解码完成标志*/
FlagStatus Heartbeat_Decode_Done = RESET;

/**
 * @defgroup 全局变量
 * @brief
 * @{
 */

/*安全开关*/
uint8_t safety_switch;

/*电机控制值*/
uint16_t motor_value[4];
/**
 * @}
 */

/**
 * @defgroup ICC_RX_Buffer
 * @brief 数据包接收相关状态变量与缓冲区
 * @{
 */
/*ICC(双机通信一帧)数据缓冲区*/
uint8_t ICC_RX_Buff[ICC_BUFFER_SIZE];
/*发送数据缓冲区*/
/*需要通过usart发送到数据，最好放入全局变量中，以防发送未完成，局部变量被释放，发送数据错误*/
static uint8_t ICC_TX_Buff[ICC_BUFFER_SIZE];
/*接收数据缓冲区，存放解析过程中接收的数据*/
uint8_t ICC_RX_Buff_Temp[ICC_BUFFER_SIZE];
/*有效载荷长度*/
uint16_t ICC_Payload_Length;
/*有效载荷索引*/
uint16_t ICC_Payload_Index;
/**
 * @}
 */

/**
 * @defgroup ICC_PUT_DATA
 * @brief 将数据放入缓冲区，这样编写优势见笔记<<将数据放入缓冲区>>
 * @{
 */
/*将数据放入缓冲区*/
#define ICC_PUT_u8(buf, offset, data) buf[offset] = (uint8_t)(data)
#define ICC_PUT_u16(buf, offset, data)                                          \
    do                                                                          \
    {                                                                           \
        (buf)[(offset)] = (uint8_t)((data) & 0xFF);            /* 写入低字节 */ \
        (buf)[(offset) + 1] = (uint8_t)(((data) >> 8) & 0xFF); /* 写入高字节 */ \
    } while (0)
/*下面的数据存放方式存在字节对齐问题(见笔记)，可能造成硬件错误*/
// #define ICC_PUT_u16(buf,offset,data)   *(uint16_t *)(&buf[offset]) = (uint16_t)(data)
// #define ICC_PUT_u32(buf,offset,data)   *(uint32_t *)(&buf[offset]) = (uint32_t)(data)
// #define ICC_PUT_i8(buf,offset,data)   buf[offset] = (int8_t)(data)
// #define ICC_PUT_i16(buf,offset,data)   *(int16_t *)(&buf[offset]) = (int16_t)(data)
// #define ICC_PUT_i32(buf,offset,data)   *(int32_t *)(&buf[offset]) = (int32_t)(data)
// #define ICC_PUT_float(buf,offset,data)   *(float *)(&buf[offset]) = (float)(data)
// #define ICC_PUT_double(buf,offset,data)   *(double *)(&buf[offset]) = (double)(data)
/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  ICC-环形缓冲区初始化
 * @note
 * @param  无
 * @retval 无
 */
static void ICC_RB_Init(void)
{
    rb_init(ICC_RX_Buff, ICC_BUFFER_SIZE, &ICC_RX_RingBuffMgr);
}

/**
 * @brief  ICC-环形缓冲区清空
 * @note
 * @param  无
 * @retval 无
 */
static void ICC_RB_Clear(void)
{
    rb_clear(&ICC_RX_RingBuffMgr);
}

/**
 * @brief  ICC-环形缓冲区溢出判断
 * @note
 * @param  无
 * @retval 溢出为SET，反之为RESET
 */
static FlagStatus ICC_RB_IsOverFlow(void)
{
    return ICC_RX_RingBuffMgr.overflow;
}

/**
 * @brief  ICC-环形缓冲区数据读取
 * @note
 * @param  无
 * @retval 无
 */
static uint8_t ICC_RB_Pop(void)
{
		uint8_t ret;
		rb_pop(&ICC_RX_RingBuffMgr,&ret);
		return ret;
		
}

/**
 * @brief  ICC-环形缓冲区是否有新数据
 * @note
 * @param  无
 * @retval 有新数据为true，反之为false
 */
static bool ICC_RB_HasNew(void)
{
    return !rb_IsEmpty(&ICC_RX_RingBuffMgr);
}

/**
 * @brief  ICC-协议解析状态机初始化
 * @note
 * @param  无
 * @retval 无
 */
static void ICC_Parser_Init(void)
{
    ICC_Parser_State = ICC_STATE_SYNC1;
    ICC_Payload_Length = 0;

}

/**
 * @brief  ICC收发初始化(串口初始化后调用)
 * @note
 * @param  无
 * @retval 无
 */
void IC_Comm_Init(void)
{
    ICC_RB_Init();
    ICC_Parser_Init();
}


/**
 * @brief  重置数据包相关的状态变量与缓冲区
 * @note
 * @param  无
 * @retval 无
 */
static void ICC_Payload_Reset(void)
{
    /*将数据包相关的状态变量重置到已知状态*/
    ICC_Payload_Length = 0;
    ICC_Payload_Index = 0;
    memset(ICC_RX_Buff_Temp, 0, ICC_BUFFER_SIZE);
}

/**
 * @brief  处理接收到的HEARTBEAT数据,提取安全开关的值
 * @note
 * @param
 * @retval 无
 */
static void ICC_Process_Heartbeat(uint8_t *payload)
{
    safety_switch = payload[0];
    Heartbeat_Decode_Done = SET;
}

/**
 * @brief  处理接收到的遥控器数据
 * @note
 * @param
 * @retval 无
 */
static void ICC_Process_RC(uint8_t *payload)
{
    /*遥控器数据(9通道)*/
    u16_u8_union_t rc_buff[9];
    /*存放遥控器数据*/
    for (uint8_t i = 0; i < 9; i++)
    {
        rc_buff[i].data_arr[0] = payload[i * 2];
        rc_buff[i].data_arr[1] = payload[i * 2 + 1];
    }
    rc_state.raw.channel1 = rc_buff[0].data;
    rc_state.raw.channel2 = rc_buff[1].data;
    rc_state.raw.channel3 = rc_buff[2].data;
    rc_state.raw.channel4 = rc_buff[3].data;
    rc_state.raw.channel5 = rc_buff[4].data;
    rc_state.raw.channel6 = rc_buff[5].data;
    rc_state.raw.channel7 = rc_buff[6].data;
    rc_state.raw.channel8 = rc_buff[7].data;
    rc_state.raw.channel9 = rc_buff[8].data;
    RC_Data_Decode_Done = SET;
}

/**
 * @brief  发送ICC数据包
 * @note
 * @param  无
 * @retval 无
 */
static void ICC_Send_Packet(uint8_t *data, uint16_t length)
{
    while (ICC_Send_Done_Flag == RESET)
    {
    };
    ICC_Send_Done_Flag = RESET;
    /*这里将数据放入了全局变量中，防止发送未完成，局部变量被释放，发送数据错误*/
    /*需要将转移数据放在检查发送完成标志之后，防止连续发送不同数据包时竞态条件导致的数据覆盖*/
    memcpy(ICC_TX_Buff, data, length);
    USART_DMA_Send(DMA2_Stream6, ICC_TX_Buff, length);
}

/**
 * @brief  解码接收到的一帧数据
 * @note
 * @param  无
 * @retval 无
 */
static void ICC_Decode(uint8_t *payload)
{
    switch (payload[0])
    {
    case ICC_MSG_TYPE_HEARTBEAT:
        ICC_Process_Heartbeat(&payload[1]);
        break;
    case ICC_MSG_TYPE_GPS_DATA:
        break;
    case ICC_MSG_TYPE_IMU_DATA:
        break;
    case ICC_MSG_TYPE_RC_DATA:
        ICC_Process_RC(&payload[1]);
        break;
    default:
        break;
    }
}

/**
 * @brief  处理接收到的数据
 * @note
 * @param  data: 接收到的数据
 * @retval 无
 */
void IC_Comm_Parser(void)
{
    uint16_t crc;
    /*检查溢出*/
    if (ICC_RB_IsOverFlow() == SET)
    {
        ICC_RB_Clear();
        ICC_DEBUG("ICC_RB_IsOverFlow\r\n");
    }
    while (ICC_RB_HasNew())
    {
        /*读取缓冲区*/
        uint8_t data;
        data = ICC_RB_Pop();
        switch (ICC_Parser_State)
        {
        case ICC_STATE_SYNC1:
            if (data == ICC_SYNC1)
            {
                ICC_Parser_State = ICC_STATE_SYNC2;
            }
            else
            {
                ICC_DEBUG("ICC_STATE_SYNC1 error\r\n");
            }
            break;
        case ICC_STATE_SYNC2:
            if (data == ICC_SYNC2)
            {
                ICC_Parser_State = ICC_STATE_LENGTH;
            }
            else
            {
                ICC_Parser_Init();
                ICC_DEBUG("ICC_STATE_SYNC2 error\r\n");
            }
            break;
        case ICC_STATE_LENGTH:
            ICC_Payload_Reset();
            ICC_Payload_Length = data;
            ICC_Payload_Index = 0;
            ICC_RX_Buff_Temp[ICC_Payload_Index++] = data;
            ICC_Parser_State = ICC_STATE_PAYLOAD;
            break;
        case ICC_STATE_PAYLOAD:
            ICC_RX_Buff_Temp[ICC_Payload_Index++] = data;
            if (ICC_Payload_Index == ICC_Payload_Length + 1)
            {
                ICC_Parser_State = ICC_STATE_CCK_A;
            }
            break;
        case ICC_STATE_CCK_A:
            ICC_RX_Buff_Temp[ICC_Payload_Index++] = data;
            ICC_Parser_State = ICC_STATE_CCK_B;
            break;
        case ICC_STATE_CCK_B:
            ICC_RX_Buff_Temp[ICC_Payload_Index++] = data;
            crc = (uint16_t)(ICC_RX_Buff_Temp[ICC_Payload_Length + 2] << 8 | ICC_RX_Buff_Temp[ICC_Payload_Length + 1]);
            /*计算校验和*/
            if (crc_calculate(ICC_RX_Buff_Temp, ICC_Payload_Length + 1) == crc)
            {
                ICC_Decode(&ICC_RX_Buff_Temp[1]);
            }
            else
            {
                ICC_DEBUG("CRC error\r\n");
            }
            ICC_Payload_Reset(); /*调试需要*/
            ICC_Parser_State = ICC_STATE_SYNC1;
            break;
        default:
            ICC_Parser_State = ICC_STATE_SYNC1;
            ICC_DEBUG("ICC_STATE_DEFAULT error\r\n");
            ICC_Payload_Reset();
            break;
        }
    }
}

/**
 * @brief  提取遥控器数据
 * @note
 * @param  无
 * @retval 无
 */
static void ICC_Extract_RC_Data(void)
{
    /*提取RC数据*/
    motor_value[0] = rc_state.raw.channel1;
    motor_value[1] = rc_state.raw.channel2;
    motor_value[2] = rc_state.raw.channel3;
    motor_value[3] = rc_state.raw.channel4;

}



///**
// * @brief  发送ICC数据包
// * @note
// * @param  无
// * @retval 无
// */
// static void ICC_Send_Packet(uint8_t *data, uint16_t length)
//{
//    /*这里将数据放入了全局变量中，防止发送未完成，局部变量被释放，发送数据错误*/
//    memcpy(ICC_TX_Buff, data, length);
//    USART_DMA_Send(DMA2_Stream6, ICC_TX_Buff, length);
//}

/**
 * @brief  发送心跳包
 * @note
 * @param  无
 * @retval 无
 */
void ICC_Send_Heartbeat(void)
{
    uint8_t heartbeat_packet[ICC_MSG_HEARTBEAT_LENGTH];
    ICC_PUT_u8(heartbeat_packet, 0, ICC_SYNC1);
    ICC_PUT_u8(heartbeat_packet, 1, ICC_SYNC2);
    ICC_PUT_u8(heartbeat_packet, 2, ICC_PAYLOAD_HEARTBEAT_LENGTH);
    ICC_PUT_u8(heartbeat_packet, 3, ICC_MSG_TYPE_HEARTBEAT);
    ICC_PUT_u8(heartbeat_packet, 4, 0x00);
    /*计算校验和*/
    uint16_t crc = crc_calculate(&heartbeat_packet[2], ICC_PAYLOAD_HEARTBEAT_LENGTH + 1);
    ICC_PUT_u16(heartbeat_packet, 5, crc);
    /*发送心跳数据包*/
    ICC_Send_Packet(heartbeat_packet, ICC_MSG_HEARTBEAT_LENGTH);
}

/**
 * @brief  打包电机控制值，输出到协处理器
 * @note   这里只是通过数据发送
 * @param  无
 * @retval 无
 */
static void ICC_Send_Motor_Value(void)
{
    /*打包电机控制值*/
    uint8_t motor_packet[ICC_MSG_PWM_LENGTH];
    ICC_PUT_u8(motor_packet, 0, ICC_SYNC1);
    ICC_PUT_u8(motor_packet, 1, ICC_SYNC2);
    ICC_PUT_u8(motor_packet, 2, ICC_PAYLOAD_PWM_LENGTH);
    ICC_PUT_u8(motor_packet, 3, ICC_MSG_TYPE_PWM_DATA);
    ICC_PUT_u16(motor_packet, 4, motor_value[0]);
    ICC_PUT_u16(motor_packet, 6, motor_value[1]);
    ICC_PUT_u16(motor_packet, 8, motor_value[2]);
    ICC_PUT_u16(motor_packet, 10, motor_value[3]);
    /*计算校验和*/
    uint16_t crc = crc_calculate(&motor_packet[2], ICC_PAYLOAD_PWM_LENGTH + 1);
    ICC_PUT_u16(motor_packet, 12, crc);
    /*发送电机数据包*/
    ICC_Send_Packet(motor_packet, ICC_MSG_PWM_LENGTH);
}

/**
 * @brief  发送电机值给协处理器
 * @note
 * @param  无
 * @retval 无
 */
void IC_Comm_Send_Motor_Value(void)
{
    if (RC_Data_Decode_Done == SET)
    {
        RC_Data_Decode_Done = RESET;
        /*提取RC数据*/
        ICC_Extract_RC_Data();
        /*发送电机数据包*/
        ICC_Send_Motor_Value();
        ICC_Send_Heartbeat();
    }
}

/**
  * @brief  双机通信任务
  * @note   
  * @param  无
  * @retval 无
  */
void IC_Comm_Task(void)
{
    IC_Comm_Parser();
}


/**
 * @brief  测试双机通信
 * @note
 * @param  无
 * @retval 无
 */
uint8_t test_data1;
void ICC_Comm_Test(void)
{
    static uint32_t heartbeat_last_time = 0;
    //		while (ICC_RB_HasNew())
    //    {
    //        /*读取缓冲区*/
    //        uint8_t data;
    //        data = ICC_RB_Pop();
    //				ICC_Send_Packet(&data, 1);
    //				printf("%d ", data);
    //		}
    // uint8_t arr[10] = {0xAA, 0x55, 0x01, 0x02, 0x03, 0xAA, 0x55, 0x01, 0x02, 0x04};
    // while (1)
    // {

    //     ICC_Send_Packet(arr, 10);
    //     delay_ms(50);
    // }
    /*发送*/
    /*发送心跳包*/
    /*发送电机数据包*/
    IC_Comm_Send_Motor_Value();
    /*接收*/
    IC_Comm_Parser();
    if (Heartbeat_Decode_Done == SET)
    {
        Heartbeat_Decode_Done = RESET;
        heartbeat_last_time = GetTick();
    }
    if (Heartbeat_Decode_Done == RESET)
    {
        if (GetTick() - heartbeat_last_time > 500)
        {
            //    ICC_DEBUG("F1信号丢失\r\n");
            
        }
    }
}

/**
 * @}
 */
