/**
 ******************************************************************************
 * @file    ic_comm.c
 * @author  kai
 * @version
 * @data    2025/06/30
 * @brief   Inter-Chip Communication��ʵ��Э�����ͽ���߼�
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
/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* ���λ���������ṹ�� */
ringbuff_t ICC_RX_RingBuffMgr;

/*ICC-Э�����״̬*/
icc_parser_state_t ICC_Parser_State = ICC_STATE_SYNC1;

/*������ɱ�־*/
FlagStatus ICC_Send_Done_Flag = SET;
/*ң�������ݽ�����ɱ�־*/
FlagStatus RC_Data_Decode_Done = RESET;
/*������������ɱ�־*/
FlagStatus Heartbeat_Decode_Done = RESET;

/**
 * @defgroup ȫ�ֱ���
 * @brief
 * @{
 */

/*��ȫ����*/
uint8_t safety_switch;
/*ң��������(9ͨ��)*/
rc_data_t rc_data;
/*�������ֵ*/
uint16_t motor_value[4];
/**
 * @}
 */

/**
 * @defgroup ICC_RX_Buffer
 * @brief ���ݰ��������״̬�����뻺����
 * @{
 */
/*ICC(˫��ͨ��һ֡)���ݻ�����*/
uint8_t ICC_RX_Buff[ICC_BUFFER_SIZE];
/*�������ݻ�����*/
/*��Ҫͨ��usart���͵����ݣ���÷���ȫ�ֱ����У��Է�����δ��ɣ��ֲ��������ͷţ��������ݴ���*/
static uint8_t ICC_TX_Buff[ICC_BUFFER_SIZE];
/*�������ݻ���������Ž��������н��յ�����*/
uint8_t ICC_RX_Buff_Temp[ICC_BUFFER_SIZE];
/*��Ч�غɳ���*/
uint16_t ICC_Payload_Length;
/*��Ч�غ�����*/
uint16_t ICC_Payload_Index;
/**
 * @}
 */

/**
 * @defgroup ICC_PUT_DATA
 * @brief �����ݷ��뻺������������д���Ƽ��ʼ�<<�����ݷ��뻺����>>
 * @{
 */
/*�����ݷ��뻺����*/
#define ICC_PUT_u8(buf, offset, data) buf[offset] = (uint8_t)(data)
#define ICC_PUT_u16(buf, offset, data)                                          \
    do                                                                          \
    {                                                                           \
        (buf)[(offset)] = (uint8_t)((data) & 0xFF);            /* д����ֽ� */ \
        (buf)[(offset) + 1] = (uint8_t)(((data) >> 8) & 0xFF); /* д����ֽ� */ \
    } while (0)
/*��������ݴ�ŷ�ʽ�����ֽڶ�������(���ʼ�)���������Ӳ������*/
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
 * @brief  ICC-���λ�������ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
static void ICC_RB_Init(void)
{
    rb_init(ICC_RX_Buff, ICC_BUFFER_SIZE, &ICC_RX_RingBuffMgr);
}

/**
 * @brief  ICC-���λ��������
 * @note
 * @param  ��
 * @retval ��
 */
static void ICC_RB_Clear(void)
{
    rb_clear(&ICC_RX_RingBuffMgr);
}

/**
 * @brief  ICC-���λ���������ж�
 * @note
 * @param  ��
 * @retval ���ΪSET����֮ΪRESET
 */
static FlagStatus ICC_RB_IsOverFlow(void)
{
    return ICC_RX_RingBuffMgr.overflow;
}

/**
 * @brief  ICC-���λ��������ݶ�ȡ
 * @note
 * @param  ��
 * @retval ��
 */
static uint8_t ICC_RB_Pop(void)
{
    return rb_pop(&ICC_RX_RingBuffMgr);
}

/**
 * @brief  ICC-���λ������Ƿ���������
 * @note
 * @param  ��
 * @retval ��������Ϊtrue����֮Ϊfalse
 */
static bool ICC_RB_HasNew(void)
{
    return !rb_IsEmpty(&ICC_RX_RingBuffMgr);
}

/**
 * @brief  ICC�շ���ʼ��(���ڳ�ʼ�������)
 * @note
 * @param  ��
 * @retval ��
 */
void IC_Comm_Init(void)
{
    ICC_RB_Init();
}

/**
 * @brief  ICC-Э�����״̬����ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
static void ICC_Parser_Init(void)
{
    ICC_Parser_State = ICC_STATE_SYNC1;
    ICC_Payload_Length = 0;
}

/**
 * @brief  �������ݰ���ص�״̬�����뻺����
 * @note
 * @param  ��
 * @retval ��
 */
static void ICC_Payload_Reset(void)
{
    /*�����ݰ���ص�״̬�������õ���֪״̬*/
    ICC_Payload_Length = 0;
    ICC_Payload_Index = 0;
    memset(ICC_RX_Buff_Temp, 0, ICC_BUFFER_SIZE);
}

/**
 * @brief  ������յ���HEARTBEAT����,��ȡ��ȫ���ص�ֵ
 * @note
 * @param
 * @retval ��
 */
static void ICC_Process_Heartbeat(uint8_t *payload)
{
    safety_switch = payload[0];
    Heartbeat_Decode_Done = SET;
}

/**
 * @brief  ������յ���ң��������
 * @note
 * @param
 * @retval ��
 */
static void ICC_Process_RC(uint8_t *payload)
{
    /*ң��������(9ͨ��)*/
    u16_u8_union_t rc_buff[9];
    /*���ң��������*/
    for (uint8_t i = 0; i < 9; i++)
    {
        rc_buff[i].data_arr[0] = payload[i * 2];
        rc_buff[i].data_arr[1] = payload[i * 2 + 1];
    }
    rc_data.channnel1 = rc_buff[0].data;
    rc_data.channnel2 = rc_buff[1].data;
    rc_data.channnel3 = rc_buff[2].data;
    rc_data.channnel4 = rc_buff[3].data;
    rc_data.channnel5 = rc_buff[4].data;
    rc_data.channnel6 = rc_buff[5].data;
    rc_data.channnel7 = rc_buff[6].data;
    rc_data.channnel8 = rc_buff[7].data;
    rc_data.channnel9 = rc_buff[8].data;
    RC_Data_Decode_Done = SET;
}

/**
 * @brief  ����ICC���ݰ�
 * @note
 * @param  ��
 * @retval ��
 */
static void ICC_Send_Packet(uint8_t *data, uint16_t length)
{
    while (ICC_Send_Done_Flag == RESET)
    {
    };
    ICC_Send_Done_Flag = RESET;
    /*���ｫ���ݷ�����ȫ�ֱ����У���ֹ����δ��ɣ��ֲ��������ͷţ��������ݴ���*/
    /*��Ҫ��ת�����ݷ��ڼ�鷢����ɱ�־֮�󣬷�ֹ�������Ͳ�ͬ���ݰ�ʱ��̬�������µ����ݸ���*/
    memcpy(ICC_TX_Buff, data, length);
    USART_DMA_Send(DMA2_Stream6, ICC_TX_Buff, length);
}

/**
 * @brief  ������յ���һ֡����
 * @note
 * @param  ��
 * @retval ��
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
 * @brief  ������յ�������
 * @note
 * @param  data: ���յ�������
 * @retval ��
 */
void IC_Comm_Parser_Process(void)
{
    uint16_t crc;
    /*������*/
    if (ICC_RB_IsOverFlow() == SET)
    {
        ICC_RB_Clear();
        ICC_DEBUG("ICC_RB_IsOverFlow\r\n");
    }
    while (ICC_RB_HasNew())
    {
        /*��ȡ������*/
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
            /*����У���*/
            if (crc_calculate(ICC_RX_Buff_Temp, ICC_Payload_Length + 1) == crc)
            {
                ICC_Decode(&ICC_RX_Buff_Temp[1]);
            }
            else
            {
                ICC_DEBUG("CRC error\r\n");
            }
            ICC_Payload_Reset(); /*������Ҫ*/
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
 * @brief  ��ȡң��������
 * @note
 * @param  ��
 * @retval ��
 */
static void ICC_Extract_RC_Data(void)
{
    /*��ȡRC����*/
    motor_value[0] = rc_data.channnel1;
    motor_value[1] = rc_data.channnel2;
    motor_value[2] = rc_data.channnel3;
    motor_value[3] = rc_data.channnel4;
}

///**
// * @brief  ����ICC���ݰ�
// * @note
// * @param  ��
// * @retval ��
// */
// static void ICC_Send_Packet(uint8_t *data, uint16_t length)
//{
//    /*���ｫ���ݷ�����ȫ�ֱ����У���ֹ����δ��ɣ��ֲ��������ͷţ��������ݴ���*/
//    memcpy(ICC_TX_Buff, data, length);
//    USART_DMA_Send(DMA2_Stream6, ICC_TX_Buff, length);
//}

/**
 * @brief  ����������
 * @note
 * @param  ��
 * @retval ��
 */
void ICC_Send_Heartbeat(void)
{
    uint8_t heartbeat_packet[ICC_MSG_HEARTBEAT_LENGTH];
    ICC_PUT_u8(heartbeat_packet, 0, ICC_SYNC1);
    ICC_PUT_u8(heartbeat_packet, 1, ICC_SYNC2);
    ICC_PUT_u8(heartbeat_packet, 2, ICC_PAYLOAD_HEARTBEAT_LENGTH);
    ICC_PUT_u8(heartbeat_packet, 3, ICC_MSG_TYPE_HEARTBEAT);
    ICC_PUT_u8(heartbeat_packet, 4, 0x00);
    /*����У���*/
    uint16_t crc = crc_calculate(&heartbeat_packet[2], ICC_PAYLOAD_HEARTBEAT_LENGTH + 1);
    ICC_PUT_u16(heartbeat_packet, 5, crc);
    /*�����������ݰ�*/
    ICC_Send_Packet(heartbeat_packet, ICC_MSG_HEARTBEAT_LENGTH);
}

/**
 * @brief  ����������ֵ�������Э������
 * @note   ����ֻ��ͨ�����ݷ���
 * @param  ��
 * @retval ��
 */
static void ICC_Send_Motor_Value(void)
{
    /*����������ֵ*/
    uint8_t motor_packet[ICC_MSG_PWM_LENGTH];
    ICC_PUT_u8(motor_packet, 0, ICC_SYNC1);
    ICC_PUT_u8(motor_packet, 1, ICC_SYNC2);
    ICC_PUT_u8(motor_packet, 2, ICC_PAYLOAD_PWM_LENGTH);
    ICC_PUT_u8(motor_packet, 3, ICC_MSG_TYPE_PWM_DATA);
    ICC_PUT_u16(motor_packet, 4, motor_value[0]);
    ICC_PUT_u16(motor_packet, 6, motor_value[1]);
    ICC_PUT_u16(motor_packet, 8, motor_value[2]);
    ICC_PUT_u16(motor_packet, 10, motor_value[3]);
    /*����У���*/
    uint16_t crc = crc_calculate(&motor_packet[2], ICC_PAYLOAD_PWM_LENGTH + 1);
    ICC_PUT_u16(motor_packet, 12, crc);
    /*���͵�����ݰ�*/
    ICC_Send_Packet(motor_packet, ICC_MSG_PWM_LENGTH);
}

/**
 * @brief  ���͵��ֵ��Э������
 * @note
 * @param  ��
 * @retval ��
 */
void IC_Comm_Send_Motor_Value(void)
{
    if (RC_Data_Decode_Done == SET)
    {
        RC_Data_Decode_Done = RESET;
        /*��ȡRC����*/
        ICC_Extract_RC_Data();
        /*���͵�����ݰ�*/
        ICC_Send_Motor_Value();
        ICC_Send_Heartbeat();
    }
}

/**
 * @brief  ����˫��ͨ��
 * @note
 * @param  ��
 * @retval ��
 */
uint8_t test_data1;
void ICC_Comm_Test(void)
{
    static uint32_t heartbeat_last_time = 0;
    //		while (ICC_RB_HasNew())
    //    {
    //        /*��ȡ������*/
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
    /*����*/
    /*����������*/
    /*���͵�����ݰ�*/
    IC_Comm_Send_Motor_Value();
    /*����*/
    IC_Comm_Parser_Process();
    if (Heartbeat_Decode_Done == SET)
    {
        Heartbeat_Decode_Done = RESET;
        heartbeat_last_time = GetTick();
    }
    if (Heartbeat_Decode_Done == RESET)
    {
        if (GetTick() - heartbeat_last_time > 500)
        {
            //    ICC_DEBUG("F1�źŶ�ʧ\r\n");
        }
    }
}

/**
 * @}
 */
