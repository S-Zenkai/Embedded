/**
 ******************************************************************************
 * @file    gps.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/04/09
 * @brief
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gps.h"
#include "ubx.h"


/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*���λ������������*/
ringbuff_t gnss_rb_mng;
/*GNSS�������ݻ�����*/
uint8_t gnss_rx_buff[GNSS_RX_BUFF_SIZE * 10];
/*������*/
uint32_t baudrate;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// /**
//  * @brief  ��GPSд����
//  * @note
//  * @param  data: ����ָ��
//  * @param  length: ���ݳ���
//  * @param  timeout: ��ʱʱ��
//  * @retval д��ɹ�����true�����򷵻�false
//  */
// bool GPS_WriteData(uint8_t *data, uint32_t length, uint32_t timeout)
// {
//     return USART_SendBytes(UART4, data, length, timeout);
// }

// /**
//  * @brief  �ӻ��λ�������ȡ���ݲ�����
//  * @note
//  * @param  data: ����ָ��
//  * @param  length: ���ݳ���
//  * @param  timeout: ��ʱʱ��
//  * @retval ��ȡ�ɹ�����true�����򷵻�false
//  */
// bool GPS_ParseData(void)
// {
//     uint8_t *pr = ubx_rb.pr;
//     uint8_t *pw = ubx_rb.pw;
//     uint8_t *pend = ubx_rb.pend;
//     uint8_t *pbuff = ubx_rb.pbuff;
// }

// bool GPS_ReadData(uint8_t *data, uint32_t length, uint32_t timeout)
// {

//     return USART_ReceiveBytes(UART4, data, length, timeout);
// }

/**
 * @brief  ����GPS������
 * @note
 * @param  ��
 * @retval ��
 */
void GNSS_ResetBaudRate(uint32_t baudrate)
{
    USART4_SetBaudRate(baudrate);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(UART4, ENABLE);
    DMA_Cmd(DMA1_Stream2, ENABLE); /*���ʣ�����Ҫ��Ҫ*/
}

/**
 * @brief  ���û��λ�����
 * @note
 * @param  ��
 * @retval ��
 */
void GNSS_RB_Clear(void)
{
    rb_clear(&gnss_rb_mng);
}

/**
 * @brief  ��黺�����Ƿ������
 * @note
 * @param  ��
 * @retval �������1�����򷵻�0
 */
uint8_t GNSS_RB_CheckOverflow(void)
{
    return gnss_rb_mng.overflow;
}

/**
 * @brief  �����յ�������ѹ�뻷�λ�����
 * @note
 * @param  data: ����ָ��
 * @retval ��
 */
void GNSS_RB_PushData(uint8_t data)
{
    rb_push(data, &gnss_rb_mng);
}

/**
 * @brief  �ӻ��λ������е���һ���ֽ�
 * @note
 * @param  ��
 * @retval ������ֵ(0��ʾ������Ϊ��)
 */
uint8_t GNSS_RB_PopData(void)
{
    return rb_pop(&gnss_rb_mng);
}

/**
 * @brief  ��ȡ��ǰ������ʣ����������
 * @note
 * @param  ��
 * @retval ʣ����������
 */
uint32_t GNSS_RB_GetDataCounter(void)
{
    return rb_GetDataCounter(&gnss_rb_mng);
}

/**
 * @brief  �жϻ������Ƿ�Ϊ��
 * @note
 * @param  ��
 * @retval Ϊ�շ���true�����򷵻�false
 */
bool GNSS_RB_IsEmpty(void)
{
    return rb_IsEmpty(&gnss_rb_mng);
}

/**
 * @brief  GPS��ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
void gps_init(void)
{
    rb_init(gnss_rx_buff, sizeof(gnss_rx_buff), &gnss_rb_mng);
    gnss_configure(&baudrate, OUTPUT_GPS, INPUT_GPS, 30000);
}
/**
 * @}
 */
