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

/*环形缓冲区管理变量*/
ringbuff_t gnss_rb_mng;
/*GNSS接收数据缓冲区*/
uint8_t gnss_rx_buff[GNSS_RX_BUFF_SIZE * 10];
/*波特率*/
uint32_t baudrate;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// /**
//  * @brief  向GPS写数据
//  * @note
//  * @param  data: 数据指针
//  * @param  length: 数据长度
//  * @param  timeout: 超时时间
//  * @retval 写入成功返回true，否则返回false
//  */
// bool GPS_WriteData(uint8_t *data, uint32_t length, uint32_t timeout)
// {
//     return USART_SendBytes(UART4, data, length, timeout);
// }

// /**
//  * @brief  从环形缓冲区读取数据并解析
//  * @note
//  * @param  data: 数据指针
//  * @param  length: 数据长度
//  * @param  timeout: 超时时间
//  * @retval 读取成功返回true，否则返回false
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
 * @brief  重置GPS波特率
 * @note
 * @param  无
 * @retval 无
 */
void GNSS_ResetBaudRate(uint32_t baudrate)
{
    USART4_SetBaudRate(baudrate);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(UART4, ENABLE);
    DMA_Cmd(DMA1_Stream2, ENABLE); /*疑问：这里要不要*/
}

/**
 * @brief  重置环形缓冲区
 * @note
 * @param  无
 * @retval 无
 */
void GNSS_RB_Clear(void)
{
    rb_clear(&gnss_rb_mng);
}

/**
 * @brief  检查缓冲区是否发生溢出
 * @note
 * @param  无
 * @retval 溢出返回1，否则返回0
 */
uint8_t GNSS_RB_CheckOverflow(void)
{
    return gnss_rb_mng.overflow;
}

/**
 * @brief  将接收到的数据压入环形缓冲区
 * @note
 * @param  data: 数据指针
 * @retval 无
 */
void GNSS_RB_PushData(uint8_t data)
{
    rb_push(data, &gnss_rb_mng);
}

/**
 * @brief  从环形缓冲区中弹出一个字节
 * @note
 * @param  无
 * @retval 弹出的值(0表示缓冲区为空)
 */
uint8_t GNSS_RB_PopData(void)
{
    return rb_pop(&gnss_rb_mng);
}

/**
 * @brief  获取当前缓冲区剩余数据数量
 * @note
 * @param  无
 * @retval 剩余数据数量
 */
uint32_t GNSS_RB_GetDataCounter(void)
{
    return rb_GetDataCounter(&gnss_rb_mng);
}

/**
 * @brief  判断缓冲区是否为空
 * @note
 * @param  无
 * @retval 为空返回true，否则返回false
 */
bool GNSS_RB_IsEmpty(void)
{
    return rb_IsEmpty(&gnss_rb_mng);
}

/**
 * @brief  GPS初始化
 * @note
 * @param  无
 * @retval 无
 */
void gps_init(void)
{
    rb_init(gnss_rx_buff, sizeof(gnss_rx_buff), &gnss_rb_mng);
    gnss_configure(&baudrate, OUTPUT_GPS, INPUT_GPS, 30000);
}
/**
 * @}
 */
