/**
  ******************************************************************************
  * @file    ringbuff.h
  * @author  
  * @version V1.0.0
  * @data    2025/04/10
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RINGBUFF_H
#define __RINGBUFF_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdbool.h>
/* Exported define ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/*环形缓冲区管理结构体*/
typedef struct
{
    uint8_t *pbuff;/*指向缓冲区首字节*/
    uint8_t *pend;/*pbuff+length，指向缓冲区尾字节的下一个字节*/
    uint8_t *pw;/*写指针*/
    uint8_t *pr;/*读指针*/
    uint32_t length;/*缓冲区长度*/
    FlagStatus overflow;/*溢出标志*/
} ringbuff_t;

/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  清除rb结构体相关信息
  * @note   
  * @param  rb:环形缓冲区结构体
  * @retval 无
  */
static inline void rb_clear(ringbuff_t *rb)
{
    rb->pr = rb->pbuff;
    rb->pw = rb->pbuff;
    rb->overflow = RESET;
}


/**
  * @brief  向环形缓冲区压入一个字节
  * @note   inline关键字可以将函数内联展开，减少函数调用开销，但会增加代码体积
  * @param  value:要压入的值
  * @param  rb:环形缓冲区结构体
  * @retval 无
  */
static inline void rb_push(uint8_t value, ringbuff_t *rb)
{
    uint8_t *pw_next = rb->pw + 1;
    if (pw_next == rb->pend)
        pw_next = rb->pbuff;
    /*pw_next=rb->pr视为缓冲区满，rb->pr=rb->pw视为缓冲区空*/
    /*这样可以区分缓冲区满和空*/
    if (pw_next != rb->pr)
    {
        *rb->pw = value;
        rb->pw = pw_next;
    }
    else
        rb->overflow = SET;
}

/**
  * @brief  像环形缓冲区压入多字节数据(小端)
  * @note   
  * @param  value:要压入的值
  * @param  rb:环形缓冲区结构体
  * @retval 无
  */
static inline void rb_push_multi(uint8_t *value, uint8_t len, ringbuff_t *rb)
{
    // for (uint8_t i = 0; i < len; i++)
    // {
    //     rb_push(value[len-i-1], rb);
    // }
    for (uint8_t i = 0; i < len; i++)
    {
        rb_push(value[i], rb);
    }
}

/**
  * @brief  环形缓冲区压出一个字节
  * @note   inline关键字可以将函数内联展开，减少函数调用开销，但会增加代码体积
  * @param  rb:环形缓冲区结构体
  * @retval 压出的值
  */
static inline bool rb_pop(ringbuff_t *rb, uint8_t *value)
{
    /*当缓冲区空时*/
    if(rb->pw==rb->pr)
        return false;
    *value = *rb->pr;
    rb->pr++;
    if(rb->pr==rb->pend)
        rb->pr = rb->pbuff;
    return true;
}

/**
  * @brief  获取当前缓冲区剩余数据数量
  * @note   
  * @param  无
  * @retval 无
  */
static inline uint32_t rb_GetDataCounter(ringbuff_t *rb)
{
    return (rb->pw - rb->pr + rb->length) % rb->length;
}

/**
  * @brief  判断缓冲区是否为空
  * @note   
  * @param  无
  * @retval 为空返回true，否则返回false
  */
static inline bool rb_IsEmpty(ringbuff_t *rb)
{
    return rb->pw == rb->pr;
}

/**
  * @brief  判断缓冲区是否满
  * @note   注意，通过overflow(溢出标志)判断实时性不好
  * @param  无
  * @retval 无
  */
 static inline bool rb_IsFull(ringbuff_t *rb)
 {
     return !((rb->pw - rb->pr + rb->length + 1) % rb->length);
 }

void rb_init(uint8_t *buff, uint32_t len, ringbuff_t *rb);

#endif /*__RINGBUFF_H*/
