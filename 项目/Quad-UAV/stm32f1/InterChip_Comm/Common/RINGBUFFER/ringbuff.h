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
#include "stm32f10x.h"
#include <stdbool.h>
/* Exported define ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/*���λ���������ṹ��*/
typedef struct
{
    uint8_t *pbuff;/*ָ�򻺳������ֽ�*/
    uint8_t *pend;/*pbuff+length��ָ�򻺳���β�ֽڵ���һ���ֽ�*/
    uint8_t *pw;/*дָ��*/
    uint8_t *pr;/*��ָ��*/
    uint32_t length;/*����������*/
    FlagStatus overflow;/*�����־*/
} ringbuff_t;

/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  ���rb�ṹ�������Ϣ
  * @note   
  * @param  rb:���λ������ṹ��
  * @retval ��
  */
static inline void rb_clear(ringbuff_t *rb)
{
    rb->pr = rb->pbuff;
    rb->pw = rb->pbuff;
    rb->overflow = RESET;
}


/**
  * @brief  ���λ�����ѹ��һ���ֽ�
  * @note   inline�ؼ��ֿ��Խ���������չ�������ٺ������ÿ������������Ӵ������
  * @param  value:Ҫѹ���ֵ
  * @param  rb:���λ������ṹ��
  * @retval ��
  */
static inline void rb_push(uint8_t value, ringbuff_t *rb)
{
    uint8_t *pw_next = rb->pw + 1;
    if (pw_next == rb->pend)
        pw_next = rb->pbuff;
    /*pw_next=rb->pr��Ϊ����������rb->pr=rb->pw��Ϊ��������*/
    /*�����������ֻ��������Ϳ�*/
    if (pw_next != rb->pr)
    {
        *rb->pw = value;
        rb->pw = pw_next;
    }
    else
        rb->overflow = SET;
}

/**
  * @brief  ���λ�����ѹ��һ���ֽ�
  * @note   inline�ؼ��ֿ��Խ���������չ�������ٺ������ÿ������������Ӵ������
  * @param  rb:���λ������ṹ��
  * @retval ѹ����ֵ
  */
static inline uint8_t rb_pop(ringbuff_t *rb)
{
    uint8_t ret;
    /*����������ʱ*/
    if(rb->pw==rb->pr)
        return 0;
    ret = *rb->pr;
    rb->pr++;
    if(rb->pr==rb->pend)
        rb->pr = rb->pbuff;
    return ret;
}

/**
  * @brief  ��ȡ��ǰ������ʣ����������
  * @note   
  * @param  ��
  * @retval ��
  */
static inline uint32_t rb_GetDataCounter(ringbuff_t *rb)
{
    return (rb->pr - rb->pw + rb->length) % rb->length;
}

/**
  * @brief  �жϻ������Ƿ�Ϊ��
  * @note   
  * @param  ��
  * @retval Ϊ�շ���true�����򷵻�false
  */
static inline bool rb_IsEmpty(ringbuff_t *rb)
{
    return rb->pw == rb->pr;
}

/**
  * @brief  �жϻ������Ƿ���
  * @note   ע�⣬ͨ��overflow(�����־)�ж�ʵʱ�Բ���
  * @param  ��
  * @retval ��
  */
 static inline bool rb_IsFull(ringbuff_t *rb)
 {
     return !((rb->pw - rb->pr + rb->length + 1) % rb->length);
 }

void rb_init(uint8_t *buff, uint32_t len, ringbuff_t *rb);

#endif /*__RINGBUFF_H*/
