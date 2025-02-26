/**
 ****************************************************************************************************
 * @file        freertos.c
 * @author      
 * @version     V1.4
 * @date        2024.10.15
 * @brief       FreeRTOS�Զ�̬��ʽ��������
 * @license     
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ��F103VET6
 *
 ****************************************************************************************************
 */

#include "FreeRTOS_demo.h"
#include "FreeRTOS.h"
#include "task.h"

#if 0
BaseType_t xTaskCreate( TaskFunction_t pxTaskCode,
                            const char * const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                            const configSTACK_DEPTH_TYPE usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pxCreatedTask )
#endif
/**
  * ��ʼ������غ궨�壬�����������������ջ���������ȼ���������
  */
#define  StartTask_stack           128
#define  StartTask_prio            1
TaskHandle_t  StartTask_handler;
void StartTask(void* pvParameters);


/**
  * ����1��غ궨�壬���������ջ���������ȼ���������
  */
#define  Task1_stack           128
#define  Task1_prio            2
TaskHandle_t  Task1_handler;
void Task1(void* pvParameters);

/**
  * ����2��غ궨�壬���������ջ���������ȼ���������
  */
#define  Task2_stack           128
#define  Task2_prio            3
TaskHandle_t  Task2_handler;
void Task2(void* pvParameters);

/**
  * ����3��غ궨�壬���������ջ���������ȼ���������
  */
#define  Task3_stack           128
#define  Task3_prio            4
TaskHandle_t  Task3_handler;
void Task3(void* pvParameters);


/**
  * @brief  �Զ�̬��ʽ����һ����ʼ����
  * @param  ��
  * @retval ��
  */
void FreeRTOS_demo(void)
{
	xTaskCreate(  (TaskFunction_t)           StartTask,                /*������*/
                (const char *)             "StartTask",              /*������*/
                (configSTACK_DEPTH_TYPE)   StartTask_stack,          /*�����ջ*/
                (void *)                   NULL,                     /*�����������*/
                (UBaseType_t)              StartTask_prio,           /*�������ȼ�*/
                (TaskHandle_t *)           &StartTask_handler        /*������*/ 
						 );
	vTaskStartScheduler();
}

/**
  * @brief  ��ʼ���񣬴�������1.2.3��ɾ��
  * @param  ��
  * @retval ��
  */
void StartTask(void* pvParameters)
{
	taskENTER_CRITICAL();                                          /* �����ٽ��� */
	
	xTaskCreate(  (TaskFunction_t)           Task1,                /*������*/
                (const char *)             "Task1",              /*������*/
                (configSTACK_DEPTH_TYPE)   Task1_stack,          /*�����ջ*/
                (void *)                   NULL,                 /*�����������*/
                (UBaseType_t)              Task1_prio,           /*�������ȼ�*/
                (TaskHandle_t *)           &Task1_handler        /*������*/ 
						 );
								
	xTaskCreate(  (TaskFunction_t)           Task2,                /*������*/
                (const char *)             "Task2",              /*������*/
                (configSTACK_DEPTH_TYPE)   Task2_stack,          /*�����ջ*/
                (void *)                   NULL,                 /*�����������*/
                (UBaseType_t)              Task2_prio,           /*�������ȼ�*/
                (TaskHandle_t *)           &Task2_handler        /*������*/ 
						 );
								
	xTaskCreate(  (TaskFunction_t)           Task3,                /*������*/
                (const char *)             "Task3",              /*������*/
                (configSTACK_DEPTH_TYPE)   Task3_stack,          /*�����ջ*/
                (void *)                   NULL,                 /*�����������*/
                (UBaseType_t)              Task3_prio,           /*�������ȼ�*/
                (TaskHandle_t *)           &Task3_handler        /*������*/ 
						 );
	vTaskDelete(NULL);
	taskEXIT_CRITICAL();                                           /* �˳��ٽ��� */
}

/**
  * @brief  ����1
  * @param  ��
  * @retval ��
  */
void Task1(void* pvParameters)
{
	while(1)
	{
		
	}  
}

/**
  * @brief  ����2
  * @param  ��
  * @retval ��
  */
void Task2(void* pvParameters)
{
	while(1)
	{
		
	}    
}

/**
  * @brief  ����3,����1�����£�ɾ������1������2�����£�ɾ�������
  * @param  ��
  * @retval ��
  */
void Task3(void* pvParameters)
{
	while(1)
	{
	}   
}

#if 0
/**
 ****************************************************************************************************
 * @file        freertos.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.4
 * @date        2022-01-04
 * @brief       FreeRTOS ��ֲʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "freertos_demo.h"
#include "bsp_usart.h"
#include "bsp_led.h"
/*FreeRTOS*********************************************************************************************/
#include "FreeRTOS.h"
#include "task.h"

/******************************************************************************************************/
/*FreeRTOS����*/

/* START_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define START_TASK_PRIO 1                   /* �������ȼ� */
#define START_STK_SIZE  128                 /* �����ջ��С */
TaskHandle_t            StartTask_Handler;  /* ������ */
void start_task(void *pvParameters);        /* ������ */

/* TASK1 ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define TASK1_PRIO      2                   /* �������ȼ� */
#define TASK1_STK_SIZE  128                 /* �����ջ��С */
TaskHandle_t            Task1Task_Handler;  /* ������ */
void task1(void *pvParameters);             /* ������ */

/* TASK2 ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define TASK2_PRIO      3                   /* �������ȼ� */
#define TASK2_STK_SIZE  128                 /* �����ջ��С */
TaskHandle_t            Task2Task_Handler;  /* ������ */
void task2(void *pvParameters);             /* ������ */

/******************************************************************************************************/


/**
 * @brief       FreeRTOS������ں���
 * @param       ��
 * @retval      ��
 */
void freertos_demo(void)
{    
    xTaskCreate((TaskFunction_t )start_task,            /* ������ */
                (const char*    )"start_task",          /* �������� */
                (uint16_t       )START_STK_SIZE,        /* �����ջ��С */
                (void*          )NULL,                  /* ������������Ĳ��� */
                (UBaseType_t    )START_TASK_PRIO,       /* �������ȼ� */
                (TaskHandle_t*  )&StartTask_Handler);   /* ������ */
    vTaskStartScheduler();
}

/**
 * @brief       start_task
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           /* �����ٽ��� */
    /* ��������1 */
    xTaskCreate((TaskFunction_t )task1,
                (const char*    )"task1",
                (uint16_t       )TASK1_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK1_PRIO,
                (TaskHandle_t*  )&Task1Task_Handler);
    /* ��������2 */
    xTaskCreate((TaskFunction_t )task2,
                (const char*    )"task2",
                (uint16_t       )TASK2_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK2_PRIO,
                (TaskHandle_t*  )&Task2Task_Handler);
    vTaskDelete(StartTask_Handler); /* ɾ����ʼ���� */
    taskEXIT_CRITICAL();            /* �˳��ٽ��� */
}

/**
 * @brief       task1
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void task1(void *pvParameters)
{    
    while(1)
    {
        LEDR_TOGGLE;                                                  /* LED0��˸ */
        vTaskDelay(1000);                                               /* ��ʱ1000ticks */
    }
}

/**
 * @brief       task2
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void task2(void *pvParameters)
{
    while(1)
    {
        LEDG_TOGGLE;
        vTaskDelay(500);                           /* ��ʱ1000ticks */
    }
}

#endif

