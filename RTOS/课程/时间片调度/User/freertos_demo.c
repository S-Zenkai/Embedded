/**
 * *****************************************************************************
 * @file        freertos_demo.c
 * @brief       ʱ��Ƭ����ʵ�飬�������ļ��н�ʱ��Ƭ��systick�ж����ڣ���Ϊ50ms��ʹ��ʱ��Ƭ
 *          ���ȣ�ͨ�����ڲ鿴�������д�����
 *              ע�����ڳ���ִ����Ҫһ��ʱ�䣬����һ��ʱ��Ƭ�У�ÿ����������4-5�Σ������Ǳ�׼��5��
 * @author      
 * @date        2024-12-18
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "FreeRTOS_demo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "bsp_systick.h"
/*-----------------------------------macro------------------------------------*/
/*��ʼ������ض��壬�������������������ջ��С�����ȼ�*/
void start_task(void *pvParameters);
#define             START_STACK_SIZE            128
#define             START_TASK_PRIO             1
TaskHandle_t        StartTask_Handle;

/*����1��ض��壬�������������������ջ��С�����ȼ�*/
void task1(void *pvParameters);
#define             TASK1_STACK_SIZE            128
#define             TASK1_PRIO                  2
TaskHandle_t        Task1_Handle;

/*����2��ض��壬�������������������ջ��С�����ȼ�*/
void task2(void *pvParameters);
#define             TASK2_STACK_SIZE            128
#define             TASK2_PRIO                  2
TaskHandle_t        Task2_Handle;
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*----------------------------------function----------------------------------*/
void FreeRTOS_demo(void)
{
    xTaskCreate(    (TaskFunction_t)                    start_task,
                    (char *)                            "start_task",
                    (configSTACK_DEPTH_TYPE)            START_STACK_SIZE,
                    (void *)                            NULL,
                    (UBaseType_t)                       START_TASK_PRIO,
                    (TaskHandle_t *)                    &StartTask_Handle);
    vTaskStartScheduler();
}
/*------------------------------------test------------------------------------*/
/**
 * @brief       ��������
 *
 * @param       pvParameters
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();
    xTaskCreate(    (TaskFunction_t)                    task1,
                    (char *)                            "task1",
                    (configSTACK_DEPTH_TYPE)            TASK1_STACK_SIZE,
                    (void *)                            NULL,
                    (UBaseType_t)                       TASK1_PRIO,
                    (TaskHandle_t *)                    &Task1_Handle);
    xTaskCreate(    (TaskFunction_t)                    task2,
                    (char *)                            "task2",
                    (configSTACK_DEPTH_TYPE)            TASK2_STACK_SIZE,
                    (void *)                            NULL,
                    (UBaseType_t)                       TASK2_PRIO,
                    (TaskHandle_t *)                    &Task2_Handle);
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

/**
 * @brief       
 *
 * @param       pvParameters
 */
void task1(void *pvParameters)
{
    uint32_t task1_run_num = 1;
    while (1)
    {
        taskENTER_CRITICAL();/*��ֹʱ��Ƭ������ӡ�����*/
        printf("����1���д�����%d\n", task1_run_num++);
        taskEXIT_CRITICAL();
        delay_ms(10);//����os��ʱ�����ᷢ�������л�Ӱ��ʵ��Ч����������ʹ��ԭʼ��ʱ����
    }
}

/**
 * @brief       
 * 
 * @param       pvParameters 
 */
void task2(void *pvParameters)
{
    uint32_t task2_run_num = 1;
    while (1)
    {
        taskENTER_CRITICAL(); /*��ֹʱ��Ƭ������ӡ�����*/
        printf("����2���д�����%d\n", task2_run_num++);
        taskEXIT_CRITICAL();
        delay_ms(10);
    }
}
