/**
 * *****************************************************************************
 * @file        freertos_demo.c
 * @brief       ����ʱ��ͳ��ʵ��,ʹ��ǰ���ȶ���portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()������ 
                portGET_RUN_TIME_COUNTER_VALUE()���ڶ����Ǹ�����������bsp_tim.c�ж��壩
 * @author      
 * @date        2024-12-18
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 							��֪��Ϊʲô������ʱ��ͳ�ƺ�����Ҫ���밴�������������á�
 *							����Щ���������У���ӡ������ʱ��ͳ�Ʊ���ʽ��̫�ԣ��ɲ����
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
#include "bsp_mm.h"
#include "bsp_key.h"
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
#define             TASK2_PRIO                  3
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
    while (1)
    {
        LEDG_TOGGLE;
        vTaskDelay(500);
    }
}

/**
 * @brief       
 * 
 * @param       pvParameters 
 */
#define uxBufferLength   300
char pcWriteBuffer[300];
void task2(void *pvParameters)
{
    uint8_t key;
    // char *pcWriteBuffer = NULL;
    // pcWriteBuffer = My_Mallco(300);
    vTaskGetRunTimeStats(pcWriteBuffer);
    printf("%s\r\n", pcWriteBuffer);
    // My_Free(pcWriteBuffer);
    while (1)
    {
        key = Key_CheckPress(0);
        if(key==1)
        {
					printf("%s\r\n", pcWriteBuffer);
            vTaskGetRunTimeStats(pcWriteBuffer);
            printf("%s\r\n", pcWriteBuffer);
        }
        // printf("�������������");
        vTaskDelay(10);
    }
}
