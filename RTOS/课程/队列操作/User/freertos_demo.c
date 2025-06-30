/**
 * *****************************************************************************
 * @file        freertos_demo.c
 * @brief       ���в���ʵ��
 * @author
 * @date        2024-12-18
 * @version     0.1
 * @copyright
 * *****************************************************************************
 * @attention
 *
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
#include "bsp_key.h"
#include "queue.h"
/*-----------------------------------macro------------------------------------*/
/*��ʼ������ض��壬�������������������ջ��С�����ȼ�*/
void start_task(void *pvParameters);
#define START_STACK_SIZE 128
#define START_TASK_PRIO 1
TaskHandle_t StartTask_Handle;

/*����1��ض��壬�������������������ջ��С�����ȼ�*/
void task1(void *pvParameters);
#define TASK1_STACK_SIZE 128
#define TASK1_PRIO 2
TaskHandle_t Task1_Handle;

/*����2��ض��壬�������������������ջ��С�����ȼ�*/
void task2(void *pvParameters);
#define TASK2_STACK_SIZE 128
#define TASK2_PRIO 3
TaskHandle_t Task2_Handle;

/*����2��ض��壬�������������������ջ��С�����ȼ�*/
void task3(void *pvParameters);
#define TASK3_STACK_SIZE 128
#define TASK3_PRIO 4
TaskHandle_t Task3_Handle;

/*����1�����С����*/
QueueHandle_t xQueue1;
/*����2����Ŵ�����*/
QueueHandle_t xQueue2;
/*����1*/
uint8_t data1 = 0;
/*����2*/
uint8_t data_big[100] = {"�����Ŵ�����875164hudsibfa"};

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*----------------------------------function----------------------------------*/
void FreeRTOS_demo(void)
{
    xTaskCreate((TaskFunction_t)start_task,
                (char *)"start_task",
                (configSTACK_DEPTH_TYPE)START_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIO,
                (TaskHandle_t *)&StartTask_Handle);
    xQueue1 = xQueueCreate(1, sizeof(uint8_t));
    xQueue2 = xQueueCreate(1, sizeof(data_big));
    if (xQueue1 != NULL || xQueue2 != NULL)
    {
        printf("���д����ɹ�\r\n");
    }
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
    xTaskCreate((TaskFunction_t)task1,
                (char *)"task1",
                (configSTACK_DEPTH_TYPE)TASK1_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK1_PRIO,
                (TaskHandle_t *)&Task1_Handle);
    xTaskCreate((TaskFunction_t)task2,
                (char *)"task2",
                (configSTACK_DEPTH_TYPE)TASK2_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK2_PRIO,
                (TaskHandle_t *)&Task2_Handle);
    xTaskCreate((TaskFunction_t)task3,
                (char *)"task3",
                (configSTACK_DEPTH_TYPE)TASK3_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK3_PRIO,
                (TaskHandle_t *)&Task3_Handle);
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

/**
 * @brief  �����°���1ʱ������ֵ���Ƶ�����1�������°���2ʱ����data_big���Ƶ�����2
 * @note
 * @param  pvParameters
 * @retval ��
 */
void task1(void *pvParameters)
{
    BaseType_t xResult;
    while (1)
    {
        uint8_t key_val = Key_CheckPress(0);
        if (key_val == 1)
        {
            data1 = 1;
            xResult = xQueueSend(xQueue1, &data1, portMAX_DELAY);
            if (xResult == pdPASS)
            {
                printf("����1���ͳɹ�\r\n");
            }
        }
        if (key_val == 2)
        {
            xResult = xQueueSend(xQueue2, data_big, portMAX_DELAY);
            if (xResult == pdPASS)
            {
                printf("����2���ͳɹ�\r\n");
            }
        }
        vTaskDelay(100);
    }
}

/**
 * @brief  �Ӷ���1�н������ݣ�����ӡ
 * @note
 * @param  ��
 * @retval ��
 */
void task2(void *pvParameters)
{
    BaseType_t xResult;
    uint8_t data1_buf;
    while (1)
    {
        xResult = xQueueReceive(xQueue1,&data1_buf,portMAX_DELAY);
        if(xResult == pdPASS)
        {
            printf("����1���ճɹ�\r\n");
            printf("data1_buf = %d\r\n",data1_buf);
        }
    }
}

/**
 * @brief  �Ӷ���2�н������ݣ�����ӡ
 * @note
 * @param  pvParameters
 * @retval ��
 */
void task3(void *pvParameters)
{
    BaseType_t xResult;
    uint8_t data_big_buf[100];
    while (1)
    {
        xResult = xQueueReceive(xQueue2,data_big_buf,portMAX_DELAY);
        if(xResult == pdPASS)
        {
            printf("����2���ճɹ�\r\n");
            printf("data_big_buf = %s\r\n",data_big_buf);
        }
    }
}