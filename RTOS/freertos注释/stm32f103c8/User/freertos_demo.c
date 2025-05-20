/**
 * *****************************************************************************
 * @file        freertos_demo.c
 * @brief
 * @author
 * @date        2024-11-15
 * @version     0.1
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * 实验平台:    野火F103VET6
 *
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "FreeRTOS_demo.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "FreeRTOS.h"
#include "task.h"
/*-----------------------------------macro------------------------------------*/
/*开始任务相关定义，包括函数声明、任务堆栈大小、优先级*/
void start_task(void *pvParameters);
#define START_STACK_SIZE 128
#define START_TASK_PRIO 1
TaskHandle_t StartTask_Handle;

/*任务1相关定义，包括函数声明、任务堆栈大小、优先级*/
void task1(void *pvParameters);
#define TASK1_STACK_SIZE 128
#define TASK1_PRIO 1
TaskHandle_t Task1_Handle;

/*任务2相关定义，包括函数声明、任务堆栈大小、优先级*/
void task2(void *pvParameters);
#define TASK2_STACK_SIZE 128
#define TASK2_PRIO 2
TaskHandle_t Task2_Handle;

/*任务3相关定义，包括函数声明、任务堆栈大小、优先级*/
void task3(void *pvParameters);
#define TASK3_STACK_SIZE 128
#define TASK3_PRIO 3
TaskHandle_t Task3_Handle;
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
    vTaskStartScheduler();
}
/*------------------------------------test------------------------------------*/

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

void task1(void *pvParameters)
{
    uint16_t counter = 0;
    while (1)
    {
        LEDG_TOGGLE;
        printf("task1:%d\n", counter++);
        vTaskDelay(500);
    }
}

void task2(void *pvParameters)
{
    uint16_t counter = 0;
    while (1)
    {
        LEDB_TOGGLE;
        printf("task2:%d\n", counter++);
        vTaskDelay(500);
    }
}

void task3(void *pvParameters)
{
    while (1)
    {
        uint8_t status = Key_CheckPress(0);
        switch (status)
        {
        case 1:
        {
            vTaskSuspend(Task1_Handle);
            printf("任务1被挂起\n");
            break;
        }
        case 2:
        {
            vTaskResume(Task1_Handle);
            printf("任务1被恢复\n");
            break;
        }
        default:
        {
            break;
        }
        }
        vTaskDelay(10);
    }
}
