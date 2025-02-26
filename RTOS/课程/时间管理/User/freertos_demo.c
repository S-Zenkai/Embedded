/**
 * *****************************************************************************
 * @file        freertos_demo.c
 * @brief       任务时间统计实验,使用前需先定义portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()函数和 
                portGET_RUN_TIME_COUNTER_VALUE()（第二个是个变量）（在bsp_tim.c中定义）
 * @author      
 * @date        2024-12-18
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 							不知道为什么，任务时间统计函数需要加入按键检查才能起作用。
 *							在有些串口助手中，打印的任务时间统计表格格式不太对，可不理会
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
/*开始任务相关定义，包括函数声明、任务堆栈大小、优先级*/
void start_task(void *pvParameters);
#define             START_STACK_SIZE            128
#define             START_TASK_PRIO             1
TaskHandle_t        StartTask_Handle;

/*任务1相关定义，包括函数声明、任务堆栈大小、优先级*/
void task1(void *pvParameters);
#define             TASK1_STACK_SIZE            128
#define             TASK1_PRIO                  2
TaskHandle_t        Task1_Handle;

/*任务2相关定义，包括函数声明、任务堆栈大小、优先级*/
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
 * @brief       开启任务
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
 * @note				当task1优先级<task2优先级时，LEDB翻转时间可能大于520ms，原因是task2优先级高会抢占task1的运行，
								当task2运行至delay(20)时并不会发生任务切换，若task2的20ms未运行完但task1阻塞时间运行完时也不会切换
								至任务1运行，导致任务1阻塞时间延长
 * @param       pvParameters
 */
void task1(void *pvParameters)
{
    while(1)
    {
        LEDB_TOGGLE;
				delay_ms(20);/*当task1优先级<task2优先级时，LEDB翻转时间可能大于520ms，原因是：*/
        vTaskDelay(500);
    }
}

/**
 * @brief       
 * @note				同样的，当task1优先级>task2优先级时，LEDB翻转时间可能大于500ms，原因同上
 * @param       pvParameters 
 */
void task2(void *pvParameters)
{
    TickType_t  xLastWakeTime;
    const TickType_t xFrequency = 500;
    while(1)
    {
        xLastWakeTime = xTaskGetTickCount();
        LEDR_TOGGLE;
        delay_ms(20);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
