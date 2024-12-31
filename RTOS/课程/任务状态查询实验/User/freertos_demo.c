/**
 * *****************************************************************************
 * @file        freertos_demo.c
 * @brief       时间片调度实验，在配置文件中将时间片（systick中断周期）改为50ms，使能时间片
 *          调度，通过串口查看任务运行次数。
 *              注：由于程序执行需要一定时间，故在一个时间片中，每个任务运行4-5次，而不是标准的5次
 * @author      
 * @date        2024-12-18
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:
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
char pcListBuffer[500];
void task2(void *pvParameters)
{
    uint8_t i;
    UBaseType_t task2_proi;
    UBaseType_t task_num;
    TaskStatus_t task_staatus_structure;
    UBaseType_t stack_min_value;
    eTaskState task_state;
    // char pcListBuffer[50];
    /*获取优先级*/
    printf("***************************优先级********************************\n");
    task2_proi = uxTaskPriorityGet(Task2_Handle);
    printf("task2_proi=%ld\n", task2_proi);
    /*优先级设置*/
    printf("\n*************************优先级设置******************************\n");
    vTaskPrioritySet(Task2_Handle, 20);
    task2_proi = uxTaskPriorityGet(Task2_Handle);
    printf("task2_proi=%ld\n", task2_proi);
    printf("\n*************************任务总数量******************************\n");
    task_num = uxTaskGetNumberOfTasks();/*5个任务（包括任务调度函数中创建的空闲任务与软件定时器任务（如果使能的话）*/
    printf("task_num=%ld\n", task_num);
    /*获取所有任务状态*/
    printf("\n*************************所有任务状态******************************\n");
    TaskStatus_t task_status_array[task_num];
    uxTaskGetSystemState(task_status_array, task_num, NULL);
    printf("pcTaskName\t\txTaskNumber\teCurrentState\tuxCurrentPriority\n");
    for (i = 0; i < task_num;i++)
    {
        printf("%-10s\t\t" ,task_status_array[i].pcTaskName);
        printf("%-12ld\t\t" ,task_status_array[i].xTaskNumber);
        printf("%-14d\t\t" ,task_status_array[i].eCurrentState);
        printf("%-18ld\n" ,task_status_array[i].uxCurrentPriority);
    }
    /*获取单个任务状态*/
    printf("\n*************************单个任务状态******************************\n");
    vTaskGetInfo(Task2_Handle, &task_staatus_structure, pdFALSE, eInvalid);
    printf("pcTaskName\t\txTaskNumber\teCurrentState\tuxCurrentPriority\n");
    printf("%-10s\t\t%-12ld\t\t%-14d\t\t%-18ld\n",
           task_staatus_structure.pcTaskName,
           task_staatus_structure.xTaskNumber,
           task_staatus_structure.eCurrentState,
           task_staatus_structure.uxCurrentPriority);
    /*获取当前任务句柄*/
    printf("\n*************************获取当前任务句柄******************************\n");
    TaskHandle_t task_handle = xTaskGetCurrentTaskHandle();
    printf("%d\n", (int)task_handle);
    printf("%d\n", (int)Task2_Handle);
    /*获取指定任务句柄*/
    printf("\n*************************获取指定任务句柄******************************\n");
    task_handle = xTaskGetHandle("task1");
    printf("%d\n", (int)task_handle);
    printf("%d\n", (int)Task1_Handle);
    /*获取任务堆栈最小值*/
    printf("\n*************************任务堆栈最小值******************************\n");
    stack_min_value = uxTaskGetStackHighWaterMark(Task2_Handle);
    printf("stack_min_value=%ld\n", stack_min_value);
    /*获取指定任务状态*/
    printf("\n*************************获取指定任务状态******************************\n");
    task_state = eTaskGetState(Task2_Handle);
    printf("task_state=%d", task_state);
    /*以表格形式获取任务状态*/
    printf("\n*************************以表格形式获取任务状态******************************\n");
    vTaskList(pcListBuffer);
    printf("%s\r\n", pcListBuffer);
    while (1)
    {
        vTaskDelay(500);
    }
}
