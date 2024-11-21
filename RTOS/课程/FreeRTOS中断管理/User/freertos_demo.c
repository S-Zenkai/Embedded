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
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_systick.h"
/*-----------------------------------macro------------------------------------*/
/*开始任务相关定义，包括函数声明、任务堆栈大小、优先级*/
void start_task(void *pvParameters);
#define         START_STACK_SIZE        128
#define         START_TASK_PRIO         1
TaskHandle_t    StartTask_Handle;

/*任务1相关定义，包括函数声明、任务堆栈大小、优先级*/
void task1(void *pvParameters);
#define         TASK1_STACK_SIZE        128
#define         TASK1_PRIO              1
TaskHandle_t    Task1_Handle;
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*----------------------------------function----------------------------------*/
void FreeRTOS_demo(void)
{
    xTaskCreate((TaskFunction_t)                start_task,
                (char *)                        "start_task",
                (configSTACK_DEPTH_TYPE)        START_STACK_SIZE,
                (void *)                        NULL,
                (UBaseType_t)                   START_TASK_PRIO,
                (TaskHandle_t *)                &StartTask_Handle);
    vTaskStartScheduler();
}
/*------------------------------------test------------------------------------*/
/**
 * @brief       开启任务
 * 
 * @param       pvParameters 
 */
void start_task(void* pvParameters)
{
    taskENTER_CRITICAL();
    xTaskCreate((TaskFunction_t)                task1,
                (char *)                        "task1",
                (configSTACK_DEPTH_TYPE)        TASK1_STACK_SIZE,
                (void *)                        NULL,
                (UBaseType_t)                   TASK1_PRIO,
                (TaskHandle_t *)                &Task1_Handle);
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

/**
 * @brief       5s关闭中断，5s开启中断，因为TIMx1中断优先级大于FreeRTOS最低管理中断优先级，故无法关闭TIMx1中断.但可关闭TIMx2中断
 *
 * @param       pvParameters
 */
void task1(void *pvParameters)
{
  uint16_t counter=0;
  while(1)
  {
    counter++;
    if (counter > 6)
    {
      counter = 0;
      portDISABLE_INTERRUPTS();
      printf("关闭中断\n");
      delay_ms(3000); /*这里不使用freertos中断是因为vTaskDelay会调用开始中断函数(在退出临界区函数中调用开中断，设置0开启所有中断)*/
      printf("开启中断\n");
      portENABLE_INTERRUPTS();
    }
    vTaskDelay(500);
  }
}



