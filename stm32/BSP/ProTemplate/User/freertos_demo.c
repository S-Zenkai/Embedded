/**
 ****************************************************************************************************
 * @file        freertos.c
 * @author      
 * @version     V1.4
 * @date        2024.10.15
 * @brief       FreeRTOS以动态方式创建任务
 * @license     
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:野火F103VET6
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
  * 起始任务相关宏定义，包括任务名、任务堆栈、任务优先级、任务句柄
  */
#define  StartTask_stack           128
#define  StartTask_prio            1
TaskHandle_t  StartTask_handler;
void StartTask(void* pvParameters);


/**
  * 任务1相关宏定义，包括任务堆栈、任务优先级、任务句柄
  */
#define  Task1_stack           128
#define  Task1_prio            2
TaskHandle_t  Task1_handler;
void Task1(void* pvParameters);

/**
  * 任务2相关宏定义，包括任务堆栈、任务优先级、任务句柄
  */
#define  Task2_stack           128
#define  Task2_prio            3
TaskHandle_t  Task2_handler;
void Task2(void* pvParameters);

/**
  * 任务3相关宏定义，包括任务堆栈、任务优先级、任务句柄
  */
#define  Task3_stack           128
#define  Task3_prio            4
TaskHandle_t  Task3_handler;
void Task3(void* pvParameters);


/**
  * @brief  以动态方式创建一个开始任务
  * @param  无
  * @retval 无
  */
void FreeRTOS_demo(void)
{
	xTaskCreate(  (TaskFunction_t)           StartTask,                /*任务函数*/
                (const char *)             "StartTask",              /*任务名*/
                (configSTACK_DEPTH_TYPE)   StartTask_stack,          /*任务堆栈*/
                (void *)                   NULL,                     /*任务输入参数*/
                (UBaseType_t)              StartTask_prio,           /*任务优先级*/
                (TaskHandle_t *)           &StartTask_handler        /*任务句柄*/ 
						 );
	vTaskStartScheduler();
}

/**
  * @brief  起始任务，创建任务1.2.3后删除
  * @param  无
  * @retval 无
  */
void StartTask(void* pvParameters)
{
	taskENTER_CRITICAL();                                          /* 进入临界区 */
	
	xTaskCreate(  (TaskFunction_t)           Task1,                /*任务函数*/
                (const char *)             "Task1",              /*任务名*/
                (configSTACK_DEPTH_TYPE)   Task1_stack,          /*任务堆栈*/
                (void *)                   NULL,                 /*任务输入参数*/
                (UBaseType_t)              Task1_prio,           /*任务优先级*/
                (TaskHandle_t *)           &Task1_handler        /*任务句柄*/ 
						 );
								
	xTaskCreate(  (TaskFunction_t)           Task2,                /*任务函数*/
                (const char *)             "Task2",              /*任务名*/
                (configSTACK_DEPTH_TYPE)   Task2_stack,          /*任务堆栈*/
                (void *)                   NULL,                 /*任务输入参数*/
                (UBaseType_t)              Task2_prio,           /*任务优先级*/
                (TaskHandle_t *)           &Task2_handler        /*任务句柄*/ 
						 );
								
	xTaskCreate(  (TaskFunction_t)           Task3,                /*任务函数*/
                (const char *)             "Task3",              /*任务名*/
                (configSTACK_DEPTH_TYPE)   Task3_stack,          /*任务堆栈*/
                (void *)                   NULL,                 /*任务输入参数*/
                (UBaseType_t)              Task3_prio,           /*任务优先级*/
                (TaskHandle_t *)           &Task3_handler        /*任务句柄*/ 
						 );
	vTaskDelete(NULL);
	taskEXIT_CRITICAL();                                           /* 退出临界区 */
}

/**
  * @brief  任务1
  * @param  无
  * @retval 无
  */
void Task1(void* pvParameters)
{
	while(1)
	{
		
	}  
}

/**
  * @brief  任务2
  * @param  无
  * @retval 无
  */
void Task2(void* pvParameters)
{
	while(1)
	{
		
	}    
}

/**
  * @brief  任务3,按键1被按下，删除任务1，按键2被按下，删除任务二
  * @param  无
  * @retval 无
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
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.4
 * @date        2022-01-04
 * @brief       FreeRTOS 移植实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
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
/*FreeRTOS配置*/

/* START_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define START_TASK_PRIO 1                   /* 任务优先级 */
#define START_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            StartTask_Handler;  /* 任务句柄 */
void start_task(void *pvParameters);        /* 任务函数 */

/* TASK1 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define TASK1_PRIO      2                   /* 任务优先级 */
#define TASK1_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            Task1Task_Handler;  /* 任务句柄 */
void task1(void *pvParameters);             /* 任务函数 */

/* TASK2 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define TASK2_PRIO      3                   /* 任务优先级 */
#define TASK2_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            Task2Task_Handler;  /* 任务句柄 */
void task2(void *pvParameters);             /* 任务函数 */

/******************************************************************************************************/


/**
 * @brief       FreeRTOS例程入口函数
 * @param       无
 * @retval      无
 */
void freertos_demo(void)
{    
    xTaskCreate((TaskFunction_t )start_task,            /* 任务函数 */
                (const char*    )"start_task",          /* 任务名称 */
                (uint16_t       )START_STK_SIZE,        /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )START_TASK_PRIO,       /* 任务优先级 */
                (TaskHandle_t*  )&StartTask_Handler);   /* 任务句柄 */
    vTaskStartScheduler();
}

/**
 * @brief       start_task
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           /* 进入临界区 */
    /* 创建任务1 */
    xTaskCreate((TaskFunction_t )task1,
                (const char*    )"task1",
                (uint16_t       )TASK1_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK1_PRIO,
                (TaskHandle_t*  )&Task1Task_Handler);
    /* 创建任务2 */
    xTaskCreate((TaskFunction_t )task2,
                (const char*    )"task2",
                (uint16_t       )TASK2_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK2_PRIO,
                (TaskHandle_t*  )&Task2Task_Handler);
    vTaskDelete(StartTask_Handler); /* 删除开始任务 */
    taskEXIT_CRITICAL();            /* 退出临界区 */
}

/**
 * @brief       task1
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void task1(void *pvParameters)
{    
    while(1)
    {
        LEDR_TOGGLE;                                                  /* LED0闪烁 */
        vTaskDelay(1000);                                               /* 延时1000ticks */
    }
}

/**
 * @brief       task2
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void task2(void *pvParameters)
{
    while(1)
    {
        LEDG_TOGGLE;
        vTaskDelay(500);                           /* 延时1000ticks */
    }
}

#endif

