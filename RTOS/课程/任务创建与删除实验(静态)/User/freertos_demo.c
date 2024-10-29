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
#include "bsp_led.h"
#include "bsp_key.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 *   用户分配空闲任务堆栈及TCB内存
 */
StaticTask_t IdleTaskTCB;
StackType_t IdleTaskStack[configMINIMAL_STACK_SIZE];

/**
 * @brief		静态创建任务未创建函数，用以静态分配空闲任务堆栈及TCB内存
 * @param
 * @retval
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
								   StackType_t **ppxIdleTaskStackBuffer,
								   uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &IdleTaskTCB;
	*ppxIdleTaskStackBuffer = IdleTaskStack;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/**
 *   用户分配空闲任务堆栈及TCB内存
 */
StaticTask_t TimerTaskTCB;
StackType_t TimerTaskStack[configTIMER_TASK_STACK_DEPTH];

/**
 * @brief		静态创建任务时未创建函数（已经宏定义可选是否定义该函数），用以分配软件定时器任务堆栈及TCB内存
 * @param
 * @retval
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
									StackType_t **ppxTimerTaskStackBuffer,
									uint32_t *pulTimerTaskStackSize)
{
	*ppxTimerTaskTCBBuffer = &TimerTaskTCB;
	*ppxTimerTaskStackBuffer = TimerTaskStack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/**
 * 起始任务相关宏定义，包括任务名、任务堆栈、任务优先级、任务句柄
 */
#define StartTask_StackSize 128
#define StartTask_Prio 1
TaskHandle_t StartTask_Handler;
StaticTask_t StartTaskTCB;
StackType_t StartTaskStack[StartTask_StackSize];
void StartTask(void *pvParameters);

/**
 * 任务1相关宏定义，包括任务名、任务堆栈、任务优先级、任务句柄
 */
#define Task1_StackSize 128
#define Task1_Prio 2
TaskHandle_t Task1_Handler;
StaticTask_t Task1TCB;
StackType_t Task1Stack[Task1_StackSize];
void Task1(void *pvParameters);

/**
 * 任务2相关宏定义，包括任务堆栈、任务优先级、任务句柄
 */
#define Task2_StackSize 128
#define Task2_Prio 3
TaskHandle_t Task2_Handler;
StaticTask_t Task2TCB;
StackType_t Task2Stack[Task2_StackSize];
void Task2(void *pvParameters);

/**
 * 任务3相关宏定义，包括任务堆栈、任务优先级、任务句柄
 */
#define Task3_StackSize 128
#define Task3_Prio 4
TaskHandle_t Task3_Handler;
StaticTask_t Task3TCB;
StackType_t Task3Stack[Task3_StackSize];
void Task3(void *pvParameters);

/**
 * @brief  以静态方式创建一个开始任务
 * @param  无
 * @retval 无
 */
void FreeRTOS_demo(void)
{
	StartTask_Handler = xTaskCreateStatic((TaskFunction_t)StartTask,	  /*任务函数*/
										  (char *)"StartTask",			  /*任务名*/
										  (uint32_t)StartTask_StackSize,  /*任务堆栈大小*/
										  (void *)NULL,					  /*任务输入参数*/
										  (UBaseType_t)StartTask_Prio,	  /*任务优先级*/
										  (StackType_t *)StartTaskStack,  /*任务堆栈*/
										  (StaticTask_t *)&StartTaskTCB); /*TCB*/
	vTaskStartScheduler();
}

/**
 * @brief  起始任务，创建任务1.2.3后删除
 * @param  无
 * @retval 无
 */
void StartTask(void *pvParameters)
{
	taskENTER_CRITICAL(); /* 进入临界区 */

	Task1_Handler = xTaskCreateStatic((TaskFunction_t)Task1,	  /*任务函数*/
									  (char *)"Task1",			  /*任务名*/
									  (uint32_t)Task1_StackSize,  /*任务堆栈大小*/
									  (void *)NULL,				  /*任务输入参数*/
									  (UBaseType_t)Task1_Prio,	  /*任务优先级*/
									  (StackType_t *)Task1Stack,  /*任务堆栈*/
									  (StaticTask_t *)&Task1TCB); /*TCB*/

	Task2_Handler = xTaskCreateStatic((TaskFunction_t)Task2,	  /*任务函数*/
									  (char *)"Task2",			  /*任务名*/
									  (uint32_t)Task2_StackSize,  /*任务堆栈大小*/
									  (void *)NULL,				  /*任务输入参数*/
									  (UBaseType_t)Task2_Prio,	  /*任务优先级*/
									  (StackType_t *)Task2Stack,  /*任务堆栈*/
									  (StaticTask_t *)&Task2TCB); /*TCB*/

	Task3_Handler = xTaskCreateStatic((TaskFunction_t)Task3,	  /*任务函数*/
									  (char *)"Task3",			  /*任务名*/
									  (uint32_t)Task3_StackSize,  /*任务堆栈大小*/
									  (void *)NULL,				  /*任务输入参数*/
									  (UBaseType_t)Task3_Prio,	  /*任务优先级*/
									  (StackType_t *)Task3Stack,  /*任务堆栈*/
									  (StaticTask_t *)&Task3TCB); /*TCB*/

	vTaskDelete(NULL);
	taskEXIT_CRITICAL(); /* 退出临界区 */
}

/**
 * @brief  任务1
 * @param  无
 * @retval 无
 */
void Task1(void *pvParameters)
{
	while (1)
	{
		printf("任务1正在运行\n");
		LEDG_TOGGLE;
		vTaskDelay(1000);
	}
}

/**
 * @brief  任务2
 * @param  无
 * @retval 无
 */
void Task2(void *pvParameters)
{
	while (1)
	{
		printf("任务2正在运行\n");
		LEDB_TOGGLE;
		vTaskDelay(1000);
	}
}

/**
 * @brief  任务3,按键1被按下，删除任务1，按键2被按下，删除任务二
 * @param  无
 * @retval 无
 */
void Task3(void *pvParameters)
{
	uint8_t val;
	while (1)
	{
		val = Key_CheckPress(0);
		printf("任务3正在运行\n");
		if (val == 1)
		{
			vTaskDelete(Task1_Handler);
			printf("删除任务1\n");
			//			/*防止任务重复删除造成错误*/
			//			if(Task1_Handler!=NULL)
			//			{
			//				vTaskDelete(Task1_Handler);
			//				printf("删除任务1\n");
			//				Task1_Handler=NULL;
			//			}
		}
		if (val == 2)
		{
			vTaskDelete(Task2_Handler);
			printf("删除任务2\n");
			/*防止任务重复删除造成错误*/
			//			if(Task2_Handler!=NULL)
			//			{
			//				vTaskDelete(Task2_Handler);
			//				printf("删除任务2\n");
			//				Task2_Handler=NULL;
			//			}
		}
		vTaskDelay(10); /*这里可以保证其他任务的运行*/
	}
}
