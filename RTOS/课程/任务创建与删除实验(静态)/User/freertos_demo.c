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
#include "bsp_led.h"
#include "bsp_key.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 *   �û�������������ջ��TCB�ڴ�
 */
StaticTask_t IdleTaskTCB;
StackType_t IdleTaskStack[configMINIMAL_STACK_SIZE];

/**
 * @brief		��̬��������δ�������������Ծ�̬������������ջ��TCB�ڴ�
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
 *   �û�������������ջ��TCB�ڴ�
 */
StaticTask_t TimerTaskTCB;
StackType_t TimerTaskStack[configTIMER_TASK_STACK_DEPTH];

/**
 * @brief		��̬��������ʱδ�����������Ѿ��궨���ѡ�Ƿ���ú����������Է��������ʱ�������ջ��TCB�ڴ�
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
 * ��ʼ������غ궨�壬�����������������ջ���������ȼ���������
 */
#define StartTask_StackSize 128
#define StartTask_Prio 1
TaskHandle_t StartTask_Handler;
StaticTask_t StartTaskTCB;
StackType_t StartTaskStack[StartTask_StackSize];
void StartTask(void *pvParameters);

/**
 * ����1��غ궨�壬�����������������ջ���������ȼ���������
 */
#define Task1_StackSize 128
#define Task1_Prio 2
TaskHandle_t Task1_Handler;
StaticTask_t Task1TCB;
StackType_t Task1Stack[Task1_StackSize];
void Task1(void *pvParameters);

/**
 * ����2��غ궨�壬���������ջ���������ȼ���������
 */
#define Task2_StackSize 128
#define Task2_Prio 3
TaskHandle_t Task2_Handler;
StaticTask_t Task2TCB;
StackType_t Task2Stack[Task2_StackSize];
void Task2(void *pvParameters);

/**
 * ����3��غ궨�壬���������ջ���������ȼ���������
 */
#define Task3_StackSize 128
#define Task3_Prio 4
TaskHandle_t Task3_Handler;
StaticTask_t Task3TCB;
StackType_t Task3Stack[Task3_StackSize];
void Task3(void *pvParameters);

/**
 * @brief  �Ծ�̬��ʽ����һ����ʼ����
 * @param  ��
 * @retval ��
 */
void FreeRTOS_demo(void)
{
	StartTask_Handler = xTaskCreateStatic((TaskFunction_t)StartTask,	  /*������*/
										  (char *)"StartTask",			  /*������*/
										  (uint32_t)StartTask_StackSize,  /*�����ջ��С*/
										  (void *)NULL,					  /*�����������*/
										  (UBaseType_t)StartTask_Prio,	  /*�������ȼ�*/
										  (StackType_t *)StartTaskStack,  /*�����ջ*/
										  (StaticTask_t *)&StartTaskTCB); /*TCB*/
	vTaskStartScheduler();
}

/**
 * @brief  ��ʼ���񣬴�������1.2.3��ɾ��
 * @param  ��
 * @retval ��
 */
void StartTask(void *pvParameters)
{
	taskENTER_CRITICAL(); /* �����ٽ��� */

	Task1_Handler = xTaskCreateStatic((TaskFunction_t)Task1,	  /*������*/
									  (char *)"Task1",			  /*������*/
									  (uint32_t)Task1_StackSize,  /*�����ջ��С*/
									  (void *)NULL,				  /*�����������*/
									  (UBaseType_t)Task1_Prio,	  /*�������ȼ�*/
									  (StackType_t *)Task1Stack,  /*�����ջ*/
									  (StaticTask_t *)&Task1TCB); /*TCB*/

	Task2_Handler = xTaskCreateStatic((TaskFunction_t)Task2,	  /*������*/
									  (char *)"Task2",			  /*������*/
									  (uint32_t)Task2_StackSize,  /*�����ջ��С*/
									  (void *)NULL,				  /*�����������*/
									  (UBaseType_t)Task2_Prio,	  /*�������ȼ�*/
									  (StackType_t *)Task2Stack,  /*�����ջ*/
									  (StaticTask_t *)&Task2TCB); /*TCB*/

	Task3_Handler = xTaskCreateStatic((TaskFunction_t)Task3,	  /*������*/
									  (char *)"Task3",			  /*������*/
									  (uint32_t)Task3_StackSize,  /*�����ջ��С*/
									  (void *)NULL,				  /*�����������*/
									  (UBaseType_t)Task3_Prio,	  /*�������ȼ�*/
									  (StackType_t *)Task3Stack,  /*�����ջ*/
									  (StaticTask_t *)&Task3TCB); /*TCB*/

	vTaskDelete(NULL);
	taskEXIT_CRITICAL(); /* �˳��ٽ��� */
}

/**
 * @brief  ����1
 * @param  ��
 * @retval ��
 */
void Task1(void *pvParameters)
{
	while (1)
	{
		printf("����1��������\n");
		LEDG_TOGGLE;
		vTaskDelay(1000);
	}
}

/**
 * @brief  ����2
 * @param  ��
 * @retval ��
 */
void Task2(void *pvParameters)
{
	while (1)
	{
		printf("����2��������\n");
		LEDB_TOGGLE;
		vTaskDelay(1000);
	}
}

/**
 * @brief  ����3,����1�����£�ɾ������1������2�����£�ɾ�������
 * @param  ��
 * @retval ��
 */
void Task3(void *pvParameters)
{
	uint8_t val;
	while (1)
	{
		val = Key_CheckPress(0);
		printf("����3��������\n");
		if (val == 1)
		{
			vTaskDelete(Task1_Handler);
			printf("ɾ������1\n");
			//			/*��ֹ�����ظ�ɾ����ɴ���*/
			//			if(Task1_Handler!=NULL)
			//			{
			//				vTaskDelete(Task1_Handler);
			//				printf("ɾ������1\n");
			//				Task1_Handler=NULL;
			//			}
		}
		if (val == 2)
		{
			vTaskDelete(Task2_Handler);
			printf("ɾ������2\n");
			/*��ֹ�����ظ�ɾ����ɴ���*/
			//			if(Task2_Handler!=NULL)
			//			{
			//				vTaskDelete(Task2_Handler);
			//				printf("ɾ������2\n");
			//				Task2_Handler=NULL;
			//			}
		}
		vTaskDelay(10); /*������Ա�֤�������������*/
	}
}
