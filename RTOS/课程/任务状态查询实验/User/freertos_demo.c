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
    /*��ȡ���ȼ�*/
    printf("***************************���ȼ�********************************\n");
    task2_proi = uxTaskPriorityGet(Task2_Handle);
    printf("task2_proi=%ld\n", task2_proi);
    /*���ȼ�����*/
    printf("\n*************************���ȼ�����******************************\n");
    vTaskPrioritySet(Task2_Handle, 20);
    task2_proi = uxTaskPriorityGet(Task2_Handle);
    printf("task2_proi=%ld\n", task2_proi);
    printf("\n*************************����������******************************\n");
    task_num = uxTaskGetNumberOfTasks();/*5�����񣨰���������Ⱥ����д����Ŀ��������������ʱ���������ʹ�ܵĻ���*/
    printf("task_num=%ld\n", task_num);
    /*��ȡ��������״̬*/
    printf("\n*************************��������״̬******************************\n");
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
    /*��ȡ��������״̬*/
    printf("\n*************************��������״̬******************************\n");
    vTaskGetInfo(Task2_Handle, &task_staatus_structure, pdFALSE, eInvalid);
    printf("pcTaskName\t\txTaskNumber\teCurrentState\tuxCurrentPriority\n");
    printf("%-10s\t\t%-12ld\t\t%-14d\t\t%-18ld\n",
           task_staatus_structure.pcTaskName,
           task_staatus_structure.xTaskNumber,
           task_staatus_structure.eCurrentState,
           task_staatus_structure.uxCurrentPriority);
    /*��ȡ��ǰ������*/
    printf("\n*************************��ȡ��ǰ������******************************\n");
    TaskHandle_t task_handle = xTaskGetCurrentTaskHandle();
    printf("%d\n", (int)task_handle);
    printf("%d\n", (int)Task2_Handle);
    /*��ȡָ��������*/
    printf("\n*************************��ȡָ��������******************************\n");
    task_handle = xTaskGetHandle("task1");
    printf("%d\n", (int)task_handle);
    printf("%d\n", (int)Task1_Handle);
    /*��ȡ�����ջ��Сֵ*/
    printf("\n*************************�����ջ��Сֵ******************************\n");
    stack_min_value = uxTaskGetStackHighWaterMark(Task2_Handle);
    printf("stack_min_value=%ld\n", stack_min_value);
    /*��ȡָ������״̬*/
    printf("\n*************************��ȡָ������״̬******************************\n");
    task_state = eTaskGetState(Task2_Handle);
    printf("task_state=%d", task_state);
    /*�Ա����ʽ��ȡ����״̬*/
    printf("\n*************************�Ա����ʽ��ȡ����״̬******************************\n");
    vTaskList(pcListBuffer);
    printf("%s\r\n", pcListBuffer);
    while (1)
    {
        vTaskDelay(500);
    }
}
