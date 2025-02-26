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
 * ʵ��ƽ̨:    Ұ��F103VET6
 *
 * *****************************************************************************
 */


/*----------------------------------include-----------------------------------*/
#include "FreeRTOS_demo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_systick.h"
/*-----------------------------------macro------------------------------------*/
/*��ʼ������ض��壬�������������������ջ��С�����ȼ�*/
void start_task(void *pvParameters);
#define         START_STACK_SIZE        128
#define         START_TASK_PRIO         1
TaskHandle_t    StartTask_Handle;

/*����1��ض��壬�������������������ջ��С�����ȼ�*/
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
 * @brief       ��������
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
 * @brief       5s�ر��жϣ�5s�����жϣ���ΪTIMx1�ж����ȼ�����FreeRTOS��͹����ж����ȼ������޷��ر�TIMx1�ж�.���ɹر�TIMx2�ж�
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
      printf("�ر��ж�\n");
      delay_ms(3000); /*���ﲻʹ��freertos�ж�����ΪvTaskDelay����ÿ�ʼ�жϺ���(���˳��ٽ��������е��ÿ��жϣ�����0���������ж�)*/
      printf("�����ж�\n");
      portENABLE_INTERRUPTS();
    }
    vTaskDelay(500);
  }
}



