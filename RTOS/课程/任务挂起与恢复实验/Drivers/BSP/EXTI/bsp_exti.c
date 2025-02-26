#include "bsp_exti.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "task.h"

static void NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief       EXTI���������GPIO��ʼ��
 * 
 */
static void EXTI_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_GPIO_APBxCLK_FUN(EXTI_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = EXTI_GPIO_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(EXTI_GPIO_Port, &GPIO_InitStructure);
}

static void EXTI_AFIO_Configure(void)
{
    EXTI_AFIO_APBxCLK_FUN(EXTI_AFIO_CLK, ENABLE);
    GPIO_EXTILineConfig(EXTI_AFIO_PortSource, EXTI_AFIO_PinSource);
}

static void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Linex;/*�жϡ��¼�ѡ��*/
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;/*EXTIʹ��*/
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;/*EXTIģʽ*/
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;/*��������*/
    EXTI_Init(&EXTI_InitStructure);
}

void ExtiInit(void)
{
    NVIC_Configure();
    EXTI_GPIO_Configure();
    EXTI_AFIO_Configure();
    EXTI_Configure();
}

/**
 * @brief       ����FreeRTOS�жϼ����������Ч��
 * 
 */
extern TaskHandle_t    Task1_Handle;
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Linex))
    {
        BaseType_t xYieldRequired;
        xYieldRequired = xTaskResumeFromISR(Task1_Handle);
        if (xYieldRequired == pdTRUE)
        {
            portYIELD_FROM_ISR(xYieldRequired);
        }
        printf("����1���ж��б��ָ�\n");
        EXTI_ClearFlag(EXTI_Linex);
    }
}
