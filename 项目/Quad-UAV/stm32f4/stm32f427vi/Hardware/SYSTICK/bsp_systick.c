#include "bsp_systick.h"




/* ���SYS_SUPPORT_OS������,˵��Ҫ֧��OS��(������UCOS) */
#if SYS_SUPPORT_OS

/* ��ӹ���ͷ�ļ� (FreeRTOS ��Ҫ�õ�) */
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);
static uint16_t g_fac_us = 0; /* us��ʱ������ */
/**
 * @brief systick �жϷ�����,ʹ�� OS ʱ�õ�
 * @param ticks: ��ʱ�Ľ�����
 * @retval ��
 */
void SysTick_Handler(void)
{
    /* OS ��ʼ����,��ִ�������ĵ��ȴ��� */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

/**
 * @brief       ��ʼ���ӳٺ���
 * @param       sysclk: ϵͳʱ��Ƶ��, ��CPUƵ��(HCLK)
 * @retval      ��
 */
void SystickInit(uint16_t sysclk)
{
    uint32_t reload;
    SysTick->CTRL = 0;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    g_fac_us = sysclk;
    reload = sysclk / 8;
    /* ʹ�� configTICK_RATE_HZ ������װ��ֵ
     * configTICK_RATE_HZ �� FreeRTOSConfig.h �ж���
     */
    reload *= 1000000 / configTICK_RATE_HZ;
    /* ɾ�����õ� g_fac_ms ��ش��� */
    SysTick->CTRL |= 1 << 1;
    SysTick->LOAD = reload;
    SysTick->CTRL |= 1 << 0;
}

/**
 * @brief       ��ʱnus
 * @param       nus: Ҫ��ʱ��us��.
 * @note        nusȡֵ��Χ: 0 ~ 477218588(���ֵ��2^32 / g_fac_us @g_fac_us = 9)
 * @retval      ��
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;
    /* ɾ�������� ��C/OS ��������������������Զ��庯�� */
    ticks = nus * g_fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
    /* ɾ�������� ��C/OS ���ڽ���������������Զ��庯�� */
}

/**
 * @brief       ��ʱnms
 * @param       nms: Ҫ��ʱ��ms�� (0< nms <= 65535)
 * @retval      ��
 */
void delay_ms(uint16_t nms)
{
    uint32_t i;
    for (i = 0; i < nms; i++)
    {
        delay_us(1000);
    }
}

#else /* ��ʹ��OSʱ, �����´��� */

volatile uint32_t sysTickCounter = 0;

/**
 * @brief  ����SysTick��ʱ��ÿ1ms�ж�һ��
 * @param  None
 * @retval None
 */
void systick_init(void) 
{
    /* �ر�SysTick,���֮ǰ��״̬ */
    SysTick->CTRL = 0;
    // ����SysTick��ʱ��
    // SysTick_ConfigĬ��ʹ��HCLK��
    // (RCC_Clocks.HCLK_Frequency / 1000) �����1ms��Ҫ�ļ���ֵ
    // SysTick_Config �� CMSIS �ṩ�ĺ��������᣺
    // 1. ���� Reload ֵ
    // 2. ���㵱ǰ����ֵ
    // 3. ����ʱ��ԴΪ HCLK
    // 4. ʹ�� SysTick �ж�
    // 5. ���� SysTick ��ʱ��
    if (SysTick_Config(SystemCoreClock  / 1000)) {
        // �������ʧ�� (���� HCLK Ƶ�ʹ��ߵ��� Reload ֵ������Χ)��������ѭ��
        while (1);
    }
    // // ����SysTick�ж����ȼ� (�����Ҫ�����Ļ�)
    // NVIC_SetPriority(SysTick_IRQn, 0); // ��������Ϊ������ȼ�
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
    sysTickCounter++; // ÿ���жϷ���ʱ����������1
}

/**
 * @brief  ��ȡ��ϵͳ���������ĺ�����
 * @param  None
 * @retval uint32_t ��ǰ�ĺ������ֵ
 */
uint32_t GetTick(void) 
{

    // ֱ�ӷ���ȫ�ּ�������ֵ
    // ��cortex-M4�ϣ���ȡ32λ������ԭ�Ӳ���������ֱ�ӷ���sysTickCounter�ǰ�ȫ��
    return sysTickCounter;
    // ���������ڽ���ȡһ���ɵ���ָ������ı�����ֱ�ӷ���ͨ�����У�
    // return sysTickCounter;
}


void delay_ms(uint32_t nms)
{
    uint32_t startTick = GetTick();
    // ʹ�ò�ֵ�Ƚ������� getTick() ���ܷ��������
    while ((GetTick() - startTick) < nms) 
    {
        /*�ڿ���ʱ��CPU�����͹���ģʽ���ȴ��жϻ��ѣ��ʺϵ�ع�����豸*/
        __WFI(); // ���ܵȴ�
    }
}


/**
 * @brief  ΢�뼶��ʱ����
 * @param  nus: Ҫ��ʱ��΢����
 * @retval None
 * @note   ���� SysTick ������������������ HCLK Ƶ�ʺʹ���ִ��ʱ�䣬΢�뼶��ʱʹ��TIMER���ȸ��ߡ�
 * SystemCoreClock ����׼ȷ��ӳ HCLK Ƶ��(Hz)��
 */
void delay_us(uint32_t nus) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD; // ��ȡSysTick����װ��ֵ

    // ������Ҫ�� SysTick ������ (���� HCLK)
    //SystemCoreClock��ϵͳʱ��Ƶ�ʣ�Ҳ��ÿ���ʱ��������
    // SystemCoreClock / 1000000  �õ�ÿ΢���Ӧ�� HCLK ������
    ticks = nus * (SystemCoreClock / 1000000);

    told = SysTick->VAL; // ��ȡ��ǰ�� SysTick ��������ֵ
    while (1) 
    {
        tnow = SysTick->VAL; // �ٴζ�ȡ��ǰֵ
        if (tnow != told) 
        {
            if (tnow < told) 
            { 
                // �����ݼ� (û�з�������)
                tcnt += told - tnow;
            } 
            else 
            { 
                // �������� (�� 0 ���� reload)
                tcnt += reload - tnow + told;
            }
            told = tnow; // �����ϴε�ֵ
            if (tcnt >= ticks) 
            {
                break; // �Ѵﵽ�򳬹�����Ľ���������ʱ����
            }
        }
    }
}


#endif /* SYS_SUPPORT_OS */
